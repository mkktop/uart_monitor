#include "app/app_http_serve.h"
#include "esp_http_server.h"
#include "driver/uart.h"

static const char *TAG = "app_wifi_web"; // 日志标签ss
httpd_handle_t server = NULL;     // HTTP服务器句柄
static int current_baudrate = 115200; //默认波特率
static TaskHandle_t uart_task_handle = NULL; // UART任务句柄
static bool ws_connected = false; // WebSocket 连接状态
// 嵌入HTML文件到固件
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

static void wifi_event_handler(void *arg, esp_event_base_t event_base,int32_t event_id, void *event_data);



/// @brief 串口读取任务
/// @param pvParameters 
static void uart_read_task(void *pvParameters) {
    httpd_handle_t server = (httpd_handle_t)pvParameters;
    uint8_t *data = (uint8_t*) malloc(RD_BUF_SIZE + 1);
    if (!data) {
        ESP_LOGE(TAG, "Failed to allocate UART read buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, RD_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            
            // 将接收到的数据转换为可显示的格式
            char ascii_buffer[RD_BUF_SIZE + 1];
            for (int i = 0; i < len; i++) {
                ascii_buffer[i] = (data[i] >= 32 && data[i] <= 126) ? data[i] : '.';
            }
            ascii_buffer[len] = '\0';
            
            // 发送到 WebSocket 客户端
            send_ws_data(server, "serial_data", ascii_buffer);
            
            ESP_LOGI(TAG, "Received %d bytes: %s", len, ascii_buffer);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(data);
    vTaskDelete(NULL);
}

/// @brief 串口初始化
/// @param baudrate 波特率
void init_uart(int baudrate) {
    if (uart_is_driver_installed(UART_NUM)) {
        ESP_LOGI(TAG, "UART driver already installed, reconfiguring...");
        // 先卸载驱动
        uart_driver_delete(UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(100)); // 给系统一些时间清理
    }
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART0_TX, UART0_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    
    current_baudrate = baudrate;
    ESP_LOGI(TAG, "UART initialized with baudrate: %d", baudrate);
    // 启动 UART 读取任务
    xTaskCreate(uart_read_task, "uart_read_task", 4096, server, 5, &uart_task_handle);
}


/**
 * @brief 处理根路径 ("/") 的 HTTP GET 请求，返回嵌入的 HTML 文件内容
 * 
 * 该函数会将响应的内容类型设置为 "text/html"，并将预先嵌入到固件中的
 * HTML 文件内容发送给客户端。
 * 
 * @param req 指向 httpd_req_t 结构体的指针，包含 HTTP 请求的相关信息
 * @return esp_err_t 操作结果，ESP_OK 表示成功，其他值表示失败
 */
static esp_err_t root_handler(httpd_req_t *req)
{
    // 设置 HTTP 响应的内容类型为 HTML
    httpd_resp_set_type(req, "text/html");
    // 发送预先嵌入到固件中的 HTML 文件内容给客户端
    // index_html_start 为 HTML 文件的起始地址，index_html_end 为结束地址
    // 通过计算两者差值得到 HTML 文件的长度
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_handler,
    .user_ctx = NULL
};


/// @brief 发送 WebSocket 数据
/// @param server 
/// @param type 
/// @param value 
void send_ws_data(httpd_handle_t server, const char *type, const char *value) {
    if (!ws_connected || !server) return;

    char msg[256];
    snprintf(msg, sizeof(msg), "{\"type\":\"%s\",\"value\":\"%s\"}", type, value);
    
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)msg;
    ws_pkt.len = strlen(msg);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // 发送到所有连接的客户端
    int client_fds[CONFIG_LWIP_MAX_LISTENING_TCP];
    size_t client_count = CONFIG_LWIP_MAX_LISTENING_TCP;

    esp_err_t ret = httpd_get_client_list(server, &client_count, client_fds);
    if (ret == ESP_OK) {
        for (size_t i = 0; i < client_count; i++) {
            httpd_ws_send_frame_async(server, client_fds[i], &ws_pkt);
        }
    }
}


/// @brief 处理接收到的 WebSocket 消息
/// @param req 
/// @return 
static esp_err_t websocket_handler(httpd_req_t *req){
    // 升级为 WebSocket 连接
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket connection established");
        ws_connected = true;
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    // 设置最大帧大小
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (ws_pkt.len) {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed: %s", esp_err_to_name(ret));
            free(buf);
            return ret;
        }
        
        ESP_LOGI(TAG, "Received WS data: %s", (char*)buf);
        
        // 解析 JSON 消息
        cJSON *root = cJSON_Parse((char*)buf);
        if (root != NULL) {
            cJSON *type = cJSON_GetObjectItem(root, "type");
            cJSON *value = cJSON_GetObjectItem(root, "value");
            
            if (type != NULL && value != NULL) {
                if (strcmp(type->valuestring, "text") == 0) {
                    // 发送文本数据到 UART
                    uart_write_bytes(UART_NUM, value->valuestring, strlen(value->valuestring));
                    ESP_LOGI(TAG, "Sent to UART: %s", value->valuestring);
                    
                } else if (strcmp(type->valuestring, "hex") == 0) {
                    // 处理 HEX 数据 
                    char *hex_str = value->valuestring;
                    size_t len = strlen(hex_str);
                    uint8_t *data = malloc(len / 2);
                    size_t data_len = 0;
                    
                    for (size_t i = 0; i < len; i += 2) {
                        if (i + 1 < len) {
                            char byte_str[3] = {hex_str[i], hex_str[i+1], '\0'};
                            data[data_len++] = (uint8_t)strtol(byte_str, NULL, 16);
                        }
                    }
                    
                    uart_write_bytes(UART_NUM, (char*)data, data_len);
                    free(data);
                    ESP_LOGI(TAG, "Sent HEX to UART: %s", hex_str);
                    
                } else if (strcmp(type->valuestring, "baudrate") == 0) {
                    // 更改波特率
                    int new_baudrate = atoi(value->valuestring);
                    ESP_LOGI(TAG, "Changing baudrate to: %d", new_baudrate);
                    if (new_baudrate > 0) {
                        // 停止 UART 读取任务
                        vTaskDelete(uart_task_handle);
                        init_uart(new_baudrate);
                        send_ws_data(req->handle, "baudrate_changed", value->valuestring);
                    }
                }
            }
            cJSON_Delete(root);
        }
        
        free(buf);
    }
    
    return ESP_OK;
}

//注册WebSocket处理程序
static const httpd_uri_t ws = {
        .uri       = "/ws",
        .method    = HTTP_GET,
        .handler   = websocket_handler,
        .user_ctx  = NULL,
        .is_websocket = true
    };

/// @brief 初始化WiFi与AP模式
/// @param  
/// @return 
esp_err_t app_wifi_web_init(void)
{
    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    // 创建默认事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // 创建默认的AP网络接口
    esp_netif_create_default_wifi_ap();
    // 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));   
    // 注册WiFi事件处理函数
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    //设置AP模式                                          
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    // 配置AP模式参数
    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .password = WIFI_AP_PASS,
            .channel = WIFI_AP_CHANNEL,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };    
    // 设置AP配置
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    // 启动WiFi
    ESP_ERROR_CHECK(esp_wifi_start());  
    vTaskDelay(pdMS_TO_TICKS(500)); 
    ESP_LOGI(TAG, "WiFi AP模式初始化完成");
    ESP_LOGI(TAG, "SSID:%s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "PASS:%s", WIFI_AP_PASS);
    return ESP_OK;
}


esp_err_t app_start_web_server(void)
{
    // 创建HTTP服务器
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // 修改HTTP服务器配置
    config.uri_match_fn = httpd_uri_match_wildcard; // 使用通配符匹配URI
    config.max_uri_handlers = 8;                    // 最大URI处理数
    config.max_resp_headers = 8;                    // 最大响应头数
    config.recv_wait_timeout = 30;                  // 接收等待超时时间
    config.send_wait_timeout = 30;                  // 发送等待超时时间
    config.lru_purge_enable = true;                 // 启用LRU清除

    // 启动HTTP服务器
    if (httpd_start(&server, &config) != ESP_OK)
    {
        return ESP_FAIL;
    }
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &ws);
    ESP_LOGI(TAG, "HTTP服务器启动完成");

    return ESP_OK;
}


/// @brief 处理wifi事件
/// @param arg 
/// @param event_base 
/// @param event_id 
/// @param event_data 
static void wifi_event_handler(void *arg, esp_event_base_t event_base,int32_t event_id, void *event_data)
{
    // 处理WiFi相关事件
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {

        }
    }

}