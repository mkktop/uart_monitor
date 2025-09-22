#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "app/app_http_serve.h"



void app_main(void)
{
    //初始化nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    //初始化uart
    init_uart(115200);
    //初始化wifi
    app_wifi_web_init();
    //初始化web服务器
    app_start_web_server();
    while (1)
    {  
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}
