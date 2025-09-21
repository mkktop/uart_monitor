#ifndef __APP_HTTP_SERVE_H__
#define __APP_HTTP_SERVE_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/uart.h"
#include "cJSON.h"
// WiFi配置参数
#define WIFI_AP_SSID "uart_monitor" // 配网热点名称
#define WIFI_AP_PASS "88888888"     // 配网热点密码
#define WIFI_AP_CHANNEL 1           // WiFi信道
#define WIFI_CONNECT_TIMEOUT 30000  // 连接超时时间(ms)
#define WIFI_AP_MAX_CONN 3          // 最大连接数
#define UART_NUM UART_NUM_1         // 使用的UART通道
#define BUF_SIZE 1024               // 缓冲区大小
#define RD_BUF_SIZE 1024            // 读取缓冲区大小
#define UART0_RX 0                  // UART0 RX引脚
#define UART0_TX 1                  // UART0 TX引脚

esp_err_t app_wifi_web_init(void);
esp_err_t app_start_web_server(void);
void send_ws_data(httpd_handle_t server, const char *type, const char *value);
void init_uart(int baudrate);
#endif /* __APP_HTTP_SERVE_H__ */