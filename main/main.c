#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "app/app_http_serve.h"
/**
 * @brief UART0 的 RX 和 TX 引脚定义
 *
 * 定义了 UART0 的 RX 和 TX 引脚，分别连接到 GPIO 1 和 GPIO 2。
 */


extern httpd_handle_t server;
//接收缓冲区
uint8_t rx_buffer[1024];

/**
 * @brief UART0 事件队列句柄
 *
 * 用于存储 UART0 事件的队列句柄，用于接收和处理 UART 事件。
 */
QueueHandle_t uart0_queue;

/**
 * @brief 主应用程序入口函数，负责初始化 UART 并处理 UART 事件。
 * 
 * 此函数会配置 UART 参数、设置 UART 引脚、安装 UART 驱动，
 * 然后进入一个无限循环，持续从队列中接收 UART 事件并进行相应处理。
 */
void app_main(void)
{
    //初始化nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    init_uart(115200);
    app_wifi_web_init();
    app_start_web_server();


    // // 定义一个 UART 事件结构体，用于存储从队列接收到的 UART 事件信息
    // uart_event_t uart0_event;
    // // 定义 UART 配置结构体，并初始化 UART 参数
    // uart_config_t uart_config = {
    //     .baud_rate = 115200,          // 设置波特率为 115200
    //     .data_bits = UART_DATA_8_BITS, // 设置数据位为 8 位
    //     .parity = UART_PARITY_DISABLE, // 禁用奇偶校验
    //     .stop_bits = UART_STOP_BITS_1, // 设置停止位为 1 位
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 禁用硬件流控制
    //     .source_clk = UART_SCLK_DEFAULT,       // 使用默认时钟源
    // };

    // // 根据配置参数配置 UART0
    // uart_param_config(UART_NUM_0, &uart_config);
    // // 设置 UART0 的 TX、RX 引脚，不改变 RTS 和 CTS 引脚
    // uart_set_pin(UART_NUM_0, UART0_TX, UART0_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // // 安装 UART0 驱动，设置接收和发送缓冲区大小为 1024 字节，
    // // 创建一个深度为 20 的事件队列，并将队列句柄存储在 uart0_queue 中
    // uart_driver_install(UART_NUM_0, 1024, 1024, 20, &uart0_queue, 0);

    // // 进入无限循环，持续处理 UART 事件
    while (1)
    {  
        vTaskDelay(pdMS_TO_TICKS(1000)); 
        // 从 uart0_queue 队列中接收 UART 事件，阻塞等待直到有事件到来
        // if (pdTRUE == xQueueReceive(uart0_queue, (void*)&uart0_event, portMAX_DELAY))
        // {
        //     // 根据接收到的 UART 事件类型进行不同处理
        //     switch (uart0_event.type)
        //     {
        //     // 当接收到 UART 数据事件时
        //     case UART_DATA:
        //         // 从 UART0 读取数据到 rx_buffer 中，读取长度为事件中记录的大小，超时时间为 1000 ticks
        //         uart_read_bytes(UART_NUM_0, (uint8_t*)&rx_buffer, uart0_event.size, 1000);
        //         // 将读取到的数据原样写回到 UART0 发送出去
        //         //uart_write_bytes(UART_NUM_0, (const char*)rx_buffer, uart0_event.size);
        //         send_ws_data(server, "text", (const char*)rx_buffer);
        //         break;
        //     // 当 UART 接收缓冲区满时
        //     case UART_BUFFER_FULL:
        //         // 清空 UART0 的输入缓冲区
        //         uart_flush_input(UART_NUM_0);
        //         // 重置 UART 事件队列
        //         xQueueReset(uart0_queue);
        //         break;
        //     // 当 UART FIFO 溢出时
        //     case UART_FIFO_OVF:
        //         // 清空 UART0 的输入缓冲区
        //         uart_flush_input(UART_NUM_0);
        //         // 重置 UART 事件队列
        //         xQueueReset(uart0_queue);
        //         break;
        //     default:
        //         break;
        //     }
        // }
    }
}
