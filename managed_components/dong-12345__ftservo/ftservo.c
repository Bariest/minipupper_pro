#include "ftservo.h"
#include "SCS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 存储当前使用的UART编号
static uint8_t s_uart_num = 0;
// 日志标签
static const char *TAG = "ftservo";

/**
 * @brief 初始化FEETECH总线伺服的UART接口
 * 
 * @param uart_num UART编号（如UART_NUM_1）
 * @param tx_pin 发送引脚号
 * @param rx_pin 接收引脚号  
 * @param rts_pin RS485方向控制引脚（负值表示不使用RS485模式）
 * @param baud_rate 波特率（通常为1000000）
 * @return int 0表示成功，非0表示失败
 */
int ftServo_Init(uint8_t uart_num, int tx_pin, int rx_pin, int rts_pin, int baud_rate)
{
    s_uart_num = uart_num;

    // 配置UART参数
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,                 // 波特率
        .data_bits = UART_DATA_8_BITS,          // 数据位：8位
        .parity = UART_PARITY_DISABLE,          // 无校验位
        .stop_bits = UART_STOP_BITS_1,          // 停止位：1位
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // 禁用硬件流控
        .source_clk = UART_SCLK_DEFAULT,        // 使用默认时钟源
    };

    // 安装UART驱动
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 256, 256, 0, NULL, 0));
    // 配置UART参数
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // 设置UART引脚
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, rts_pin, UART_PIN_NO_CHANGE));

    // 根据RTS引脚配置工作模式
    if (rts_pin >= 0) {
        // 启用RS485半双工模式（用于总线通信）
        ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));
        ESP_LOGI(TAG, "UART%d RS485 half-duplex mode, RTS=%d", uart_num, rts_pin);
    } else {
        ESP_LOGI(TAG, "UART%d full-duplex mode", uart_num);
    }

    ESP_LOGI(TAG, "UART%d initialized: baud=%d, tx=%d, rx=%d", uart_num, baud_rate, tx_pin, rx_pin);
    return 0;
}

/**
 * @brief 反初始化UART接口
 * 
 * @param uart_num 要反初始化的UART编号
 */
void ftServo_Deinit(uint8_t uart_num)
{
    uart_driver_delete(uart_num);
}

/**
 * @brief 通过UART发送数据
 * 
 * @param nDat 要发送的数据缓冲区指针
 * @param nLen 要发送的数据长度
 */
void ftUart_Send(uint8_t *nDat, int nLen)
{
    // 发送数据
    uart_write_bytes(s_uart_num, (const char *)nDat, nLen);
    // 等待发送完成
    uart_wait_tx_done(s_uart_num, pdMS_TO_TICKS(10));
}

/**
 * @brief 从UART读取数据
 * 
 * @param nDat 接收数据的缓冲区指针
 * @param nLen 要读取的最大数据长度
 * @return int 实际读取的数据长度，0表示超时或无数据
 */
int ftUart_Read(uint8_t *nDat, int nLen)
{
    // 尝试读取数据，超时时间为50ms
    int len = uart_read_bytes(s_uart_num, nDat, nLen, pdMS_TO_TICKS(50));
    if (len <= 0) {
        return 0;
    }
    return len;
}

void ftUart_FlushRx(void)
{
    uart_flush_input(s_uart_num);
}

/**
 * @brief 总线通信延时函数
 * 
 * 在总线通信中提供必要的延时，确保通信稳定性
 */
void ftBus_Delay(void)
{
    vTaskDelay(pdMS_TO_TICKS(1));
}

int ftServo_InitWithType(servo_type_t type, int tx_pin, int rx_pin, int rts_pin, int baud_rate)
{
    if (baud_rate <= 0) {
        baud_rate = 1000000;
    }

    int ret = ftServo_Init(UART_NUM_1, tx_pin, rx_pin, rts_pin, baud_rate);
    if (ret != 0) {
        return ret;
    }

    switch (type) {
        case SERVO_SCSCL:
        case SERVO_HLS:
            setEnd(1);
            break;
        case SERVO_SMS_STS:
        default:
            setEnd(0);
            break;
    }

    ESP_LOGI(TAG, "Servo type %d initialized, baud=%d", type, baud_rate);
    return 0;
}