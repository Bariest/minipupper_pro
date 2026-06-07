#ifndef _FTSERVO_HAL_H
#define _FTSERVO_HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FEETECH总线舵机类型枚举
 */
typedef enum {
    SERVO_SCSCL = 0,    /**< SCS系列标准舵机 (SCSCL)，如 SCS15/SCS20/SCS30 */
    SERVO_SMS_STS,      /**< SMS/STS系列舵机，如 SMS03/SMS05/STS30 */
    SERVO_HLS,          /**< HLS系列舵机，如 HLS15/HLS20 */
} servo_type_t;

/**
 * @brief 一键初始化FEETECH总线舵机
 * 
 * 选择舵机类型并配置UART引脚，内部自动完成UART初始化和协议参数配置。
 * 调用此函数后，可直接使用 WritePos/ReadPos 等API。
 * 
 * @param type 舵机类型，见 servo_type_t 枚举
 * @param tx_pin UART发送引脚号 (GPIO)
 * @param rx_pin UART接收引脚号 (GPIO)
 * @param rts_pin RS485方向控制引脚号，-1表示不使用RS485模式（全双工）
 * @param baud_rate 波特率，传0则使用默认值(1000000)
 * @return int 0成功，非0失败
 */
int ftServo_InitWithType(servo_type_t type, int tx_pin, int rx_pin, int rts_pin, int baud_rate);

/**
 * @brief 初始化FEETECH总线伺服的UART接口（底层API）
 * 
 * @param uart_num UART编号（如UART_NUM_1）
 * @param tx_pin 发送引脚号
 * @param rx_pin 接收引脚号
 * @param rts_pin RS485方向控制引脚（负值表示不使用RS485模式）
 * @param baud_rate 波特率（通常为1000000）
 * @return int 0表示成功，非0表示失败
 */
int ftServo_Init(uint8_t uart_num, int tx_pin, int rx_pin, int rts_pin, int baud_rate);

/**
 * @brief 反初始化UART接口
 * 
 * @param uart_num 要反初始化的UART编号
 */
void ftServo_Deinit(uint8_t uart_num);

/**
 * @brief 通过UART发送数据
 * 
 * @param nDat 要发送的数据缓冲区指针
 * @param nLen 要发送的数据长度
 */
void ftUart_Send(uint8_t *nDat, int nLen);

/**
 * @brief 从UART读取数据
 * 
 * @param nDat 接收数据的缓冲区指针
 * @param nLen 要读取的最大数据长度
 * @return int 实际读取的数据长度，0表示超时或无数据
 */
int ftUart_Read(uint8_t *nDat, int nLen);

/**
 * @brief 清空UART接收缓冲区
 * 
 * 丢弃接收缓冲区中的所有数据，用于半双工通信中清除发送回显
 */
void ftUart_FlushRx(void);

/**
 * @brief 总线通信延时函数
 * 
 * 在总线通信中提供必要的延时，确保通信稳定性
 */
void ftBus_Delay(void);

#ifdef __cplusplus
}
#endif

#endif