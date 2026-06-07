#ifndef _SCS_H
#define _SCS_H

#include <stdint.h>

/**
 * @brief SCS伺服通信错误码枚举
 */
enum SCS_ERR_LIST
{
  SCS_ERR_NO_REPLY = 1,   // 无应答错误
  SCS_ERR_CRC_CMP  = 2,   // CRC校验错误  
  SCS_ERR_SLAVE_ID = 3,   // 从机ID错误
  SCS_ERR_BUFF_LEN = 4,   // 缓冲区长度错误
};

// === 基础写入函数 ===

/**
 * @brief 通用写入函数
 * 向指定ID的伺服电机内存地址写入数据
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param nDat 数据缓冲区指针
 * @param nLen 数据长度
 * @return int 操作结果
 */
extern int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);

/**
 * @brief 寄存器写入函数（延迟执行）
 * 写入数据但不立即执行，需要调用regAction()触发
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param nDat 数据缓冲区指针
 * @param nLen 数据长度
 * @return int 操作结果
 */
extern int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);

/**
 * @brief 触发寄存器写入动作
 * 执行之前通过regWrite设置的动作
 * @param ID 伺服电机ID
 * @return int 操作结果
 */
extern int regAction(uint8_t ID);

/**
 * @brief 同步写入函数
 * 同时向多个伺服电机写入相同的数据
 * @param ID 伺服电机ID数组
 * @param IDN 伺服电机数量
 * @param MemAddr 内存地址
 * @param nDat 数据缓冲区指针
 * @param nLen 数据长度
 */
extern void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);

// === 字节/字写入函数 ===

/**
 * @brief 写入单字节数据
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param bDat 要写入的字节数据
 * @return int 操作结果
 */
extern int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat);

/**
 * @brief 写入双字节数据（16位）
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param wDat 要写入的字数据
 * @return int 操作结果
 */
extern int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat);

// === 读取函数 ===

/**
 * @brief 通用读取函数
 * 从指定ID的伺服电机内存地址读取数据
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param nData 接收数据的缓冲区指针
 * @param nLen 要读取的数据长度
 * @return int 操作结果
 */
extern int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen);

/**
 * @brief 读取单字节数据
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @return int 读取到的字节数据
 */
extern int readByte(uint8_t ID, uint8_t MemAddr);

/**
 * @brief 读取双字节数据（16位）
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @return int 读取到的字数据
 */
extern int readWord(uint8_t ID, uint8_t MemAddr);

// === 系统控制函数 ===

/**
 * @brief Ping伺服电机
 * 检测指定ID的伺服电机是否在线
 * @param ID 伺服电机ID
 * @return int 返回伺服电机的实际ID
 */
extern int Ping(uint8_t ID);

/**
 * @brief 重置伺服电机
 * 将伺服电机恢复到出厂设置
 * @param ID 伺服电机ID
 * @return int 操作结果
 */
extern int Reset(uint8_t ID);

// === 同步读取函数 ===

/**
 * @brief 同步读取数据包发送
 * 发送同步读取请求
 * @param ID 伺服电机ID数组
 * @param IDN 伺服电机数量
 * @param MemAddr 内存地址
 * @param nLen 要读取的数据长度
 * @return int 操作结果
 */
extern int syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen);

/**
 * @brief 同步读取数据包接收
 * 接收单个伺服电机的同步读取响应
 * @param ID 伺服电机ID
 * @param nDat 接收数据的缓冲区指针
 * @return int 操作结果
 */
extern int syncReadPacketRx(uint8_t ID, uint8_t *nDat);

/**
 * @brief 同步读取字节数据
 * @return int 读取到的字节数据
 */
extern int syncReadRxPacketToByte(void);

/**
 * @brief 同步读取字数据
 * @param negBit 符号位位置
 * @return int 读取到的字数据
 */
extern int syncReadRxPacketToWrod(uint8_t negBit);

/**
 * @brief 开始同步读取
 * 初始化同步读取过程
 * @param IDN 伺服电机数量
 * @param rxLen 接收数据长度
 */
extern void syncReadBegin(uint8_t IDN, uint8_t rxLen);

/**
 * @brief 结束同步读取
 * 完成同步读取过程
 */
extern void syncReadEnd(void);

// === 内部辅助函数 ===

/**
 * @brief 写入数据缓冲区
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param nDat 数据缓冲区指针
 * @param nLen 数据长度
 * @param Fun 功能码
 */
extern void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun);

/**
 * @brief 主机数据转SCS格式
 * 将主机端的整数数据转换为SCS协议的高低字节格式
 * @param DataL 低字节指针
 * @param DataH 高字节指针
 * @param Data 要转换的数据
 */
extern void Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data);

/**
 * @brief SCS格式转主机数据
 * 将SCS协议的高低字节格式转换为主机端的整数数据
 * @param DataL 低字节
 * @param DataH 高字节
 * @return int 转换后的数据
 */
extern int SCS2Host(uint8_t DataL, uint8_t DataH);

/**
 * @brief 确认应答
 * 等待并验证伺服电机的应答
 * @param ID 伺服电机ID
 * @return int 应答结果
 */
extern int Ack(uint8_t ID);

/**
 * @brief 检查数据包头部
 * 验证接收到的数据包头部是否正确
 * @return int 检查结果
 */
extern int checkHead(void);

// === 全局设置函数 ===

/**
 * @brief 设置数据包结束字节
 * @param _End 结束字节值
 */
extern void setEnd(uint8_t _End);

/**
 * @brief 获取当前数据包结束字节
 * @return uint8_t 当前结束字节值
 */
extern uint8_t getEnd(void);

/**
 * @brief 设置通信电平
 * @param _Level 电平值
 */
extern void setLevel(uint8_t _Level);

/**
 * @brief 获取当前状态
 * @return int 当前状态码
 */
extern int getState(void);

/**
 * @brief 获取最后的错误码
 * @return int 最后的错误码
 */
extern int getLastError(void);

// === 串口通信底层函数 ===

/**
 * @brief 写入SCS数据
 * @param nDat 数据缓冲区指针
 * @param nLen 数据长度
 * @return int 写入结果
 */
extern int writeSCS(uint8_t *nDat, int nLen);

/**
 * @brief 写入单字节SCS数据
 * @param bDat 要写入的字节
 * @return int 写入结果
 */
extern int writeByteSCS(unsigned char bDat);

/**
 * @brief 读取SCS数据
 * @param nDat 接收数据的缓冲区指针
 * @param nLen 要读取的数据长度
 * @return int 读取结果
 */
extern int readSCS(uint8_t *nDat, int nLen);

/**
 * @brief 清空接收缓冲区
 */
extern void rFlushSCS(void);

/**
 * @brief 清空发送缓冲区
 */
extern void wFlushSCS(void);

#endif