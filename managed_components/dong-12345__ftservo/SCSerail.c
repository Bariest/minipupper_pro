#include <stdint.h>
#include "ftservo.h"

// 发送缓冲区（用于批量发送数据）
uint8_t wBuf[128];
// 发送缓冲区当前长度
uint8_t wLen = 0;

/**
 * @brief 从UART读取数据（底层接口）
 * 
 * 调用硬件抽象层的UART读取函数
 * @param nDat 接收数据的缓冲区指针
 * @param nLen 要读取的最大数据长度
 * @return int 实际读取的数据长度
 */
int readSCS(unsigned char *nDat, int nLen)
{
	return ftUart_Read(nDat, nLen);
}

/**
 * @brief 向UART写入数据到缓冲区（底层接口）
 * 
 * 将数据写入发送缓冲区，等待批量发送
 * @param nDat 要发送的数据缓冲区指针
 * @param nLen 要发送的数据长度
 * @return int 当前缓冲区中的数据长度
 */
int writeSCS(unsigned char *nDat, int nLen)
{
	while(nLen--){
		if(wLen<sizeof(wBuf)){
			wBuf[wLen] = *nDat;
			wLen++;
			nDat++;
		}
	}
	return wLen;
}

/**
 * @brief 向UART写入单字节数据到缓冲区（底层接口）
 * 
 * 将单字节数据写入发送缓冲区
 * @param bDat 要发送的字节数据
 * @return int 当前缓冲区中的数据长度
 */
int writeByteSCS(unsigned char bDat)
{
	if(wLen<sizeof(wBuf)){
		wBuf[wLen] = bDat;
		wLen++;
	}
	return wLen;
}

/**
 * @brief 清空接收缓冲区（底层接口）
 * 
 * 在读取操作前提供必要的延时，确保通信稳定性
 */
void rFlushSCS()
{
	ftUart_FlushRx();
	ftBus_Delay();
}

/**
 * @brief 刷新发送缓冲区（底层接口）
 * 
 * 将发送缓冲区中的所有数据一次性发送出去
 */
void wFlushSCS()
{
	if(wLen){
		ftUart_Send(wBuf, wLen);
		wLen = 0;
		ftUart_FlushRx();
	}
}