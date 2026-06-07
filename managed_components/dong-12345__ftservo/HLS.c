#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "HLS.h"

/**
 * @brief HLS系列伺服电机位置控制（带加速度和扭矩限制）
 * 
 * 控制HLS系列伺服电机移动到指定位置，支持加速度和扭矩限制
 * @param ID 伺服电机ID
 * @param Position 目标位置（0-4095，对应0°-360°，负值表示反向）
 * @param Speed 移动速度（0-2400，单位：0.114 rpm）
 * @param ACC 加速度等级（0-254）
 * @param Torque 扭矩限制值（0-2047）
 * @return int 操作结果（1成功，0失败）
 */
int WritePosEx2(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC, uint16_t Torque)
{
	uint8_t bBuf[7];
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);  // 设置符号位表示负方向
	}

	bBuf[0] = ACC;                           // 加速度
	Host2SCS(bBuf+1, bBuf+2, Position);      // 位置
	Host2SCS(bBuf+3, bBuf+4, Torque);        // 扭矩限制
	Host2SCS(bBuf+5, bBuf+6, Speed);         // 速度
	
	return genWrite(ID, HLS_ACC, bBuf, 7);
}

/**
 * @brief HLS系列伺服电机寄存器位置控制（延迟执行，带加速度和扭矩限制）
 * 
 * 写入位置控制参数但不立即执行，需要调用regAction()触发
 * @param ID 伺服电机ID
 * @param Position 目标位置
 * @param Speed 移动速度
 * @param ACC 加速度等级
 * @param Torque 扭矩限制值
 * @return int 操作结果（1成功，0失败）
 */
int RegWritePosEx2(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC, uint16_t Torque)
{
	uint8_t bBuf[7];
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, Position);
	Host2SCS(bBuf+3, bBuf+4, Torque);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return regWrite(ID, HLS_ACC, bBuf, 7);
}

/**
 * @brief HLS系列伺服电机同步位置控制（带加速度和扭矩限制）
 * 
 * 同时控制多个HLS伺服电机移动到各自的目标位置
 * @param ID 伺服电机ID数组
 * @param IDN 伺服电机数量
 * @param Position 目标位置数组
 * @param Speed 移动速度数组（可为NULL）
 * @param ACC 加速度等级数组（可为NULL）
 * @param Torque 扭矩限制值数组
 */
void SyncWritePosEx2(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[], uint16_t Torque[])
{
	uint8_t offbuf[32*7];  // 最大支持32个伺服电机
	uint8_t i;
	uint16_t V;
  for(i = 0; i<IDN; i++){
		if(Position[i]<0){
			Position[i] = -Position[i];
			Position[i] |= (1<<15);
		}

		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
		if(ACC){
			offbuf[i*7] = ACC[i];
		}else{
			offbuf[i*7] = 0;
		}
		Host2SCS(offbuf+i*7+1, offbuf+i*7+2, Position[i]);
    Host2SCS(offbuf+i*7+3, offbuf+i*7+4, Torque[i]);
    Host2SCS(offbuf+i*7+5, offbuf+i*7+6, V);
	}
  syncWrite(ID, IDN, HLS_ACC, offbuf, 7);
}

/**
 * @brief HLS系列伺服电机轮子模式速度控制（带扭矩限制）
 * 
 * 在轮子模式下控制HLS伺服电机的旋转速度，并设置扭矩限制
 * @param ID 伺服电机ID
 * @param Speed 旋转速度（-2400到2400，负值表示反向）
 * @param ACC 加速度等级
 * @param Torque 扭矩限制值
 * @return int 操作结果（1成功，0失败）
 */
int WriteSpeEx(uint8_t ID, int16_t Speed, uint8_t ACC, uint16_t Torque)
{
	uint8_t bBuf[7];

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, 0);        // 位置设为0（轮子模式）
	Host2SCS(bBuf+3, bBuf+4, Torque);   // 扭矩限制
	Host2SCS(bBuf+5, bBuf+6, Speed);    // 速度
	
	return genWrite(ID, HLS_ACC, bBuf, 7);
}