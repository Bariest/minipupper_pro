#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "SMS_STS.h"

/**
 * @brief SMS_STS系列伺服电机位置控制（带加速度）
 * 
 * 控制SMS_STS系列伺服电机移动到指定位置，支持加速度控制
 * @param ID 伺服电机ID
 * @param Position 目标位置（0-4095，对应0°-360°，负值表示反向）
 * @param Speed 移动速度（0-2400，单位：0.114 rpm）
 * @param ACC 加速度等级（0-254）
 * @return int 操作结果（1成功，0失败）
 */
int WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[7];
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);  // 设置符号位表示负方向
	}

	bBuf[0] = ACC;                           // 加速度
	Host2SCS(bBuf+1, bBuf+2, Position);      // 位置
	Host2SCS(bBuf+3, bBuf+4, 0);             // 保留字段
	Host2SCS(bBuf+5, bBuf+6, Speed);         // 速度
	
	return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

/**
 * @brief SMS_STS系列伺服电机寄存器位置控制（延迟执行，带加速度）
 * 
 * 写入位置控制参数但不立即执行，需要调用regAction()触发
 * @param ID 伺服电机ID
 * @param Position 目标位置
 * @param Speed 移动速度
 * @param ACC 加速度等级
 * @return int 操作结果（1成功，0失败）
 */
int RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[7];
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, Position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return regWrite(ID, SMS_STS_ACC, bBuf, 7);
}

/**
 * @brief SMS_STS系列伺服电机同步位置控制（带加速度）
 * 
 * 同时控制多个SMS_STS伺服电机移动到各自的目标位置
 * @param ID 伺服电机ID数组
 * @param IDN 伺服电机数量
 * @param Position 目标位置数组
 * @param Speed 移动速度数组（可为NULL）
 * @param ACC 加速度等级数组（可为NULL）
 */
void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[])
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
    Host2SCS(offbuf+i*7+3, offbuf+i*7+4, 0);
    Host2SCS(offbuf+i*7+5, offbuf+i*7+6, V);
	}
  syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
}

/**
 * @brief 设置SMS_STS伺服电机为轮子模式
 * 
 * 将伺服电机切换到连续旋转模式（轮子模式）
 * @param ID 伺服电机ID
 * @return int 操作结果（1成功，0失败）
 */
int WheelMode(uint8_t ID)
{
	return writeByte(ID, SMS_STS_MODE, 1);		
}

/**
 * @brief SMS_STS系列伺服电机轮子模式速度控制
 * 
 * 在轮子模式下控制伺服电机的旋转速度
 * @param ID 伺服电机ID
 * @param Speed 旋转速度（-2400到2400，负值表示反向）
 * @param ACC 加速度等级
 * @return int 操作结果（1成功，0失败）
 */
int WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[7];

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, 0);        // 位置设为0（轮子模式）
	Host2SCS(bBuf+3, bBuf+4, 0);        // 保留字段
	Host2SCS(bBuf+5, bBuf+6, Speed);    // 速度
	
	return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

/**
 * @brief SMS_STS伺服电机角度偏移校准
 * 
 * 执行角度偏移校准操作
 * @param ID 伺服电机ID
 * @return int 操作结果（1成功，0失败）
 */
int CalibrationOfs(uint8_t ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128);
}

/**
 * @brief 解锁SMS_STS伺服电机EEPROM
 * 
 * 解锁EEPROM以便进行参数写入
 * @param ID 伺服电机ID
 * @return int 操作结果（1成功，0失败）
 */
int unLockEpromEx(uint8_t ID)
{
	return writeByte(ID, SMS_STS_LOCK, 0);
}

/**
 * @brief 锁定SMS_STS伺服电机EEPROM
 * 
 * 锁定EEPROM防止意外修改
 * @param ID 伺服电机ID
 * @return int 操作结果（1成功，0失败）
 */
int LockEpromEx(uint8_t ID)
{
	return writeByte(ID, SMS_STS_LOCK, 1);
}