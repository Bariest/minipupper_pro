#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "SCSCL.h"

// 反馈数据缓存区（用于批量读取）
static uint8_t Mem[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L+1];

/**
 * @brief SCSCL系列伺服电机位置控制
 * 
 * 控制SCSCL系列伺服电机移动到指定位置
 * @param ID 伺服电机ID
 * @param Position 目标位置（0-4095，对应0°-360°）
 * @param Time 移动时间（毫秒），0表示使用默认速度
 * @param Speed 移动速度（0-2047）
 * @return int 操作结果（1成功，0失败）
 */
int WritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	uint8_t bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);  // 位置数据
	Host2SCS(bBuf+2, bBuf+3, Time);      // 时间数据
	Host2SCS(bBuf+4, bBuf+5, Speed);     // 速度数据
	
	return genWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

/**
 * @brief SCSCL系列伺服电机寄存器位置控制（延迟执行）
 * 
 * 写入位置控制参数但不立即执行，需要调用RegWriteAction()触发
 * @param ID 伺服电机ID
 * @param Position 目标位置
 * @param Time 移动时间
 * @param Speed 移动速度
 * @return int 操作结果（1成功，0失败）
 */
int RegWritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	uint8_t bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);
	Host2SCS(bBuf+2, bBuf+3, Time);
	Host2SCS(bBuf+4, bBuf+5, Speed);
	
	return regWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

/**
 * @brief 触发所有寄存器写入动作
 * 
 * 执行所有之前通过RegWritePos设置的动作（广播模式）
 */
void RegWriteAction()
{
	regAction(0xfe);  // 广播ID 0xFE
}

/**
 * @brief SCSCL系列伺服电机同步位置控制
 * 
 * 同时控制多个SCSCL伺服电机移动到各自的目标位置
 * @param ID 伺服电机ID数组
 * @param IDN 伺服电机数量
 * @param Position 目标位置数组
 * @param Time 移动时间数组（可为NULL）
 * @param Speed 移动速度数组（可为NULL）
 */
void SyncWritePos(uint8_t ID[], uint8_t IDN, uint16_t Position[], uint16_t Time[], uint16_t Speed[])
{
  uint8_t offbuf[32*6];  // 最大支持32个伺服电机
	uint8_t i;
  for(i = 0; i<IDN; i++){
		uint16_t T, V;
		if(Time){
			T = Time[i];
		}else{
			T = 0;  // 使用默认时间
		}
		if(Speed){
			V = Speed[i];
		}else{
			V = 0;  // 使用默认速度
		}
    Host2SCS(offbuf+i*6+0, offbuf+i*6+1, Position[i]);
    Host2SCS(offbuf+i*6+2, offbuf+i*6+3, T);
    Host2SCS(offbuf+i*6+4, offbuf+i*6+5, V);
  }
  syncWrite(ID, IDN, SCSCL_GOAL_POSITION_L, offbuf, 6);
}

/**
 * @brief 设置SCSCL伺服电机为PWM模式
 * 
 * 将伺服电机切换到PWM输出模式（用于直接控制电机输出）
 * @param ID 伺服电机ID
 * @return int 操作结果（1成功，0失败）
 */
int PWMMode(uint8_t ID)
{
	uint8_t bBuf[4];
	bBuf[0] = 0;
	bBuf[1] = 0;
	bBuf[2] = 0;
	bBuf[3] = 0;
	// 清除角度限制，进入PWM模式
	return genWrite(ID, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4);	
}

/**
 * @brief SCSCL系列伺服电机PWM输出控制
 * 
 * 在PWM模式下直接控制电机输出
 * @param ID 伺服电机ID
 * @param pwmOut PWM输出值（-1023到1023，负值表示反向）
 * @return int 操作结果（1成功，0失败）
 */
int WritePWM(uint8_t ID, int16_t pwmOut)
{
	uint8_t bBuf[2];
	if(pwmOut<0){
		pwmOut = -pwmOut;
		pwmOut |= (1<<10);  // 设置符号位
	}
	Host2SCS(bBuf+0, bBuf+1, pwmOut);

	return genWrite(ID, SCSCL_GOAL_TIME_L, bBuf, 2);
}

/**
 * @brief 启用/禁用SCSCL伺服电机扭矩
 * 
 * 控制伺服电机的扭矩输出开关
 * @param ID 伺服电机ID
 * @param Enable 1-启用扭矩，0-禁用扭矩
 * @return int 操作结果（1成功，0失败）
 */
int EnableTorque(uint8_t ID, uint8_t Enable)
{
	return writeByte(ID, SCSCL_TORQUE_ENABLE, Enable);
}

/**
 * @brief 解锁SCSCL伺服电机EEPROM
 * 
 * 解锁EEPROM以便进行参数写入
 * @param ID 伺服电机ID
 * @return int 操作结果（1成功，0失败）
 */
int unLockEprom(uint8_t ID)
{
	return writeByte(ID, SCSCL_LOCK, 0);
}

/**
 * @brief 锁定SCSCL伺服电机EEPROM
 * 
 * 锁定EEPROM防止意外修改
 * @param ID 伺服电机ID
 * @return int 操作结果（1成功，0失败）
 */
int LockEprom(uint8_t ID)
{
	return writeByte(ID, SCSCL_LOCK, 1);
}

/**
 * @brief 批量读取SCSCL伺服电机反馈数据
 * 
 * 一次性读取伺服电机的所有状态参数到缓存区
 * @param ID 伺服电机ID
 * @return int 读取的数据长度，-1表示失败
 */
int FeedBack(int ID)
{
	int nLen = Read(ID, SCSCL_PRESENT_POSITION_L, Mem, sizeof(Mem));
	if(nLen!=sizeof(Mem)){
		return -1;
	}
	return nLen;
}
	
/**
 * @brief 读取SCSCL伺服电机当前位置
 * 
 * @param ID 伺服电机ID，-1表示从缓存区读取（需先调用FeedBack）
 * @return int 当前位置值（带符号，-4095到4095）
 */
int ReadPos(int ID)
{
	int Pos = -1;
	if(ID==-1){
		// 从缓存区读取
		Pos = Mem[SCSCL_PRESENT_POSITION_L-SCSCL_PRESENT_POSITION_L];
		Pos <<= 8;
		Pos |= Mem[SCSCL_PRESENT_POSITION_H-SCSCL_PRESENT_POSITION_L];
	}else{
		// 直接从伺服电机读取
		Pos = readWord(ID, SCSCL_PRESENT_POSITION_L);
	}
	if(Pos&(1<<15)){
		Pos = -(Pos&~(1<<15));  // 处理负数
	}	
	return Pos;
}

/**
 * @brief 读取SCSCL伺服电机当前速度
 * 
 * @param ID 伺服电机ID，-1表示从缓存区读取
 * @return int 当前速度值（带符号）
 */
int ReadSpeed(int ID)
{
	int Speed = -1;
	if(ID==-1){
		Speed = Mem[SCSCL_PRESENT_SPEED_L-SCSCL_PRESENT_POSITION_L];
		Speed <<= 8;
		Speed |= Mem[SCSCL_PRESENT_SPEED_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Speed = readWord(ID, SCSCL_PRESENT_SPEED_L);
	}
	if(Speed&(1<<15)){
		Speed = -(Speed&~(1<<15));
	}	
	return Speed;
}

/**
 * @brief 读取SCSCL伺服电机当前负载
 * 
 * @param ID 伺服电机ID，-1表示从缓存区读取
 * @return int 当前负载值（带符号，符号位在第10位）
 */
int ReadLoad(int ID)
{
	int Load = -1;
	if(ID==-1){
		Load = Mem[SCSCL_PRESENT_LOAD_L-SCSCL_PRESENT_POSITION_L];
		Load <<= 8;
		Load |= Mem[SCSCL_PRESENT_LOAD_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Load = readWord(ID, SCSCL_PRESENT_LOAD_L);
	}
	if(Load&(1<<10)){
		Load = -(Load&~(1<<10));
	}
	return Load;
}

/**
 * @brief 读取SCSCL伺服电机电压
 * 
 * @param ID 伺服电机ID，-1表示从缓存区读取
 * @return int 电压值（单位：0.1V）
 */
int ReadVoltage(int ID)
{
	int Voltage = -1;
	if(ID==-1){
		Voltage = Mem[SCSCL_PRESENT_VOLTAGE-SCSCL_PRESENT_POSITION_L];	
	}else{
		Voltage = readByte(ID, SCSCL_PRESENT_VOLTAGE);
	}
	return Voltage;
}

/**
 * @brief 读取SCSCL伺服电机温度
 * 
 * @param ID 伺服电机ID，-1表示从缓存区读取
 * @return int 温度值（摄氏度）
 */
int ReadTemper(int ID)
{
	int Temper = -1;
	if(ID==-1){
		Temper = Mem[SCSCL_PRESENT_TEMPERATURE-SCSCL_PRESENT_POSITION_L];	
	}else{
		Temper = readByte(ID, SCSCL_PRESENT_TEMPERATURE);
	}
	return Temper;
}

/**
 * @brief 读取SCSCL伺服电机运动状态
 * 
 * @param ID 伺服电机ID，-1表示从缓存区读取
 * @return int 运动状态（0-停止，1-运动中）
 */
int ReadMove(int ID)
{
	int Move = -1;
	if(ID==-1){
		Move = Mem[SCSCL_MOVING-SCSCL_PRESENT_POSITION_L];	
	}else{
		Move = readByte(ID, SCSCL_MOVING);
	}
	return Move;
}

/**
 * @brief 读取SCSCL伺服电机电流
 * 
 * @param ID 伺服电机ID，-1表示从缓存区读取
 * @return int 电流值（带符号，单位：mA）
 */
int ReadCurrent(int ID)
{
	int Current = -1;
	if(ID==-1){
		Current = Mem[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L];
		Current <<= 8;
		Current |= Mem[SCSCL_PRESENT_CURRENT_L-SCSCL_PRESENT_POSITION_L];
	}else{
		Current = readWord(ID, SCSCL_PRESENT_CURRENT_L);
	}
	if(Current&(1<<15)){
		Current = -(Current&~(1<<15));
	}	
	return Current;
}