#ifndef _SCSCL_H
#define _SCSCL_H

#include <stdint.h>

#define SCSCL_MODEL_L 3
#define SCSCL_MODEL_H 4

#define SCSCL_ID 5
#define SCSCL_BAUD_RATE 6
#define SCSCL_MIN_ANGLE_LIMIT_L 9
#define SCSCL_MIN_ANGLE_LIMIT_H 10
#define SCSCL_MAX_ANGLE_LIMIT_L 11
#define SCSCL_MAX_ANGLE_LIMIT_H 12
#define SCSCL_CW_DEAD 26
#define SCSCL_CCW_DEAD 27

#define SCSCL_TORQUE_ENABLE 40
#define SCSCL_GOAL_POSITION_L 42
#define SCSCL_GOAL_POSITION_H 43
#define SCSCL_GOAL_TIME_L 44
#define SCSCL_GOAL_TIME_H 45
#define SCSCL_GOAL_SPEED_L 46
#define SCSCL_GOAL_SPEED_H 47
#define SCSCL_LOCK 48

#define SCSCL_PRESENT_POSITION_L 56
#define SCSCL_PRESENT_POSITION_H 57
#define SCSCL_PRESENT_SPEED_L 58
#define SCSCL_PRESENT_SPEED_H 59
#define SCSCL_PRESENT_LOAD_L 60
#define SCSCL_PRESENT_LOAD_H 61
#define SCSCL_PRESENT_VOLTAGE 62
#define SCSCL_PRESENT_TEMPERATURE 63
#define SCSCL_MOVING 66
#define SCSCL_PRESENT_CURRENT_L 69
#define SCSCL_PRESENT_CURRENT_H 70

extern int WritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed);
extern int RegWritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed);
extern void RegWriteAction(void);
extern void SyncWritePos(uint8_t ID[], uint8_t IDN, uint16_t Position[], uint16_t Time[], uint16_t Speed[]);
extern int PWMMode(uint8_t ID);
extern int WritePWM(uint8_t ID, int16_t pwmOut);
extern int EnableTorque(uint8_t ID, uint8_t Enable);
extern int unLockEprom(uint8_t ID);
extern int LockEprom(uint8_t ID);
extern int FeedBack(int ID);
extern int ReadPos(int ID);
extern int ReadSpeed(int ID);
extern int ReadLoad(int ID);
extern int ReadVoltage(int ID);
extern int ReadTemper(int ID);
extern int ReadMove(int ID);
extern int ReadCurrent(int ID);

#endif
