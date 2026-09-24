#pragma once

//舵机控制

#define N_MAX 2500
#define T_NORMAL 1000
#define SEVRO_NUMBER 5
#define DEBUG_MODE 1
void robot_arm_init(void);

uint16_t Servo_position_get(uint8_t id);

uint16_t Servo_mode_get(uint8_t id);

void Servo_position_set(uint8_t id,uint16_t n,uint16_t t);

void Servo_temp_and_v_get(uint8_t id,double _temp_v[2]);
//CAN通讯

#define GETANGLE 0x200
#define GETTANDV 0x201
#define GETERR   0x202

#define CAN_ID_CMD_DOWN 0x104

void CAN_Servo_status_send_pkg(ServoMsg_t *st,uint8_t pkg[8]);