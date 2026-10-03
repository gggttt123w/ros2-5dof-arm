#pragma once
#include "main.h"

extern uint8_t APS6404_LastStatus;
extern uint8_t APS6404_LastRx[6];   /* 0x9F 期间 MISO 上的原始 6 字节 */

uint16_t APS6404_ReadID(void);
void APS6404_Init(void);
void APS6404_WireCheck(void);   /* 接线自检，死循环，仅排查时用 */