/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/** 一帧 CAN 报文。标准帧/扩展帧统一放进 id，上层不用再区分。 */
typedef struct {
  uint32_t id;        /* 标准帧 = 11 位 StdId，扩展帧 = 29 位 ExtId */
  uint8_t  data[8];
  uint8_t  len;       /* 有效数据长度 0~8 */
} CAN_Frame_t;


/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
/** 配置过滤器(全通)、启动 CAN、打开 FIFO0 接收中断。在 MX_CAN_Init() 之后调用一次。
  * @retval HAL_OK / HAL_ERROR */
uint8_t CAN_Init(void);

/** 发送一帧标准数据帧。
  * @param id   11 位标准 ID
  * @param data 数据指针，len 为 0 时可传 NULL
  * @param len  数据长度 0~8，超过 8 按 8 处理
  * @retval HAL_OK / HAL_ERROR(三个发送邮箱都满) */
uint8_t CAN_Send(uint32_t id, const uint8_t *data, uint8_t len);

/** 取一帧收到的报文（非阻塞，只能在任务上下文调用，不要在中断里调用）。
  * 收到的帧由 HAL_CAN_RxFifo0MsgPendingCallback 在中断里存进 16 帧的环形缓冲区。
  * @param  frame 输出，取到的帧
  * @retval 1 = 取到一帧；0 = 当前没有待处理的帧 */
uint8_t CAN_GetRxFrame(CAN_Frame_t *frame);

/** 因接收缓冲区满而被丢掉的帧数。任务处理不过来时才会涨，正常应一直为 0。 */
uint32_t CAN_GetRxDropCount(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

