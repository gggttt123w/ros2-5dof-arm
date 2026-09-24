/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* 接收环形缓冲区。head 只在 CAN 中断里写、tail 只在任务里写，
   两个都是 uint8_t —— Cortex-M4 上字节读写是原子的，且这是
   单生产者/单消费者结构，所以不需要关中断保护。 */
#define CAN_RX_QUEUE_LEN   16U

static volatile CAN_Frame_t can_rx_buf[CAN_RX_QUEUE_LEN];
static volatile uint8_t     can_rx_head = 0U;   /* 中断里写 */
static volatile uint8_t     can_rx_tail = 0U;   /* 任务里读 */
static volatile uint32_t    can_rx_drop = 0U;   /* 缓冲区满而丢掉的帧数 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  配置过滤器、启动 CAN、打开 FIFO0 接收中断。必须在 MX_CAN1_Init() 之后调用。
  * @retval HAL_OK / HAL_ERROR
  */
uint8_t CAN_Init(void)
{
  CAN_FilterTypeDef filter = {0};

  /* 过滤器组 0：掩码全 0 = 不比较，总线上所有报文都收进 FIFO0，调试够用。
     以后只想收固定几个 ID 时，把 IdHigh/IdLow 和对应的 Mask 填上即可，
     标准 ID 左移 5 位放进高 16 位。 */
  filter.FilterBank           = 0;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh         = 0x0000;
  filter.FilterIdLow          = 0x0000;
  filter.FilterMaskIdHigh     = 0x0000;
  filter.FilterMaskIdLow      = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation     = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  发送一帧标准数据帧（11 位 ID）
  * @param  id  标准 ID
  * @param  data 数据指针，len 为 0 时可传 NULL
  * @param  len  数据长度，0~8
  * @retval HAL_OK / HAL_ERROR
  */
uint8_t CAN_Send(uint32_t id, const uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t tx_mailbox;

  if (len > 8U)
  {
    len = 8U;
  }

  tx_header.StdId = id;
  tx_header.ExtId = 0U;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = len;
  tx_header.TransmitGlobalTime = DISABLE;

  /* 三个发送邮箱都满时返回 HAL_BUSY，由调用者决定要不要重发 */
  if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  取一帧收到的报文（非阻塞）。只应在任务上下文调用。
  * @param  frame 输出，取到的帧
  * @retval 1 = 取到一帧；0 = 当前没有待处理的帧
  */
uint8_t CAN_GetRxFrame(CAN_Frame_t *frame)
{
  if (can_rx_tail == can_rx_head)
  {
    return 0U;              /* 空 */
  }

  frame->id  = can_rx_buf[can_rx_tail].id;
  frame->len = can_rx_buf[can_rx_tail].len;
  for (uint8_t i = 0U; i < 8U; i++)
  {
    frame->data[i] = can_rx_buf[can_rx_tail].data[i];
  }

  can_rx_tail = (uint8_t)((can_rx_tail + 1U) % CAN_RX_QUEUE_LEN);
  return 1U;
}

uint32_t CAN_GetRxDropCount(void)
{
  return can_rx_drop;
}

/**
  * @brief  收到 CAN 报文的 HAL 回调（在 CAN 中断里执行）
  * @note   函数名不要改。FIFO0 挂起一次可能已积压多帧，这里循环取空。
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (hcan->Instance != CAN1)
  {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      break;
    }

    uint8_t next = (uint8_t)((can_rx_head + 1U) % CAN_RX_QUEUE_LEN);
    if (next == can_rx_tail)
    {
      can_rx_drop++;        /* 满了：丢最新一帧，保住已有的数据 */
      continue;
    }

    can_rx_buf[can_rx_head].id  = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId
                                                               : rx_header.ExtId;
    can_rx_buf[can_rx_head].len = (uint8_t)((rx_header.DLC > 8U) ? 8U : rx_header.DLC);
    for (uint8_t i = 0U; i < 8U; i++)
    {
      can_rx_buf[can_rx_head].data[i] = (i < can_rx_buf[can_rx_head].len) ? rx_data[i] : 0U;
    }

    can_rx_head = next;
  }
}
/* USER CODE END 1 */

