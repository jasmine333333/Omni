/** 
 *******************************************************************************
 * @file      : communication_tools.cpp
 * @brief     : 
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "communication_tools.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

CAN_TxHeaderTypeDef can_tx_header_debug = {0};  ///< CAN 发送数据帧头定义 【dubug用】
uint32_t can_tx_mail_box_debug = 0;             ///< CAN 发送数据帧邮箱 【dubug用】
size_t can_tx_error_times = 0;                  ///< CAN 发送数据帧错误次数 【dubug用】

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

void InitCanFilter(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef canfilter;

  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_16BIT;

  // filtrate any ID you want here
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;

  canfilter.FilterActivation = ENABLE;
  canfilter.SlaveStartFilterBank = 14;

  // use different filter for can1&can2
  if (hcan == &hcan1) {
    canfilter.FilterBank = 0;
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
  }
  if (hcan == &hcan2) {
    canfilter.FilterBank = 14;
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO1;
  }
  if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK) {
    Error_Handler();
  }
};

void SendCanData(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t tx_data[8])
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t mail_box = 0;
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

  can_tx_header_debug = tx_header;
  can_tx_mail_box_debug = mail_box;

  if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &mail_box) != HAL_OK) {
    can_tx_error_times++;
  }
};
/* Private function definitions ----------------------------------------------*/
