/** 
 *******************************************************************************
 * @file      : comm_task.cpp
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
#include "comm_task.hpp"

// hal
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"

// HW-Components
#include "tick.hpp"

// custom
#include "ins_all.hpp"

using hello_world::comm::CanRxMgr;
using hello_world::comm::CanTxMgr;
using hello_world::comm::UartRxMgr;
using hello_world::comm::UartTxMgr;
using hello_world::remote_control::DT7;
using hello_world::referee::Referee;

/* Private macro -------------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static CanRxMgr* can1_rx_mgr_ptr = nullptr;
static CanTxMgr* can1_tx_mgr_ptr = nullptr;

static CanRxMgr* can2_rx_mgr_ptr = nullptr;
static CanTxMgr* can2_tx_mgr_ptr = nullptr;

static UartRxMgr* rc_rx_mgr_ptr = nullptr;

static UartRxMgr* rfr_rx_mgr_ptr = nullptr;
static UartTxMgr* rfr_tx_mgr_ptr = nullptr;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void PrivatePointerInit(void);
static void CommAddReceiver(void);
static void CommAddTransmitter(void);
static void CommHardWareInit(void);

/* Exported function definitions ---------------------------------------------*/

void CommTaskInit(void)
{
  PrivatePointerInit();
  CommAddReceiver();
  CommAddTransmitter();
  CommHardWareInit();
};

void CommTask(void)
{
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr", can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr", can2_tx_mgr_ptr);
  if (can1_tx_mgr_ptr == nullptr || can2_tx_mgr_ptr == nullptr) {
    return;
  }
  can1_tx_mgr_ptr->startTransmit();
  can2_tx_mgr_ptr->startTransmit();
  
};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr", can1_rx_mgr_ptr);
  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr", can2_rx_mgr_ptr);
  if (can1_rx_mgr_ptr == nullptr || can2_rx_mgr_ptr == nullptr) {
    return;
  }
  can1_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
  can2_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr", can1_rx_mgr_ptr);
  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr", can2_rx_mgr_ptr);
  if (can1_rx_mgr_ptr == nullptr || can2_rx_mgr_ptr == nullptr) {
    return;
  }
  can1_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
  can2_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr", can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr", can2_tx_mgr_ptr);
  if (can1_tx_mgr_ptr == nullptr || can2_tx_mgr_ptr == nullptr) {
    return;
  }
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan)
{
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr", can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr", can2_tx_mgr_ptr);
  if (can1_tx_mgr_ptr == nullptr || can2_tx_mgr_ptr == nullptr) {
    return;
  }
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan)
{
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr", can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr", can2_tx_mgr_ptr);
  if (can1_tx_mgr_ptr == nullptr || can2_tx_mgr_ptr == nullptr) {
    return;
  }
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr", can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr", can2_tx_mgr_ptr);
  if (can1_tx_mgr_ptr == nullptr || can2_tx_mgr_ptr == nullptr) {
    return;
  }
  can1_tx_mgr_ptr->errorCallback(hcan);
  can2_tx_mgr_ptr->errorCallback(hcan);
}

uint32_t uart_rx_tick = 0;
uint32_t uart_rx_cb_in = 0;
float uart3_uticks = 0;
float uart6_uticks = 0;
uint32_t uart3_rx_cnt = 0;
uint32_t uart6_rx_cnt = 0;
/**
 * @brief   UART接收回调函数
 * @param   none
 * @retval  none
 * @note    none
**/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
  uart_rx_tick = hello_world::tick::GetTickMs();
  uart_rx_cb_in++;
  // 遥控器
  if (huart == &huart3) {
    uart3_rx_cnt++;
    uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim2);
    rc_rx_mgr_ptr->rxEventCallback(huart, Size);
    HAL_IWDG_Refresh(&hiwdg);
    uint32_t tick_end = __HAL_TIM_GET_COUNTER(&htim2);
    uart3_uticks = (tick_end - tick_start) / (84.0f * 1e3);
    uart3_rx_cnt--;
  }
  // 裁判系统
  else if (huart == &huart6) {
    uart6_rx_cnt++;
    uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim2);
    rfr_rx_mgr_ptr->rxEventCallback(huart, Size);
    uint32_t tick_end = __HAL_TIM_GET_COUNTER(&htim2);
    uart6_uticks = (tick_end - tick_start) / (84.0f * 1e3);
    uart6_rx_cnt--;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
  // 遥控器
  if (huart == &huart3) {
    rc_rx_mgr_ptr->startReceive();
  }
  // 裁判系统
  else if (huart == &huart6) {
    rfr_rx_mgr_ptr->startReceive();
  }
};

/* Private function definitions ----------------------------------------------*/

static void PrivatePointerInit(void)
{
  can1_rx_mgr_ptr = CreateCan1RxMgr();
  can1_tx_mgr_ptr = CreateCan1TxMgr();

  can2_rx_mgr_ptr = CreateCan2RxMgr();
  can2_tx_mgr_ptr = CreateCan2TxMgr();

  rc_rx_mgr_ptr = CreateRcRxMgr();

  rfr_rx_mgr_ptr = CreateRfrRxMgr();
  rfr_tx_mgr_ptr = CreateRfrTxMgr();
}

static void CommAddReceiver(void)
{
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr", can1_rx_mgr_ptr);
  can1_rx_mgr_ptr->addReceiver(CreateGimbalChassisComm());
  can1_rx_mgr_ptr->addReceiver(CreateMotorYaw());  // 用于底盘控制，只接受消息

  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr", can2_rx_mgr_ptr);
  can2_rx_mgr_ptr->addReceiver(CreateCap());
  can2_rx_mgr_ptr->addReceiver(CreateMotorWheelLeftFront());
  can2_rx_mgr_ptr->addReceiver(CreateMotorWheelLeftRear());
  can2_rx_mgr_ptr->addReceiver(CreateMotorWheelRightRear());
  can2_rx_mgr_ptr->addReceiver(CreateMotorWheelRightFront());

  HW_ASSERT(rc_rx_mgr_ptr != nullptr, "rc_rx_mgr_ptr is nullptr", rc_rx_mgr_ptr);
  rc_rx_mgr_ptr->addReceiver(CreateRemoteControl());

  HW_ASSERT(rfr_rx_mgr_ptr != nullptr, "rfr_rx_mgr_ptr is nullptr", rfr_rx_mgr_ptr);
  rfr_rx_mgr_ptr->addReceiver(CreateReferee());
}

static void CommAddTransmitter()
{
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr", can1_tx_mgr_ptr);
  can1_tx_mgr_ptr->addTransmitter(CreateGimbalChassisComm());

  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr", can2_tx_mgr_ptr);
  can2_tx_mgr_ptr->addTransmitter(CreateCap());
  can2_tx_mgr_ptr->addTransmitter(CreateMotorWheelLeftFront());
  can2_tx_mgr_ptr->addTransmitter(CreateMotorWheelLeftRear());
  can2_tx_mgr_ptr->addTransmitter(CreateMotorWheelRightRear());
  can2_tx_mgr_ptr->addTransmitter(CreateMotorWheelRightFront());

  HW_ASSERT(rfr_tx_mgr_ptr != nullptr, "rfr_tx_mgr_ptr is nullptr", rfr_tx_mgr_ptr);
  rfr_tx_mgr_ptr->addTransmitter(CreateReferee());
}

void CommHardWareInit(void)
{
  // CAN init
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr", can1_rx_mgr_ptr);
  can1_rx_mgr_ptr->filterInit();
  can1_rx_mgr_ptr->startReceive();
  HAL_CAN_Start(&hcan1);

  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr", can2_rx_mgr_ptr);
  can2_rx_mgr_ptr->filterInit();
  can2_rx_mgr_ptr->startReceive();
  HAL_CAN_Start(&hcan2);

  // rc DMA init
  HW_ASSERT(rc_rx_mgr_ptr != nullptr, "rc_rx_mgr_ptr is nullptr", rc_rx_mgr_ptr);
  rc_rx_mgr_ptr->startReceive();
  // rfr DMA Init
  HW_ASSERT(rfr_rx_mgr_ptr != nullptr, "rfr_rx_mgr_ptr is nullptr", rfr_rx_mgr_ptr);
  rfr_rx_mgr_ptr->startReceive();
};