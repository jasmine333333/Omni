/** 
 *******************************************************************************
 * @file      : ins_comm.cpp
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

#include "ins_comm.hpp"

#include "DT7.hpp"
#include "can.h"
#include "usart.h"

/* Private macro -------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

typedef hello_world::comm::CanRxMgr CanRxMgr;
typedef hello_world::comm::CanTxMgr CanTxMgr;

typedef hello_world::comm::UartRxMgr UartRxMgr;
typedef hello_world::comm::UartTxMgr UartTxMgr;

/* Private constants ---------------------------------------------------------*/

const size_t kRxvisionBufferSize = 13;

/* Private variables ---------------------------------------------------------*/

static bool is_can_1_rx_mgr_inited = false;
static CanRxMgr can_1_rx_mgr = CanRxMgr();

static bool is_can1_tx_mgr_inited = false;
static CanTxMgr can_1_tx_mgr = CanTxMgr();

static bool is_can_2_rx_mgr_inited = false;
static CanRxMgr can_2_rx_mgr = CanRxMgr();

static bool is_can2_tx_mgr_inited = false;
static CanTxMgr can_2_tx_mgr = CanTxMgr();

static bool is_vision_rx_mgr_inited = false;
static UartRxMgr vision_rx_mgr = UartRxMgr();

static bool is_vision_tx_mgr_inited = false;
static UartTxMgr vision_tx_mgr = UartTxMgr();

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

CanRxMgr* CreateCan1RxMgr(void)
{
  if (!is_can_1_rx_mgr_inited) {
    can_1_rx_mgr.init(&hcan1, CanRxMgr::RxType::kFifo0);
    can_1_rx_mgr.clearReceiver();
    is_can_1_rx_mgr_inited = true;
  }
  return &can_1_rx_mgr;
};

CanTxMgr* CreateCan1TxMgr(void)
{
  if (!is_can1_tx_mgr_inited) {
    can_1_tx_mgr.init(&hcan1);
    can_1_tx_mgr.clearTransmitter();
    is_can1_tx_mgr_inited = true;
  }
  return &can_1_tx_mgr;
};

CanRxMgr* CreateCan2RxMgr(void)
{
  if (!is_can_2_rx_mgr_inited) {
    can_2_rx_mgr.init(&hcan2, CanRxMgr::RxType::kFifo1);
    can_2_rx_mgr.clearReceiver();
    is_can_2_rx_mgr_inited = true;
  }
  return &can_2_rx_mgr;
};
CanTxMgr* CreateCan2TxMgr(void)
{
  if (!is_can2_tx_mgr_inited) {
    can_2_tx_mgr.init(&hcan2);
    can_2_tx_mgr.clearTransmitter();
    is_can2_tx_mgr_inited = true;
  }
  return &can_2_tx_mgr;
};

UartRxMgr* CreateVisionRxMgr(void)
{
  if (!is_vision_rx_mgr_inited) {
    vision_rx_mgr.init(&huart6, UartRxMgr::EofType::kIdle, kRxvisionBufferSize, kRxvisionBufferSize);
    vision_rx_mgr.clearReceiver();
    is_vision_rx_mgr_inited = true;
  }
  return &vision_rx_mgr;
};
UartTxMgr* CreateVisionTxMgr(void)
{
  if (!is_vision_tx_mgr_inited) {
    vision_tx_mgr.init(&huart6, kRxvisionBufferSize);
    vision_tx_mgr.clearTransmitter();
    is_vision_tx_mgr_inited = true;
  }
  return &vision_tx_mgr;
};

/* Private function definitions ----------------------------------------------*/
