/** 
 *******************************************************************************
 * @file      : ins_comm.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INSTANCE_INS_COMM_HPP_
#define INSTANCE_INS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/

#include "can_rx_mgr.hpp"
#include "can_tx_mgr.hpp"
#include "uart_rx_mgr.hpp"
#include "uart_tx_mgr.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

hello_world::comm::CanRxMgr* CreateCan1RxMgr(void);
hello_world::comm::CanTxMgr* CreateCan1TxMgr(void);

hello_world::comm::CanRxMgr* CreateCan2RxMgr(void);
hello_world::comm::CanTxMgr* CreateCan2TxMgr(void);

hello_world::comm::UartRxMgr* CreateRfrRxMgr(void);
hello_world::comm::UartTxMgr* CreateRfrTxMgr(void);

hello_world::comm::UartRxMgr* CreateRcRxMgr(void);

#endif /* INSTANCE_INS_COMM_HPP_ */
