/** 
 *******************************************************************************
 * @file      :communication_tools.hpp
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
#ifndef ROBOT_COMPONETS_COMMUNICATION_TOOLS_HPP_
#define ROBOT_COMPONETS_COMMUNICATION_TOOLS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
#include "can.h"
#include "super_cap.hpp"
#include "gimbal_chassis_comm.hpp"
#include "iwdg.h"
#include "motor.hpp"
#include "usart.h"
/* Exported macro ------------------------------------------------------------*/

void InitCanFilter(CAN_HandleTypeDef *hcan);
void SendCanData(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t tx_data[8]);
#endif /* ROBOT_COMPONETS_COMMUNICATION_TOOLS_HPP_ */
