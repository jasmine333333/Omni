/** 
 *******************************************************************************
 * @file      : ins_pid.hpp
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
#ifndef INSTANCE_INS_PID_HPP_
#define INSTANCE_INS_PID_HPP_

/* Includes ------------------------------------------------------------------*/
#include "pid.hpp"

namespace hw_pid = hello_world::pid;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hw_pid::MultiNodesPid* CreatePidMotorYaw();
hw_pid::MultiNodesPid* CreatePidMotorPitch();
hw_pid::MultiNodesPid* CreatePidMotorFricLeft();
hw_pid::MultiNodesPid* CreatePidMotorFricRight();
hw_pid::MultiNodesPid* CreatePidMotorFeed();
#endif /* INSTANCE_INS_PID_HPP_ */
