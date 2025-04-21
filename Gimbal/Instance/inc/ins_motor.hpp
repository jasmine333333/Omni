/** 
 *******************************************************************************
 * @file      : ins_motor.hpp
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
#ifndef INSTANCE_INS_MOTOR_HPP_
#define INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"

namespace hw_motor = hello_world::motor;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hw_motor::DM_J4310* CreateMotorYaw();
hw_motor::DM_J4310* CreateMotorPitch();
hw_motor::Motor* CreateMotorFricLeft();
hw_motor::Motor* CreateMotorFricRight();
hw_motor::Motor* CreateMotorFeed();
#endif /* INSTANCE_INS_MOTOR_HPP_ */
