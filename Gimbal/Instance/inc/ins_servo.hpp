/** 
 *******************************************************************************
 * @file      : ins_servo.hpp
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
#ifndef INSTANCE_INS_SERVO_HPP_
#define INSTANCE_INS_SERVO_HPP_

/* Includes ------------------------------------------------------------------*/
#include "servo.hpp"

namespace hw_servo = hello_world::servo;

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

hw_servo::Servo *CreateMiniPitchServo(void);
hw_servo::Servo *CreateScopeServo(void);

#endif /* INSTANCE_INS_SERVO_HPP_ */
