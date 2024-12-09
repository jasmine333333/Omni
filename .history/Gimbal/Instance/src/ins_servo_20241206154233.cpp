/** 
 *******************************************************************************
 * @file      : ins_servo.cpp
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

#include "ins_servo.hpp"
#include "tim.h"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

hw_servo::Servo *unique_mini_pitch_servo_ptr = nullptr;
hw_servo::Servo *unique_scope_servo_ptr = nullptr;
           

hw_servo::Servo *CreateMiniPitchServo(void) {
  if (unique_mini_pitch_servo_ptr == nullptr) {
    unique_mini_pitch_servo_ptr = new hw_servo::Servo(&htim6, TIM_CHANNEL_2,
                                                      hw_servo::ServoInfo{
                                                        .operating_freq = 50.0f,
                                                        .min_pulse_duration = 500,
                                                        .max_pulse_duration = 2500,
                                                        .angle_range = 180.0f,
                                                      },
                                                      90);
  }
  return unique_mini_pitch_servo_ptr;
};
hw_servo::Servo *CreateScopeServo(void) {
  if (unique_scope_servo_ptr == nullptr) {
    unique_scope_servo_ptr = new hw_servo::Servo(&htim6, TIM_CHANNEL_3,
                                                      hw_servo::ServoInfo{
                                                        .operating_freq = 50.0f,
                                                        .min_pulse_duration = 500,
                                                        .max_pulse_duration = 2500,
                                                        .angle_range = 180.0f,
                                                      },
                                                      135);
  }
  return unique_scope_servo_ptr;
};

/* Private function definitions ----------------------------------------------*/
