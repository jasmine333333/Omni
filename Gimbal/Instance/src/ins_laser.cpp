/** 
 *******************************************************************************
 * @file      : ins_laser.cpp
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
#include "ins_laser.hpp"

#include "tim.h"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hw_laser::Laser* unique_laser = nullptr;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_laser::Laser *CreateLaser(void) { 
  if (unique_laser == nullptr){
    unique_laser = new hw_laser::Laser(&htim3, TIM_CHANNEL_3, 75);
  }
  unique_laser->disable();
  return unique_laser;
};
/* Private function definitions ----------------------------------------------*/
