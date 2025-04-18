/** 
 *******************************************************************************
 * @file      :ins_vision.cpp
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
#include "ins_vision.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hw_vision::Vision::Config kvisionConfig= {
    .default_blt_spd = 23.7f,
    .hfov = 0.785,
    .vfov = 0.6183,
};
hw_vision::Vision unique_vision = hw_vision::Vision(kvisionConfig);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_vision::Vision* CreateVision() { return &unique_vision; };
/* Private function definitions ----------------------------------------------*/
