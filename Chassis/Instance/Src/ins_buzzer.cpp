/** 
 *******************************************************************************
 * @file      : ins_buzzer.cpp
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
#include "ins_buzzer.hpp"

#include "tim.h"
/* Private constants ---------------------------------------------------------*/
const hw_buzzer::TuneListInfo kTuneListInfo = {
    .intensity_scale = 1.0f,
    .tune_duration = 125,
    .list =
        {
               hw_buzzer::kTuneA3,  hw_buzzer::kTuneA3, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneD4, hw_buzzer::kTuneG4,  hw_buzzer::kTuneG4, hw_buzzer::kTuneD4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4,  hw_buzzer::kTuneG4, hw_buzzer::kTuneG4,
               hw_buzzer::kTuneG4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneF4S, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,  hw_buzzer::kTuneD4, hw_buzzer::kTuneE4,
               hw_buzzer::kTuneG4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,  hw_buzzer::kTuneD4, hw_buzzer::kTuneG4,
               hw_buzzer::kTuneD4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneA4, hw_buzzer::kTuneE4, hw_buzzer::kTuneG4,  hw_buzzer::kTuneD5, hw_buzzer::kTuneG4,
               hw_buzzer::kTuneA4,  hw_buzzer::kTuneE5, hw_buzzer::kTuneA4, hw_buzzer::kTuneD5, hw_buzzer::kTuneE5,  hw_buzzer::kTuneA3, hw_buzzer::kTuneE4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneG4, hw_buzzer::kTuneA3,  hw_buzzer::kTuneD4, hw_buzzer::kTuneE4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4,  hw_buzzer::kTuneG4, hw_buzzer::kTuneA4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,  hw_buzzer::kTuneD4, hw_buzzer::kTuneE4,
               hw_buzzer::kTuneG4,  hw_buzzer::kTuneA3, hw_buzzer::kTuneD4, hw_buzzer::kTuneA3, hw_buzzer::kTuneG3,  hw_buzzer::kTuneG3, hw_buzzer::kTuneC4,
               hw_buzzer::kTuneC4,  hw_buzzer::kTuneC4, hw_buzzer::kTuneD4, hw_buzzer::kTuneC4, hw_buzzer::kTuneD4,  hw_buzzer::kTuneE4, hw_buzzer::kTuneD4,
               hw_buzzer::kTuneE4,  hw_buzzer::kTuneG4, hw_buzzer::kTuneE4, hw_buzzer::kTuneG4, hw_buzzer::kTuneA4,  hw_buzzer::kTuneG4, hw_buzzer::kTuneA4,
               hw_buzzer::kTuneC5,  hw_buzzer::kTuneA4, hw_buzzer::kTuneC5, hw_buzzer::kTuneD5, hw_buzzer::kTuneC5,  hw_buzzer::kTuneD5, hw_buzzer::kTuneE5,
               hw_buzzer::kTuneD5,  hw_buzzer::kTuneE5, hw_buzzer::kTuneG5, hw_buzzer::kTuneA5, hw_buzzer::kTuneA5,  hw_buzzer::kTuneA5, hw_buzzer::kTuneA5,
               hw_buzzer::kTuneEnd,
               },
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hw_buzzer::Buzzer* unique_buzzer_ptr = nullptr;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_buzzer::Buzzer* CreateBuzzer(void)
{
  // method 1: use point
  if (unique_buzzer_ptr == nullptr) {
    unique_buzzer_ptr = new hw_buzzer::Buzzer(&htim4, TIM_CHANNEL_3, hw_buzzer::PlayConfig::kSinglePlayback, &kTuneListInfo);
  }
  return unique_buzzer_ptr;
  // method 2: use static
  // static hw_buzzer::Buzzer unique_buzzer(&htim4, TIM_CHANNEL_3, hw_buzzer::kPlayConfigSinglePlayback, &kTuneListInfo);
  // return &unique_buzzer;
};
/* Private function definitions ----------------------------------------------*/
