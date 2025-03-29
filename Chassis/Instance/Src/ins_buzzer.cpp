/** 
 *******************************************************************************
 * @file      :ins_buzzer.cpp
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
  .tune_duration = 275,
  .list =
      {   // 12 33 33    33 21 23    33 12 35   
          // 66 11 33   23 33 33   17d 6d5d 6d6d  
          // 33 22 53   33 33 12   35 66 11  
          // 33 23 35   55 17d 6d5d  6d6d 33 21 
          // 7d5d 5d6d 6d6d   17d 6d5d 6d6d   33 21 7d1   
          // 11 11 11   11
          hw_buzzer::kTuneC4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,
          hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneC4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4,
          hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneC4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneG4,

          hw_buzzer::kTuneA4, hw_buzzer::kTuneA4, hw_buzzer::kTuneC4, hw_buzzer::kTuneC4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,
          hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4,
          hw_buzzer::kTuneC4, hw_buzzer::kTuneB3, hw_buzzer::kTuneA3, hw_buzzer::kTuneG3, hw_buzzer::kTuneA3, hw_buzzer::kTuneA3,

          hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneD4, hw_buzzer::kTuneG4, hw_buzzer::kTuneE4,
          hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneC4, hw_buzzer::kTuneD4,
          hw_buzzer::kTuneE4, hw_buzzer::kTuneG4, hw_buzzer::kTuneA4, hw_buzzer::kTuneA4, hw_buzzer::kTuneC4, hw_buzzer::kTuneC4,

          hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneG4,
          hw_buzzer::kTuneG4, hw_buzzer::kTuneG4, hw_buzzer::kTuneC4, hw_buzzer::kTuneB3, hw_buzzer::kTuneA3, hw_buzzer::kTuneG3,
          hw_buzzer::kTuneA3, hw_buzzer::kTuneA3, hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneC4,

          hw_buzzer::kTuneB3, hw_buzzer::kTuneG3, hw_buzzer::kTuneG3, hw_buzzer::kTuneA3, hw_buzzer::kTuneA3, hw_buzzer::kTuneA3,
          hw_buzzer::kTuneC4, hw_buzzer::kTuneB3, hw_buzzer::kTuneA3, hw_buzzer::kTuneG3, hw_buzzer::kTuneA3, hw_buzzer::kTuneA3,
          hw_buzzer::kTuneE4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneC4, hw_buzzer::kTuneB3, hw_buzzer::kTuneC4,

          hw_buzzer::kTuneC4, hw_buzzer::kTuneC4, hw_buzzer::kTuneC4, hw_buzzer::kTuneC4, hw_buzzer::kTuneC4, hw_buzzer::kTuneC4,
          hw_buzzer::kTuneC4, hw_buzzer::kTuneC4, hw_buzzer::kTuneEnd, 
          // // 第一段旋律
          // hw_buzzer::kTuneC4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneC4, // C-D-E-D-C
          // hw_buzzer::kTuneC4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneC4, // C-D-E-D-C
          // hw_buzzer::kTuneE4, hw_buzzer::kTuneF4, hw_buzzer::kTuneG4, hw_buzzer::kTuneF4, hw_buzzer::kTuneE4, // E-F-G-F-E
          // hw_buzzer::kTuneD4, hw_buzzer::kTuneC4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, // D-C-D-E-D

          // // 第二段旋律（略微变调）
          // hw_buzzer::kTuneC4, hw_buzzer::kTuneE4, hw_buzzer::kTuneF4, hw_buzzer::kTuneG4, hw_buzzer::kTuneF4, // C-E-F-G-F
          // hw_buzzer::kTuneE4, hw_buzzer::kTuneC4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, // E-C-D-E-D
          // hw_buzzer::kTuneC4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneC4, hw_buzzer::kTuneEnd, // C-D-E-C

          // // 第三段旋律（变调与节奏变化）
          // hw_buzzer::kTuneG4, hw_buzzer::kTuneA4, hw_buzzer::kTuneG4, hw_buzzer::kTuneF4, hw_buzzer::kTuneE4, // G-A-G-F-E
          // hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneF4, hw_buzzer::kTuneG4, hw_buzzer::kTuneA4, // D-E-F-G-A
          // hw_buzzer::kTuneA4, hw_buzzer::kTuneC5, hw_buzzer::kTuneB4, hw_buzzer::kTuneC5, hw_buzzer::kTuneEnd, // A-C5-B-C5

          // // 第四段旋律（转调）
          // hw_buzzer::kTuneF4, hw_buzzer::kTuneG4, hw_buzzer::kTuneA4, hw_buzzer::kTuneB4, hw_buzzer::kTuneA4, // F-G-A-B-A
          // hw_buzzer::kTuneG4, hw_buzzer::kTuneF4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneC4, // G-F-E-D-C
          // hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, hw_buzzer::kTuneF4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, // D-E-F-E-D
          // hw_buzzer::kTuneC4, hw_buzzer::kTuneEnd, // C

          // // 结尾部分
          // hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneC4, hw_buzzer::kTuneD4, hw_buzzer::kTuneE4, // E-D-C-D-E
          // hw_buzzer::kTuneF4, hw_buzzer::kTuneG4, hw_buzzer::kTuneA4, hw_buzzer::kTuneA4, hw_buzzer::kTuneG4, // F-G-A-A-G
          // hw_buzzer::kTuneF4, hw_buzzer::kTuneE4, hw_buzzer::kTuneD4, hw_buzzer::kTuneEnd, // F-E-D-End
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
