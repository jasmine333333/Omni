/** 
 *******************************************************************************
 * @file      : ins_buzzer.hpp
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
#ifndef INSTANCE_INS_BUZZER_HPP_
#define INSTANCE_INS_BUZZER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "buzzer.hpp"

namespace hw_buzzer = hello_world::buzzer;

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hw_buzzer::Buzzer* CreateBuzzer(void);
#endif /* INSTANCE_INS_BUZZER_HPP_ */
