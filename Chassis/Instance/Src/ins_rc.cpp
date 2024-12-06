/** 
 *******************************************************************************
 * @file      : ins_rc.cpp
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
#include "ins_rc.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_rc::DT7 unique_rc = hw_rc::DT7();

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

hw_rc::DT7* CreateRemoteControl() { return &unique_rc; };

/* Private function definitions ----------------------------------------------*/
