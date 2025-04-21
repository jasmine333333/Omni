/** 
 *******************************************************************************
 * @file      :main_task.hpp
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
#ifndef MAIN_TASK_HPP_
#define MAIN_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
static uint8_t car_version = 0;
/* Exported macro ------------------------------------------------------------*/
enum
{
    kxiao_omni = 0,
    kxiaoxiao_omni = 1,
};
enum{
    version_kxiao_omni = 0,
    version_kxiaoxiao_omni = 1,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void MainTaskInit(void);
void MainTask(void);

#ifdef __cplusplus
}

#endif
#endif /* MAIN_TASK_HPP_ */
