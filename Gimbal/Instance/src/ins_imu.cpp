/** 
 *******************************************************************************
 * @file      :ins_imu.cpp
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
#include "ins_imu.hpp"
#include "spi.h"
/* Private constants ---------------------------------------------------------*/
const hello_world::imu::Imu::Config kImuInitParams = {
    .sample_num = 1000,
    .default_gyro_offset = {0, 0, 0}, ///< 默认陀螺仪偏置值
    .rot_mat_flatten = {0, 1, 0, 1, 0, 0, 0, 0, -1},
    .bmi088_hw_config = {
        .hspi = &hspi1,
        .acc_cs_port = GPIOA,
        .acc_cs_pin = GPIO_PIN_4,
        .gyro_cs_port = GPIOB,
        .gyro_cs_pin = GPIO_PIN_0,
    }

};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::imu::Imu *CreateImu(void) { 
    static hello_world::imu::Imu unique_imu = hello_world::imu::Imu(kImuInitParams);
    return &unique_imu;
};
/* Private function definitions ----------------------------------------------*/
