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
/* Private constants ---------------------------------------------------------*/
const float kImuRotMatFlatten[9] = {0, 1, 0, -1, 0, 0, 0, 0, 1};
const robot::Imu::Config kImuInitConfig = {
    .offset_max_count = 1000,
    .acc_threshold = 10.0f,
    .gyro_threshold = 0.1f,
    .samp_freq = 1000.0f,
    .kp = 0.2f,
    .ki = 0.0f,
    .bmi088_hw_config_ptr = &robot::Imu::kBmi088DefaultHWConfig,
    .rot_mat_ptr = kImuRotMatFlatten,
    .bmi088_config_ptr = &robot::Imu::kBmi088DefaultConfig,
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// (LKY) 不能如下定义全局变量, 因为IMU的初始化(Imu.bmi088_ptr_)依赖于hspi1,
// (LKY) 必须等hspi1初始化结束(MX_SPI1_Init()完成), 才能初始化unique_imu
// robot::Imu unique_imu = robot::Imu(kImuInitConfig);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Imu *CreateImu(void) {
    static robot::Imu unique_imu = robot::Imu(kImuInitConfig); 
    return &unique_imu; 
};

/* Private function definitions ----------------------------------------------*/
