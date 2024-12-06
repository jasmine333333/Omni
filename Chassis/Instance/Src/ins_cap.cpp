/** 
 *******************************************************************************
 * @file      : ins_cap.cpp
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
#include "ins_cap.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/

typedef hello_world::cap::SuperCap Cap;

/* Private constants ---------------------------------------------------------*/

const Cap::Config kCapConfig = {
    .max_charge_volt = 26.0f,     ///< 最大充电电压，实际可能可以比该值更高，单位：V
    .min_valid_volt = 16.0f,      ///< 最小有效放电电压，小于此值之后不允许开启超电，单位：V
    .auto_disable_power = 20.0f,  ///< 自动关闭能量阈值，单位：%，当剩余能量低于此值时，自动关闭超电，[0, 100)
    .min_enable_power = 40.0f,    ///< 最小开启能量阈值，单位：%，当剩余能量高于此值时，才能开启超电，[0, 100)
    .pwr_filter_beta = 0.7f,      ///< 剩余能量的低通滤波系数，[0, 1]，为 0 时即不滤波
};

/* Private variables ---------------------------------------------------------*/
static Cap unique_cap = Cap(100, kCapConfig, Cap::Version::kVer2024);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

Cap* CreateCap(void) { return &unique_cap; };

/* Private function definitions ----------------------------------------------*/
