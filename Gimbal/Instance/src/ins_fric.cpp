/** 
 *******************************************************************************
 * @file      :ins_fric.cpp
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
#include "ins_fric.hpp"
#include "ins_pid.hpp"
#include "ins_motor.hpp"
#include "fric_2motor.hpp"
/* Private macro -------------------------------------------------------------*/
hello_world::module::Fric::Config kFricConfig = {
    .default_spd_ref = 650.0f,            ///< 摩擦轮期望速度预设值 rad/s
    .default_spd_ref_backward = -100.0f,  ///< 摩擦轮反转目标速度

    .stuck_curr_thre = 14.0f,  ///< 用于判断摩擦轮堵转的电流阈值
    .spd_delta_thre = 10.0f,   ///< 用于判断摩擦轮速度保持恒定的阈值 (rad)
    .spd_err_thre = 5.0f,      ///< 用于判断摩擦轮速度跟上期望转速的阈值 (rad)
    .spd_stop_thre = 5.0f,
    .opt_spd_same_pid_enabled = true,
   .opt_blt_spd_cl = {
        .is_enabled = true,
        .min_reasonable_blt_spd = 20.0f, //20.0f
        .max_reasonable_blt_spd = 28.0f,//28.0f
        .min_target_blt_spd = 23.35f,//23.2f
        .max_target_blt_spd = 24.2f,//24.25f
        .spd_gradient = 0.5f   //1.0f
    }    
};
Fric unique_fric = Fric(kFricConfig);
Fric *CreateFric()
{
  static bool is_fric_created = false;
  if (!is_fric_created) {
    unique_fric.registerPid(CreatePidMotorFricLeft(),Fric::PidIdx::kFirst); // 注册MultiNodesPID指针
    unique_fric.registerPid(CreatePidMotorFricRight(),Fric::PidIdx::kSecond); // 注册MultiNodesPID指针
    unique_fric.registerPid(CreatePidSamespd(),Fric::PidIdx::kSameSpd);
    unique_fric.registerMotor(CreateMotorFricLeft(),Fric::MotorIdx::kFirst);  // 注册电机指针
    unique_fric.registerMotor(CreateMotorFricRight(),Fric::MotorIdx::kSecond);  // 注册电机指针
    is_fric_created = true;
  }
  return &unique_fric;

}
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
