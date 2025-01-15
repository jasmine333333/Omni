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
/* Private macro -------------------------------------------------------------*/
hello_world::module::Fric::Config kFricConfig = {
    .min_reasonable_blt_spd = 14.0f,             ///< 最小合理弹丸射速 m/s
    .max_reasonable_blt_spd = 16.5f,             ///< 最大合理弹丸射速 m/s
    .min_target_blt_spd = 15.3f,         ///< 最小目标弹丸速度 m/s
    .max_target_blt_spd = 15.9f,         ///< 最大目标弹丸速度 m/s
    .default_spd_ref = 640.0f,            ///< 摩擦轮期望速度预设值 rad/s
    .default_spd_ref_backward = -100.0f,  ///< 摩擦轮反转目标速度
    .spd_gradient = 5.0f,

    .stuck_curr_thre = 14.0f,  ///< 用于判断摩擦轮堵转的电流阈值
    .spd_delta_thre = 10.0f,   ///< 用于判断摩擦轮速度保持恒定的阈值 (rad)
    .spd_err_thre = 5.0f,      ///< 用于判断摩擦轮速度跟上期望转速的阈值 (rad)
};
Fric unique_fric = Fric(kFricConfig);
Fric *CreateFric()
{
  static bool is_fric_created = false;
  if (!is_fric_created) {
    unique_fric.registerPid(CreatePidMotorFricLeft(),0x01); // 注册MultiNodesPID指针
    unique_fric.registerPid(CreatePidMotorFricRight,0x02); // 注册MultiNodesPID指针
    unique_fric.registerMotor(CreateMotorFricLeft,0x01);  // 注册电机指针
    unique_fric.registerMotor(CreateMotorFricRight,0x02);  // 注册电机指针
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
