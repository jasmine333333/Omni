/** 
 *******************************************************************************
 * @file      : ins_pid.cpp
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
#include "ins_pid.hpp"

#include "motor.hpp"

/* Private constants ---------------------------------------------------------*/
const float kMaxPidOutWheel = 20.0f;  ///< 3508电流控制的最大输出
const float kMaxPidOutFollowOmega = 1.0f;

const hw_pid::OutLimit kOutLimitWheel = hw_pid::OutLimit(true, -kMaxPidOutWheel, kMaxPidOutWheel);
const hw_pid::OutLimit kOutLimitFollowOmega = hw_pid::OutLimit(true, -kMaxPidOutFollowOmega, kMaxPidOutFollowOmega);

const hw_pid::MultiNodesPid::ParamsList kPidParamsWheel = {
    {
     .auto_reset = true,
     .kp = 0.6,
     .ki = 0.01,
     .kd = 0,
     .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
     .inte_anti_windup = hw_pid::InteAntiWindup(true, -1.0f, 1.0f),
     .out_limit = kOutLimitWheel,
     },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsFollowOmega = {
    {
     .auto_reset = true,
     .kp = 0.7,
     .ki = 0,
     .kd = 0,
     .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 1.0, 0.1),
     .period_sub = hw_pid::PeriodSub(true, 2 * PI),
     .out_limit = kOutLimitFollowOmega,
     },
    //  这个双环不一定有用，拿的是YAW轴电机速度
    // 这玩意儿云台动起来就有额外偏差，有点问题，建议注释掉只调单环
    // {
    //  .auto_reset = true,
    //  .kp = 3,
    //  .ki = 0,
    //  .kd = 0,
    //  .period_sub = hw_pid::PeriodSub(false, 0),
    //  .out_limit = hw_pid::OutLimit(true, -80, 80),
    //  },
};

const hw_pid::MultiNodesPid::Type kPidTypeCascade = hw_pid::MultiNodesPid::Type::kCascade;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_pid::MultiNodesPid unique_pid_wheel_left_front(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel);
hw_pid::MultiNodesPid unique_pid_wheel_left_rear(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel);
hw_pid::MultiNodesPid unique_pid_wheel_right_front(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel);
hw_pid::MultiNodesPid unique_pid_wheel_right_rear(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel);

hw_pid::MultiNodesPid unique_pid_follow_omega(kPidTypeCascade, kOutLimitFollowOmega, kPidParamsFollowOmega);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_pid::MultiNodesPid* CreatePidMotorWheelLeftFront() { return &unique_pid_wheel_left_front; };
hw_pid::MultiNodesPid* CreatePidMotorWheelLeftRear() { return &unique_pid_wheel_left_rear; };
hw_pid::MultiNodesPid* CreatePidMotorWheelRightFront() { return &unique_pid_wheel_right_front; };
hw_pid::MultiNodesPid* CreatePidMotorWheelRightRear() { return &unique_pid_wheel_right_rear; };
hw_pid::MultiNodesPid* CreatePidFollowOmega() { return &unique_pid_follow_omega; };
/* Private function definitions ----------------------------------------------*/
