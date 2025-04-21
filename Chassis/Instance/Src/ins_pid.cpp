/**
 *******************************************************************************
 * @file      :ins_pid.cpp
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
#include "main_task.hpp"
/* Private constants ---------------------------------------------------------*/
const float kMaxPidOutWheel = 20.0f; ///< 3508电流控制的最大输出
const float kMaxPidOutFollowOmega = 1.0f;

const hw_pid::OutLimit kOutLimitWheel = hw_pid::OutLimit(true, -kMaxPidOutWheel, kMaxPidOutWheel);
const hw_pid::OutLimit kOutLimitFollowOmega = hw_pid::OutLimit(true, -kMaxPidOutFollowOmega, kMaxPidOutFollowOmega);

const hw_pid::MultiNodesPid::ParamsList kPidParamsWheel_1 = {
    {
        .auto_reset = true,
        .kp = 2.15f, // 2.15f
        .ki = 0,
        .kd = 0,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .inte_anti_windup = hw_pid::InteAntiWindup(true, -1.0f, 1.0f),
        .out_limit = kOutLimitWheel,
    },
};
const hw_pid::MultiNodesPid::ParamsList kPidParamsWheel_2 = {
    {
        .auto_reset = true,
        .kp = 2.15f, // 2.15f
        .ki = 0,
        .kd = 0,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .inte_anti_windup = hw_pid::InteAntiWindup(true, -1.0f, 1.0f),
        .out_limit = kOutLimitWheel,
    },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsFollowOmega_1 = {
    {
        .auto_reset = true,
        .kp = 1.48f,
        .ki = 0,
        .kd = 50,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 1.0, 0.1),
        .period_sub = hw_pid::PeriodSub(true, 2 * PI),
        .out_limit = kOutLimitFollowOmega,
    },
};
const hw_pid::MultiNodesPid::ParamsList kPidParamsFollowOmega_2 = {
    {
        .auto_reset = true,
        .kp = 1.48f,
        .ki = 0,
        .kd = 50,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 1.0, 0.1),
        .period_sub = hw_pid::PeriodSub(true, 2 * PI),
        .out_limit = kOutLimitFollowOmega,
    },
};

const hw_pid::MultiNodesPid::Type kPidTypeCascade = hw_pid::MultiNodesPid::Type::kCascade;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_pid::MultiNodesPid unique_pid_wheel_left_front_1(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel_1);
hw_pid::MultiNodesPid unique_pid_wheel_left_rear_1(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel_1);
hw_pid::MultiNodesPid unique_pid_wheel_right_front_1(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel_1);
hw_pid::MultiNodesPid unique_pid_wheel_right_rear_1(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel_1);
hw_pid::MultiNodesPid unique_pid_follow_omega_1(kPidTypeCascade, kOutLimitFollowOmega, kPidParamsFollowOmega_1);

hw_pid::MultiNodesPid unique_pid_wheel_left_front_2(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel_2);
hw_pid::MultiNodesPid unique_pid_wheel_left_rear_2(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel_2);
hw_pid::MultiNodesPid unique_pid_wheel_right_front_2(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel_2);
hw_pid::MultiNodesPid unique_pid_wheel_right_rear_2(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel_2);
hw_pid::MultiNodesPid unique_pid_follow_omega_2(kPidTypeCascade, kOutLimitFollowOmega, kPidParamsFollowOmega_2);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_pid::MultiNodesPid *CreatePidMotorWheelLeftFront() { return &unique_pid_wheel_left_front_1; };
hw_pid::MultiNodesPid *CreatePidMotorWheelLeftRear() { return &unique_pid_wheel_left_rear_1; };
hw_pid::MultiNodesPid *CreatePidMotorWheelRightFront() { return &unique_pid_wheel_right_front_1; };
hw_pid::MultiNodesPid *CreatePidMotorWheelRightRear() { return &unique_pid_wheel_right_rear_1; };
hw_pid::MultiNodesPid *CreatePidFollowOmega() { return &unique_pid_follow_omega_1; };

// hw_pid::MultiNodesPid* CreatePidMotorWheelLeftFront()
// {
//     if (car_version == 0)
// {
//     return &unique_pid_wheel_left_front_1;
// }
// else if(car_version == 1)
// {
//     return &unique_pid_wheel_left_front_2; };
// }
// hw_pid::MultiNodesPid* CreatePidMotorWheelLeftRear()
// {
//     if (car_version == 0)
// {
//     return &unique_pid_wheel_left_rear_1;
// }
// else if(car_version == 1)
// {
//     return &unique_pid_wheel_left_rear_2; };

// };
// hw_pid::MultiNodesPid* CreatePidMotorWheelRightFront()
// {
//     if (car_version == 0)
// {
//     return &unique_pid_wheel_right_front_1;
// }
// else if(car_version == 1)
// {
//     return &unique_pid_wheel_right_front_2; };
// };
// hw_pid::MultiNodesPid* CreatePidMotorWheelRightRear()
// {
//     if (car_version == 0)
// {
//     return &unique_pid_wheel_right_rear_1;
// }
// else if(car_version == 1)
// {
//     return &unique_pid_wheel_right_rear_2; };
// };
// hw_pid::MultiNodesPid* CreatePidFollowOmega()
// {
//     if (car_version == 0)
// {
//     return &unique_pid_follow_omega_1;
// }
// else if(car_version == 1)
// {
//     return &unique_pid_follow_omega_2; };

// };
/* Private function definitions ----------------------------------------------*/
