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
#include "main_task.hpp"
#include "motor.hpp"

/* Private constants ---------------------------------------------------------*/
const float kMaxPidOutYaw = 6.9f;
const float kMaxPidOutPitch = 6.9f;
const float kMaxPidOutFric = 16384.0f;
const float kMaxPidOutFeed = 15.0f;  ///< 3508电流控制的最大输出
const float kMaxPidOutSameSpd = 100.0f;
const hw_pid::OutLimit kOutLimitYaw = hw_pid::OutLimit(true, -kMaxPidOutYaw, kMaxPidOutYaw);
const hw_pid::OutLimit kOutLimitPitch = hw_pid::OutLimit(true, -kMaxPidOutPitch, kMaxPidOutPitch);
const hw_pid::OutLimit kOutLimitFeed = hw_pid::OutLimit(true, -kMaxPidOutFeed, kMaxPidOutFeed);
const hw_pid::OutLimit kOutLimitFric = hw_pid::OutLimit(true, -kMaxPidOutFric, kMaxPidOutFric);
const hw_pid::OutLimit kOutLimitSameSpd = hw_pid::OutLimit(true, -kMaxPidOutSameSpd, kMaxPidOutSameSpd);

const hw_pid::MultiNodesPid::ParamsList kPidParamsYaw_1 = {
    {
      // 角度环
      .auto_reset = false,  ///< 是否自动清零
      .kp = 22.5f,//22.5f
      .ki = 0.00f,  // 0.45
      .kd = 200.0f,
      .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1f, 0.1f, 0.1f),
      .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
      .period_sub = hw_pid::PeriodSub(true, 2 * PI),
      .inte_anti_windup = hw_pid::InteAntiWindup(true, -2.0f, 2.0f),
      .inte_changing_rate = hw_pid::InteChangingRate(true, 0.2f, 0.35f),
      // .diff_filter = hw_pid::DiffFilter(true, -3.0f, 3.0f, 0.5f),
      .out_limit = hw_pid::OutLimit(true, -20.0f, 20.0f),  ///< 输出限制 @see OutLimit
     },
    {
      // 速度环
      .auto_reset = false,  ///< 是否自动清零
      .kp = 1.860f,         // 1.86f
      .ki = 0.000f,
      .kd = 0.0f,
      .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1f, 0.1f, 0.1f),
      .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
      .inte_anti_windup = hw_pid::InteAntiWindup(true, -1.5f, 1.5f),
      .inte_changing_rate = hw_pid::InteChangingRate(true, 0.1f, 0.5f),
      .diff_filter = hw_pid::DiffFilter(true, -3.0f, 3.0f, 0.5f),
      .out_limit = hw_pid::OutLimit(true, -7.0f, 7.0f),  ///< 输出限制 @see OutLimit
     },
};
const hw_pid::MultiNodesPid::ParamsList kPidParamsYaw_2 = {
  {
    // 角度环
    .auto_reset = false,  ///< 是否自动清零
    .kp = 22.5f,//22.5f
    .ki = 0.00f,  // 0.45
    .kd = 200.0f,
    .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1f, 0.1f, 0.1f),
    .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
    .period_sub = hw_pid::PeriodSub(true, 2 * PI),
    .inte_anti_windup = hw_pid::InteAntiWindup(true, -2.0f, 2.0f),
    .inte_changing_rate = hw_pid::InteChangingRate(true, 0.2f, 0.35f),
    // .diff_filter = hw_pid::DiffFilter(true, -3.0f, 3.0f, 0.5f),
    .out_limit = hw_pid::OutLimit(true, -20.0f, 20.0f),  ///< 输出限制 @see OutLimit
   },
  {
    // 速度环
    .auto_reset = false,  ///< 是否自动清零
    .kp = 1.860f,         // 1.86f
    .ki = 0.000f,
    .kd = 0.0f,
    .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1f, 0.1f, 0.1f),
    .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
    .inte_anti_windup = hw_pid::InteAntiWindup(true, -1.5f, 1.5f),
    .inte_changing_rate = hw_pid::InteChangingRate(true, 0.1f, 0.5f),
    .diff_filter = hw_pid::DiffFilter(true, -3.0f, 3.0f, 0.5f),
    .out_limit = hw_pid::OutLimit(true, -7.0f, 7.0f),  ///< 输出限制 @see OutLimit
   },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsPitch_1 = {
    {
      // 角度环
      .auto_reset = true,  ///< 是否自动清零
      .kp = 23.8f,    //23.8
      .ki = 0.00f,          
      .kd = 50.00f,//200.0
      .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
      .period_sub = hw_pid::PeriodSub(true, 2 * PI),
      .inte_anti_windup = hw_pid::InteAntiWindup(true, -2.0f, 2.0f),
      .inte_changing_rate = hw_pid::InteChangingRate(true, 0.08f, 0.10f),
      .diff_filter = hw_pid::DiffFilter(true, -3.0f, 3.0f, 0.5f),
      .out_limit = hw_pid::OutLimit(true, -20.0f, 20.0f),  ///< 输出限制 @see OutLimit
     },
    {
      // 速度环
      .auto_reset = true,  ///< 是否自动清零
      .kp = 1.15f,   //1.25
      .ki = 0.0f,
      .kd = 0.0f,
      .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
      .inte_anti_windup = hw_pid::InteAntiWindup(true, -2.0f, 2.0f),
      .inte_changing_rate = hw_pid::InteChangingRate(true, 0.05f, 0.1f),
      .diff_filter = hw_pid::DiffFilter(true, -3.0f, 3.0f, 0.5f),
      // .diff_previous = hw_pid::DiffPrevious(true, 80.0f),
      .out_limit = hw_pid::OutLimit(true, -7.0f, 7.0f),  ///< 输出限制 @see OutLimit
     },
};
const hw_pid::MultiNodesPid::ParamsList kPidParamsPitch_2 = {
  {
    // 角度环
    .auto_reset = true,  ///< 是否自动清零
    .kp = 23.8f,    //23.8
    .ki = 0.00f,          
    .kd = 50.00f,//200.0
    .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
    .period_sub = hw_pid::PeriodSub(true, 2 * PI),
    .inte_anti_windup = hw_pid::InteAntiWindup(true, -2.0f, 2.0f),
    .inte_changing_rate = hw_pid::InteChangingRate(true, 0.08f, 0.10f),
    .diff_filter = hw_pid::DiffFilter(true, -3.0f, 3.0f, 0.5f),
    .out_limit = hw_pid::OutLimit(true, -20.0f, 20.0f),  ///< 输出限制 @see OutLimit
   },
  {
    // 速度环
    .auto_reset = true,  ///< 是否自动清零
    .kp = 1.15f,   //1.25
    .ki = 0.0f,
    .kd = 0.0f,
    .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
    .inte_anti_windup = hw_pid::InteAntiWindup(true, -2.0f, 2.0f),
    .inte_changing_rate = hw_pid::InteChangingRate(true, 0.05f, 0.1f),
    .diff_filter = hw_pid::DiffFilter(true, -3.0f, 3.0f, 0.5f),
    // .diff_previous = hw_pid::DiffPrevious(true, 80.0f),
    .out_limit = hw_pid::OutLimit(true, -7.0f, 7.0f),  ///< 输出限制 @see OutLimit
   },
};


const hw_pid::MultiNodesPid::ParamsList kPidParamsFric_1 = {
    {
      .auto_reset = true,  ///< 是否自动清零
      .kp = 201.50f,
      .ki = 0.0f,//0.1
      .kd = 0.0f,
      .inte_anti_windup = hw_pid::InteAntiWindup(true, -5000.0f, 5000.0f),
      .out_limit = hw_pid::OutLimit(true, -16384.0f, 16384.0f),  ///< 输出限制 @see OutLimit
     },
};
const hw_pid::MultiNodesPid::ParamsList kPidParamsFric_2 = {
  {
    .auto_reset = true,  ///< 是否自动清零
    .kp = 201.50f,
    .ki = 0.0f,//0.1
    .kd = 0.0f,
    .inte_anti_windup = hw_pid::InteAntiWindup(true, -5000.0f, 5000.0f),
    .out_limit = hw_pid::OutLimit(true, -16384.0f, 16384.0f),  ///< 输出限制 @see OutLimit
   },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsSameSpd_1 = {
    {
      .auto_reset = true,  ///< 是否自动清零
      .kp = 2.40f,
      .ki = 0.001f,
      .kd = 0.0f,
      .inte_anti_windup = hw_pid::InteAntiWindup(true, -5000.0f, 5000.0f),
      .out_limit = hw_pid::OutLimit(true, -100.0f, 100.0f),  ///< 输出限制 @see OutLimit
     },
};
const hw_pid::MultiNodesPid::ParamsList kPidParamsSameSpd_2 = {
  {
    .auto_reset = true,  ///< 是否自动清零
    .kp = 2.40f,
    .ki = 0.001f,
    .kd = 0.0f,
    .inte_anti_windup = hw_pid::InteAntiWindup(true, -5000.0f, 5000.0f),
    .out_limit = hw_pid::OutLimit(true, -100.0f, 100.0f),  ///< 输出限制 @see OutLimit
   },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsFeed_1 = {
    {
          // 角度环
          .auto_reset = true,  ///< 是否自动清零
          .kp = 53.3f,
          .ki = 0.0f,
          .kd = 0.0f,
          .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1f, 0.1f, 0.1f),
          .period_sub = hw_pid::PeriodSub(true, 2 * PI),
          .out_limit = hw_pid::OutLimit(true, -20.0f, 20.0f),  ///< 输出限制 @see OutLimit
                                                            //.inte_anti_windup=pid::InteAntiWindup(-0.00,0.00)
      },
      {
          // 速度环
          .auto_reset = true,  ///< 是否自动清零
          .kp = 5.575f,
          .ki = 0.0f,
          .kd = 0.0f,
          .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1f, 0.1f, 0.1f),
          .out_limit = hw_pid::OutLimit(true, -10000.0f, 10000.0f),  ///< 输出限制 @see OutLimit
     },
};
const hw_pid::MultiNodesPid::ParamsList kPidParamsFeed_2 = {
  {
        // 角度环
        .auto_reset = true,  ///< 是否自动清零
        .kp = 53.3f,
        .ki = 0.0f,
        .kd = 0.0f,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1f, 0.1f, 0.1f),
        .period_sub = hw_pid::PeriodSub(true, 2 * PI),
        .out_limit = hw_pid::OutLimit(true, -20.0f, 20.0f),  ///< 输出限制 @see OutLimit
                                                          //.inte_anti_windup=pid::InteAntiWindup(-0.00,0.00)
    },
    {
        // 速度环
        .auto_reset = true,  ///< 是否自动清零
        .kp = 5.575f,
        .ki = 0.0f,
        .kd = 0.0f,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1f, 0.1f, 0.1f),
        .out_limit = hw_pid::OutLimit(true, -10000.0f, 10000.0f),  ///< 输出限制 @see OutLimit
   },
};

const hw_pid::MultiNodesPid::Type kPidTypeCascade = hw_pid::MultiNodesPid::Type::kCascade;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_pid::MultiNodesPid unique_pid_yaw_1(kPidTypeCascade, kOutLimitYaw, kPidParamsYaw_1);
hw_pid::MultiNodesPid unique_pid_pitch_1(kPidTypeCascade, kOutLimitPitch, kPidParamsPitch_1);
hw_pid::MultiNodesPid unique_pid_fric_left_1(kPidTypeCascade, kOutLimitFric, kPidParamsFric_1);
hw_pid::MultiNodesPid unique_pid_fric_right_1(kPidTypeCascade, kOutLimitFric, kPidParamsFric_1);
hw_pid::MultiNodesPid unique_pid_feed_1(kPidTypeCascade, kOutLimitFeed, kPidParamsFeed_1);
hw_pid::MultiNodesPid unique_pid_samespd_1(kPidTypeCascade,kOutLimitSameSpd,kPidParamsSameSpd_1);

hw_pid::MultiNodesPid unique_pid_yaw_2(kPidTypeCascade, kOutLimitYaw, kPidParamsYaw_2);
hw_pid::MultiNodesPid unique_pid_pitch_2(kPidTypeCascade, kOutLimitPitch, kPidParamsPitch_2);
hw_pid::MultiNodesPid unique_pid_fric_left_2(kPidTypeCascade, kOutLimitFric, kPidParamsFric_2);
hw_pid::MultiNodesPid unique_pid_fric_right_2(kPidTypeCascade, kOutLimitFric, kPidParamsFric_2);
hw_pid::MultiNodesPid unique_pid_feed_2(kPidTypeCascade, kOutLimitFeed, kPidParamsFeed_2);
hw_pid::MultiNodesPid unique_pid_samespd_2(kPidTypeCascade,kOutLimitSameSpd,kPidParamsSameSpd_2);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_pid::MultiNodesPid* CreatePidMotorYaw() { return &unique_pid_yaw_1; };
hw_pid::MultiNodesPid* CreatePidMotorPitch() { return &unique_pid_pitch_1; };
hw_pid::MultiNodesPid* CreatePidMotorFricLeft() { return &unique_pid_fric_left_1; };
hw_pid::MultiNodesPid* CreatePidMotorFricRight() { return &unique_pid_fric_right_1; };
hw_pid::MultiNodesPid* CreatePidMotorFeed() { return &unique_pid_feed_1; };
hw_pid::MultiNodesPid* CreatePidSamespd() { return &unique_pid_samespd_1;};

// hw_pid::MultiNodesPid* CreatePidMotorYaw() 
// { 
//     if (car_version == 0)
// {
//     return &unique_pid_yaw_1;
// }
// else if(car_version == 1)
// {
//     return &unique_pid_yaw_2; };
// };
// hw_pid::MultiNodesPid* CreatePidMotorPitch() 
// { 
//   if (car_version == 0)
//   {
//       return &unique_pid_pitch_2;
//   }
//   else if(car_version == 1)
//   {
//       return &unique_pid_pitch_2; };
// };
// hw_pid::MultiNodesPid* CreatePidMotorFricLeft() 
// { 
//   if (car_version == 0)
//   {
//       return &unique_pid_fric_left_2;
//   }
//   else if(car_version == 1)
//   {
//       return &unique_pid_fric_left_2; };
// };
// hw_pid::MultiNodesPid* CreatePidMotorFricRight() 
// { 
//   if (car_version == 0)
//   {
//       return &unique_pid_fric_right_1;
//   }
//   else if(car_version == 1)
//   {
//       return &unique_pid_fric_right_2; };
// };
// hw_pid::MultiNodesPid* CreatePidMotorFeed() 
// { 
//   if (car_version == 0)
//   {
//       return &unique_pid_feed_1;
//   }
//   else if(car_version == 1)
//   {
//       return &unique_pid_feed_2; };
// };
// hw_pid::MultiNodesPid* CreatePidSamespd() 
// { 
//   if (car_version == 0)
//   {
//     return &unique_pid_samespd_1;
//   }
//   else if(car_version == 1)
//   {
//     return &unique_pid_samespd_2;};
// };
/* Private function definitions ----------------------------------------------*/
