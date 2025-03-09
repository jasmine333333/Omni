/** 
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
/* Private constants ---------------------------------------------------------*/

const hw_motor::OptionalParams kMotorParamsYaw = {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirFwd,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = false,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0.5f,
    /** 电机外置减速器的减速比（额外） */
    // .ex_redu_rat = 14,
};

const hw_motor::OptionalParams kMotorParamsPitch = {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirRev,  // 设置pitch轴电机低头角度低，抬头角度高
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = false,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = -0.0086f - 0.0106f+0.038f,
};

const hw_motor::OptionalParams kMotorParamsFricLeft = {
    .input_type = hw_motor::InputType::kRaw,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirFwd,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
};

const hw_motor::OptionalParams kMotorParamsFricRight = {
    .input_type = hw_motor::InputType::kRaw,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirRev,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
};

const hw_motor::OptionalParams kMotorParamsFeed = {
    // feed轮是2006电机
    .input_type = hw_motor::InputType::kCurr,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirRev,  //TODO(ZSC) 该方向适配 2024 分区赛英雄
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
    /** 电机外置减速器的减速比（额外） */
    .ex_redu_rat = 51,
};

enum MotorID {
  kMotorIdFricRight = 1u,   ///< 面向枪口时的右侧摩擦轮  
  kMotorIdFricLeft = 2u,    ///< 面向枪口时的左侧摩擦轮
  kMotorIdYaw = 2u,
  kMotorIdPitch = 1u,
  kMotorIdFeed = 3u,
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_motor::DM_J4310 unique_motor_yaw = hw_motor::DM_J4310(kMotorIdYaw, kMotorParamsYaw);
hw_motor::DM_J4310 unique_motor_pitch = hw_motor::DM_J4310(kMotorIdPitch, kMotorParamsPitch);
hw_motor::M3508 unique_motor_fric_left = hw_motor::M3508(kMotorIdFricLeft, kMotorParamsFricLeft);
hw_motor::M3508 unique_motor_fric_right = hw_motor::M3508(kMotorIdFricRight, kMotorParamsFricRight);
hw_motor::M2006 unique_motor_feed = hw_motor::M2006(kMotorIdFeed, kMotorParamsFeed);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

hw_motor::Motor* CreateMotorYaw() { return &unique_motor_yaw; };
hw_motor::Motor* CreateMotorPitch() { return &unique_motor_pitch; };
hw_motor::Motor* CreateMotorFricLeft() { return &unique_motor_fric_left; };
hw_motor::Motor* CreateMotorFricRight() { return &unique_motor_fric_right; };
hw_motor::Motor* CreateMotorFeed() { return &unique_motor_feed; };
/* Private function definitions ----------------------------------------------*/
