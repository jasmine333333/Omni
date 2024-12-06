/** 
 *******************************************************************************
 * @file      : ins_motor.cpp
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

const hw_motor::OptionalParams kMotorParamsWheelLeftFront = {
    .input_type = hw_motor::InputType::kCurr,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirFwd,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
    /** 电机外置减速器的减速比（额外） */
    .ex_redu_rat = 14,
};
const hw_motor::OptionalParams kMotorParamsWheelLeftRear = {
    .input_type = hw_motor::InputType::kCurr,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirFwd,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
    /** 电机外置减速器的减速比（额外） */
    .ex_redu_rat = 14,
};
const hw_motor::OptionalParams kMotorParamsWheelRightRear = {
    .input_type = hw_motor::InputType::kCurr,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirRev,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
    /** 电机外置减速器的减速比（额外） */
    .ex_redu_rat = 14,
};
const hw_motor::OptionalParams kMotorParamsWheelRightFront = {
    .input_type = hw_motor::InputType::kCurr,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirRev,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
    /** 电机外置减速器的减速比（额外） */
    .ex_redu_rat = 14,
};

const hw_motor::OptionalParams kMotorParamsYaw = {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirFwd,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = false,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
    /** 电机外置减速器的减速比（额外） */
    // .ex_redu_rat = 14,
};

// TODO: 这里的 MotorId 需要按照实际情况修改
enum MotorID {
  kMotorIdWheelLeftFront = 1u,
  kMotorIdWheelLeftRear,
  kMotorIdWheelRightRear,
  kMotorIdWheelRightFront,
  kMotorIdYaw = 3u,
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_motor::M3508 unique_motor_wheel_left_front = hw_motor::M3508(kMotorIdWheelLeftFront, kMotorParamsWheelLeftFront);
hw_motor::M3508 unique_motor_wheel_left_rear = hw_motor::M3508(kMotorIdWheelLeftRear, kMotorParamsWheelLeftRear);
hw_motor::M3508 unique_motor_wheel_right_rear = hw_motor::M3508(kMotorIdWheelRightRear, kMotorParamsWheelRightRear);
hw_motor::M3508 unique_motor_wheel_right_front = hw_motor::M3508(kMotorIdWheelRightFront, kMotorParamsWheelRightFront);
hw_motor::DM_J4310 unique_motor_yaw = hw_motor::DM_J4310(kMotorIdYaw, kMotorParamsYaw);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

hw_motor::Motor* CreateMotorWheelLeftFront() { return &unique_motor_wheel_left_front; };
hw_motor::Motor* CreateMotorWheelLeftRear() { return &unique_motor_wheel_left_rear; };
hw_motor::Motor* CreateMotorWheelRightRear() { return &unique_motor_wheel_right_rear; };
hw_motor::Motor* CreateMotorWheelRightFront() { return &unique_motor_wheel_right_front; };
hw_motor::Motor* CreateMotorYaw() { return &unique_motor_yaw; };

/* Private function definitions ----------------------------------------------*/
