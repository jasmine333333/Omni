/** 
 *******************************************************************************
 * @file      : scope.cpp
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
#include "scope.hpp"

#include "base.hpp"
/* Private macro -------------------------------------------------------------*/
namespace robot
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

#pragma region 数据更新
void Scope::update()
{
  updateData();
  updatePwrState();
};
void Scope::updateData() { updateWorkTick(); };

void Scope::updatePwrState()
{
  // 目前倍镜舵机由C板PWM输出，所以死亡和复活模式都可以直接切到下一模式
  if (pwr_state_ == PwrState::Dead) {
    setPwrState(PwrState::Resurrection);
  } else if (pwr_state_ == PwrState::Resurrection) {
    setPwrState(PwrState::Working);
  } else if (pwr_state_ == PwrState::Working) {
    // 正常工作状态
  } else {
    // 其他状态
    setPwrState(PwrState::Dead);
  }
};
#pragma endregion

#pragma region 执行任务
void Scope::run()
{
  // 任务执行
  if (pwr_state_ == PwrState::Dead) {
    runOnDead();
  } else if (pwr_state_ == PwrState::Resurrection) {
    runOnResurrection();
  } else if (pwr_state_ == PwrState::Working) {
    runOnWorking();
  } else {
    runOnDead();
  }
};

void Scope::runOnDead()
{
  resetDataOnDead();

  for (size_t i = 0; i < 2; i++) {
    HW_ASSERT(servo_ptr_[i] != nullptr, "pointer to servo %d is nullptr", i);
    servo_ptr_[i]->disable();
  }
};

void Scope::runOnResurrection()
{
  resetDataOnResurrection();
  for (size_t i = 0; i < 2; i++) {
    HW_ASSERT(servo_ptr_[i] != nullptr, "pointer to servo %d is nullptr", i);
    servo_ptr_[i]->enable();
  }
};

void Scope::runOnWorking()
{
  if (working_mode_ == WorkingMode::Normal) {
    pitch_ang_ref_ = cfg_.pitch_normal_ang;
    scope_ang_ref_ = cfg_.not_using_scope_ang;
  } else if (working_mode_ == WorkingMode::Farshoot) {
    pitch_ang_recorded_ += norm_pitch_delta_ * cfg_.pitch_sensitivity;

    pitch_ang_recorded_ = hello_world::AngleNormDeg(pitch_ang_recorded_);
    pitch_ang_recorded_ = hello_world::Bound(pitch_ang_recorded_, cfg_.min_pitch_ang, cfg_.max_pitch_ang);

    pitch_ang_ref_ = pitch_ang_recorded_;

    scope_ang_ref_ = use_scope_flag_ ? cfg_.using_scope_ang : cfg_.not_using_scope_ang;
  }

  servo_ptr_[(uint8_t)ServoIdx::Pitch]->setAngle(pitch_ang_ref_);
  servo_ptr_[(uint8_t)ServoIdx::Scope]->setAngle(scope_ang_ref_);
};

void Scope::standby()
{
  pitch_ang_ref_ = cfg_.pitch_normal_ang;
  scope_ang_ref_ = cfg_.not_using_scope_ang;

  servo_ptr_[(uint8_t)ServoIdx::Pitch]->setAngle(pitch_ang_ref_);
  servo_ptr_[(uint8_t)ServoIdx::Scope]->setAngle(scope_ang_ref_);
};
#pragma endregion

#pragma region 数据重置
void Scope::reset()
{
  ctrl_angle_flag_ = false;    ///< 控制角度标志位
  switch_scope_flag_ = false;  ///< 开关倍镜标志位

  working_mode_ = WorkingMode::Normal;       ///< 当前工作模式
  last_working_mode_ = WorkingMode::Normal;  ///< 上一次工作模式

  norm_pitch_delta_ = 0.0f;  ///< 相对角度增量

  use_scope_flag_ = false;  ///< 是否使用倍镜

  last_switch_scope_tick_ = 0;  ///< 上一次开关倍镜的时间戳

  pitch_ang_ref_ = 0.0f;       ///< pitch 角度期望值，单位：Deg
  pitch_ang_recorded_ = 0.0f;  ///< 记录下的 pitch 角度，单位：Deg

  scope_ang_ref_ = 0.0f;
};

void Scope::resetDataOnDead()
{
  ctrl_angle_flag_ = false;    ///< 控制角度标志位
  switch_scope_flag_ = false;  ///< 开关倍镜标志位

  setWorkingMode(WorkingMode::Normal);

  norm_pitch_delta_ = 0.0f;  ///< 相对角度增量

  use_scope_flag_ = false;  ///< 是否使用倍镜

  pitch_ang_ref_ = cfg_.pitch_normal_ang;
  scope_ang_ref_ = cfg_.not_using_scope_ang;
};

void Scope::resetDataOnResurrection()
{
  ctrl_angle_flag_ = false;    ///< 控制角度标志位
  switch_scope_flag_ = false;  ///< 开关倍镜标志位

  setWorkingMode(WorkingMode::Normal);

  norm_pitch_delta_ = 0.0f;  ///< 相对角度增量

  use_scope_flag_ = false;  ///< 是否使用倍镜

  pitch_ang_ref_ = cfg_.pitch_normal_ang;
  scope_ang_ref_ = cfg_.not_using_scope_ang;
};
#pragma endregion

#pragma region 通信数据设置函数
#pragma endregion

#pragma region 注册函数
void Scope::registerServo(Servo *ptr, ServoIdx idx)
{
  HW_ASSERT(ptr != nullptr, "servo is nullptr", ptr);
  HW_ASSERT((0 <= (uint8_t)idx) && (idx < ServoIdx::Num), "idx out of range", idx);
  servo_ptr_[(uint8_t)idx] = ptr;
};
#pragma endregion

#pragma region 其他工具函数
#pragma endregion

/* Private function definitions ----------------------------------------------*/
}  // namespace robot
