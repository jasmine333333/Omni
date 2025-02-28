/** 
 *******************************************************************************
 * @file      :gimbal.cpp
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
#include "gimbal.hpp"
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

void Gimbal::update()
{
  updateData();
  updatePwrState();
}

void Gimbal::updateData()
{
  updateWorkTick();
  updateMotorData();
  updateImuData();
  updateIsPwrOn();
};

void Gimbal::updatePwrState()
{
  // 无论任何状态，断电意味着要切到死亡状态
  if (!is_pwr_on_) {
    setPwrState(PwrState::Dead);
    return;
  }

  PwrState current_state = pwr_state_;
  PwrState next_state = current_state;
  if (current_state == PwrState::Dead) {
    // 死亡状态下，如果上电，则切到复活状态
    if (is_pwr_on_) {
      next_state = PwrState::Resurrection;
    }
  } else if (current_state == PwrState::Resurrection) {
    // 复活状态下，如果所有云台关节电机上电完毕，且云台板准备就绪，则切到工作状态
    // 都有云台板通讯告知
    // 1. 为什么要判断云台板准备就绪？
    // 云台板计算零飘完零飘后会告知底盘准备就绪，如果云台板还未准备好，云台关节就能够运动会导致云台板的姿态计算结果可能不准确
    // 2. 为什么要所有云台关节电机上电完毕，才认为云台模块准备好了？
    // 没啥原因，暂时先这么写着
    if (is_any_motor_pwron_) {
      next_state = PwrState::Working;
    }
  } else if (current_state == PwrState::Working) {
    // 工作状态下，保持当前状态
  } else {
    // 其他状态，认为是死亡状态
    next_state = PwrState::Dead;
  }
  setPwrState(next_state);
};

void Gimbal::updateMotorData()
{
  Motor *motor_ptr = nullptr;
  JointIdx motor_idxs[2] = {kJointYaw, kJointPitch};

  bool is_any_motor_pwron = false;
  bool is_all_motor_pwron = true;

  for (size_t i = 0; i < 2; i++) {
    JointIdx motor_idx = motor_idxs[i];
    motor_ptr = motor_ptr_[motor_idx];
    HW_ASSERT(motor_ptr != nullptr, "pointer %d to motor %d is nullptr", motor_ptr, motor_idx);
    if (motor_ptr->isOffline()) {
      motor_ang_fdb_[motor_idx] = 0.0f;
      motor_spd_fdb_[motor_idx] = 0.0f;

      is_all_motor_pwron = false;
    } else {
      is_any_motor_pwron = true;

      motor_ang_fdb_[motor_idx] = motor_ptr->angle();
      // motor_spd_fdb_[joint_idx] = motor_ptr->spd();
      // 达妙速度反馈噪声大，使用td滤波计算速度
      motor_spd_td_ptr_[motor_idx]->calc(&motor_ang_fdb_[motor_idx], &motor_spd_fdb_[motor_idx]);
    }
  }

  is_any_motor_pwron_ = is_any_motor_pwron;
  is_all_motor_pwron_ = is_all_motor_pwron;
};

void Gimbal::updateIsPwrOn() { is_pwr_on_ = is_any_motor_pwron_ || is_rfr_pwr_on_; };

void Gimbal::updateImuData()
{
  HW_ASSERT(imu_ptr_ != nullptr, "pointer %d to imu %d is nullptr", imu_ptr_);
  // TODO(ZSC): 之后需要检查此处的数据映射
  // IMU是右手系，但pitch轴直觉上应该得是左手系，即低头角度为负，抬头角度为正，(故在此处加负号)
  imu_ang_fdb_[kJointYaw] = imu_ptr_->getAngYaw();
  imu_ang_fdb_[kJointPitch] = -imu_ptr_->getAngPitch();

  imu_spd_fdb_[kJointYaw] = imu_ptr_->getGyroYaw();
  imu_spd_fdb_[kJointPitch] = -imu_ptr_->getGyroPitch();
};

#pragma endregion

#pragma region 任务执行
void Gimbal::run()
{
  if (pwr_state_ == PwrState::Dead) {
    runOnDead();
  } else if (pwr_state_ == PwrState::Resurrection) {
    runOnResurrection();
  } else if (pwr_state_ == PwrState::Working) {
    runOnWorking();
  } else {
    // 其他状态，认为是死亡状态
    runOnDead();
  }
}
void Gimbal::runOnDead()
{
  // HW_ASSERT(laser_ptr_ != nullptr, "pointer to laser is nullptr", laser_ptr_);
  // laser_ptr_->disable();
  resetDataOnDead();
  setCommData(false);
};

void Gimbal::runOnResurrection()
{
  // HW_ASSERT(laser_ptr_ != nullptr, "pointer to laser is nullptr", laser_ptr_);
  // laser_ptr_->disable();
  resetDataOnResurrection();
  setCommData(false);
};
void Gimbal::runOnWorking()
{
  // HW_ASSERT(laser_ptr_ != nullptr, "pointer to laser is nullptr", laser_ptr_);
  // if (gimbal_ctrl_mode == CtrlMode::Auto) {
  //   laser_ptr_->disable();
  // } else {
  //   laser_ptr_->enable();
  // }
  // laser_ptr_->enable();
  // }
  // laser_ptr_->disable();
  if (working_mode_ == WorkingMode::dead)
  {
    // laser_ptr_->disable();
    JointIdx joint_idxs[2] = {kJointYaw, kJointPitch};
    for (size_t i = 0; i < 2; i++) {
      JointIdx joint_idx = joint_idxs[i];
      pid_ptr_[joint_idx]->reset();
      motor_ptr_[joint_idx]->setInput(0);}
      
  }
  else{
  calcCtrlAngBased();
  adjustJointFdb();
  adjustLastJointAngRef();
  calcJointAngRef();
  calcJointTorRef();
  setCommData(true);
  }
  };

void Gimbal::standby()
{
  calcCtrlAngBased();
  adjustJointFdb();
  adjustLastJointAngRef();
  calcJointAngRef();

  setCommData(false);
};

void Gimbal::calcCtrlAngBased()
{
  last_ctrl_ang_based_[kJointYaw] = ctrl_ang_based_[kJointYaw];
  last_ctrl_ang_based_[kJointPitch] = ctrl_ang_based_[kJointPitch];

  if (working_mode_ == WorkingMode::Normal) {
    ctrl_ang_based_[kJointYaw] = CtrlAngBased::Imu;
    ctrl_ang_based_[kJointPitch] = CtrlAngBased::Motor;
  } else {
    ctrl_ang_based_[kJointYaw] = CtrlAngBased::Imu;
    ctrl_ang_based_[kJointPitch] = CtrlAngBased::Imu;
  }
};

void Gimbal::adjustJointFdb()
{
  JointIdx joint_idxs[kJointNum] = {kJointYaw, kJointPitch};
  for (size_t i = 0; i < kJointNum; i++) {
    JointIdx joint_idx = joint_idxs[i];
    if (ctrl_ang_based_[joint_idx] == CtrlAngBased::Motor) {
      joint_ang_fdb_[joint_idx] = motor_ang_fdb_[joint_idx];
    } else {
      joint_ang_fdb_[joint_idx] = imu_ang_fdb_[joint_idx];
    }
    joint_spd_fdb_[joint_idx] = imu_spd_fdb_[joint_idx];
  }
}

void Gimbal::adjustLastJointAngRef()
{
  // 判断工作模式是否发生变化
  JointIdx joint_idxs[2] = {kJointYaw, kJointPitch};
  for (size_t i = 0; i < 2; i++) {
    JointIdx joint_idx = joint_idxs[i];
    if (last_ctrl_ang_based_[joint_idx] == ctrl_ang_based_[joint_idx]) {
      last_joint_ang_ref_[joint_idx] = joint_ang_ref_[joint_idx];
      continue;
    }

    if (ctrl_ang_based_[joint_idx] == CtrlAngBased::Motor) {
      last_joint_ang_ref_[joint_idx] = motor_ang_fdb_[joint_idx];
    } else if (ctrl_ang_based_[joint_idx] == CtrlAngBased::Imu) {
      last_joint_ang_ref_[joint_idx] = imu_ang_fdb_[joint_idx];
    } else {
      HW_ASSERT(false, "unknown ctrl_ang_based_[joint_idx] %d", ctrl_ang_based_[joint_idx]);
    }
  }
}
bool debug_enemydetected ;
float debug_pitch = 0.0f;
float debug_yaw = 0.0f;
void Gimbal::calcJointAngRef()
{
  debug_enemydetected = vis_data_.is_target_detected;
  // 如果控制模式是自动，且视觉模块没有离线、视觉模块检测到有效目标，且视觉反馈角度与当前角度相差不大
  Cmd tmp_ang_ref = {0.0f};
  if (ctrl_mode_ == CtrlMode::Auto && fabsf(joint_ang_fdb_[kJointYaw] - vis_data_.cmd.yaw) < 0.3927f &&
      fabsf(joint_ang_fdb_[kJointPitch] - vis_data_.cmd.pitch) < 0.30543f
    ) {
    tmp_ang_ref = vis_data_.cmd;
  } else  {
    // 否则，根据上一控制周期的关节角度参考值计算当前控制周期的关节角度参考值
    float sensitivity_yaw = cfg_.sensitivity_yaw;      // yaw角度灵敏度，单位 rad/ms
    float sensitivity_pitch = cfg_.sensitivity_pitch;  // pitch角度灵敏度，单位 rad/ms
    // 如果翻转头部朝向标志位为真，且距离上一次翻转头部朝向的时间超过200ms
    if (rev_head_flag_ && work_tick_ - last_rev_head_tick_ > 200) {
      tmp_ang_ref.yaw = last_joint_ang_ref_[kJointYaw] + PI;
      last_rev_head_tick_ = work_tick_;
    }

    tmp_ang_ref.yaw = last_joint_ang_ref_[kJointYaw] + norm_cmd_delta_.yaw * sensitivity_yaw;
    tmp_ang_ref.pitch = last_joint_ang_ref_[kJointPitch];
    // 当 PItch 轴角度到达限位危险值时，不再进行增减，防止在限位附近震荡
    // 此种方法略有改善，震荡问题依然存在，原因可能为IMU反馈值没有及时更新或者更新速度慢导致的
    // 此时的现象为，电机角度反馈值震荡，导致ref值的动态限制值震荡，导致控制角度震荡
    // 但此时 IMU 反馈值的震荡较小
    //  Pitch轴限位
    bool is_pitch_ang_too_large = motor_ang_fdb_[kJointPitch] > cfg_.max_pitch_ang - 0.05f;
    bool is_pitch_ang_too_small = motor_ang_fdb_[kJointPitch] < cfg_.min_pitch_ang + 0.05f;
    if (is_pitch_ang_too_large && norm_cmd_delta_.pitch > 0) {
    } else if (is_pitch_ang_too_small && norm_cmd_delta_.pitch < 0) {
    } else {
      tmp_ang_ref.pitch += norm_cmd_delta_.pitch * sensitivity_pitch;
    }
  }

  // 角度归一化到[-pi, pi)
  tmp_ang_ref.yaw = hello_world::AngleNormRad(tmp_ang_ref.yaw);
  tmp_ang_ref.pitch = hello_world::AngleNormRad(tmp_ang_ref.pitch);

  //  Pitch轴限位
  if (ctrl_ang_based_[kJointPitch] == CtrlAngBased::Motor) {
    tmp_ang_ref.pitch = hello_world::Bound(tmp_ang_ref.pitch, cfg_.min_pitch_ang, cfg_.max_pitch_ang);
  } else if (ctrl_ang_based_[kJointPitch] == CtrlAngBased::Imu) {
    // {fdb}_{imu} - {lim}_{imu} = {fdb}_{motor} - {lim}_{motor}
    // {lim}_{imu} = {lim}_{motor} + {fdb}_{imu} - {fdb}_{motor}
    float motor_imu_delta = imu_ang_fdb_[kJointPitch] - motor_ang_fdb_[kJointPitch];
    tmp_ang_ref.pitch = hello_world::Bound(tmp_ang_ref.pitch, cfg_.min_pitch_ang + motor_imu_delta, cfg_.max_pitch_ang + motor_imu_delta);
  } else {
    HW_ASSERT(false, "unknown ctrl_ang_based_[kJointPitch] %d", ctrl_ang_based_[kJointPitch]);
  }

  joint_ang_ref_[kJointYaw] = tmp_ang_ref.yaw;
  joint_ang_ref_[kJointPitch] = tmp_ang_ref.pitch;
  // joint_ang_ref_[kJointPitch] = debug_pitch;
  // joint_ang_ref_[kJointYaw] = debug_yaw;
};

void Gimbal::calcJointTorRef()
{
  JointIdx joint_idxs[kJointNum] = {kJointYaw, kJointPitch};
  float pitch_ffd = calcJointFfdResistance(kJointPitch) +
                    cfg_.max_pitch_torq * arm_cos_f32(joint_ang_fdb_[kJointPitch] + cfg_.pitch_center_offset);
  float yaw_ffd = calcJointFfdResistance(kJointYaw);
  float *ffd_list[2] = {&yaw_ffd, &pitch_ffd};
  for (uint8_t i = 0; i < kJointNum; i++) {
    JointIdx joint_idx = joint_idxs[i];
    float ref[2] = {joint_ang_ref_[joint_idx], 0.0f};
    float fdb[2] = {joint_ang_fdb_[joint_idx], imu_spd_fdb_[joint_idx]};
    float *ffd = ffd_list[i];
    Pid *pid_ptr = pid_ptr_[joint_idx];
    HW_ASSERT(pid_ptr != nullptr, "pointer to PID %d is nullptr", joint_idx);
    pid_ptr->calc(ref, fdb, ffd, &joint_tor_ref_[joint_idx]);
  }
  
}
float Gimbal::calcJointFfdResistance(JointIdx idx)
{
  float err = joint_ang_ref_[idx] - joint_ang_fdb_[idx];
  if (err > cfg_.allowed_ang_err[idx]) {
    return cfg_.resist_ffd_torq[idx];
  } else if (err < -cfg_.allowed_ang_err[idx]) {
    return -cfg_.resist_ffd_torq[idx];
  } else {
    return err * cfg_.resist_ffd_torq[idx] / cfg_.allowed_ang_err[idx];
  }
}

#pragma endregion

#pragma region 数据重置

/** 
 * @brief 重置除指针之外的所有数据，使状态机回到初始状态
 */
void Gimbal::reset()
{
  pwr_state_ = PwrState::Dead;  ///< 工作状态

  // 在 runOnWorking 函数中更新的数据
  ctrl_mode_ = CtrlMode::Manual;        ///< 控制模式
  working_mode_ = WorkingMode::Normal;  ///< 工作模式

  memset(ctrl_ang_based_, (int)CtrlAngBased::Imu, sizeof(ctrl_ang_based_));

  memset(joint_ang_ref_, 0, sizeof(joint_ang_ref_));            ///< 控制指令，基于关节空间
  memset(joint_ang_fdb_, 0, sizeof(joint_ang_fdb_));            ///< 云台关节角度反馈值
  memset(joint_spd_fdb_, 0, sizeof(joint_spd_fdb_));            ///< 云台关节速度反馈值
  memset(last_joint_ang_ref_, 0, sizeof(last_joint_ang_ref_));  ///< 上一次控制指令，基于关节空间
  memset(joint_tor_ref_, 0, sizeof(joint_tor_ref_));            ///< 控制指令，基于关节力矩

  // 从电机中拿的数据
  is_any_motor_pwron_ = false;  ///< 任意电机是否处于就绪状态
  is_all_motor_pwron_ = false;  ///< 所有电机是否都处于就绪状态

  memset(motor_ang_fdb_, 0, sizeof(motor_ang_fdb_));  ///< 云台关节角度反馈值【电机】
  memset(motor_spd_fdb_, 0, sizeof(motor_spd_fdb_));  ///< 云台关节速度反馈值【电机】

  // 从IMU中拿的数据
  memset(imu_ang_fdb_, 0, sizeof(imu_ang_fdb_));  ///< 云台关节角度反馈值【IMU】
  memset(imu_spd_fdb_, 0, sizeof(imu_spd_fdb_));  ///< 云台关节速度反馈值【IMU】

  resetPids();
};

void Gimbal::resetDataOnDead()
{
  // 在 runOnWorking 函数中更新的数据
  ctrl_mode_ = CtrlMode::Manual;        ///< 控制模式
  working_mode_ = WorkingMode::Normal;  ///< 工作模式

  memset(ctrl_ang_based_, (int)CtrlAngBased::Imu, sizeof(ctrl_ang_based_));

  memset(joint_ang_ref_, 0, sizeof(joint_ang_ref_));            ///< 控制指令，基于关节空间
  memset(joint_ang_fdb_, 0, sizeof(joint_ang_fdb_));            ///< 云台关节角度反馈值
  memset(joint_spd_fdb_, 0, sizeof(joint_spd_fdb_));            ///< 云台关节速度反馈值
  memset(last_joint_ang_ref_, 0, sizeof(last_joint_ang_ref_));  ///< 上一次控制指令，基于关节空间
  memset(joint_tor_ref_, 0, sizeof(joint_tor_ref_));            ///< 控制指令，基于关节力矩

  resetPids();
}

void Gimbal::resetDataOnResurrection()
{
  // 在 runOnWorking 函数中更新的数据
  ctrl_mode_ = CtrlMode::Manual;        ///< 控制模式
  working_mode_ = WorkingMode::Normal;  ///< 工作模式

  memset(ctrl_ang_based_, (int)CtrlAngBased::Imu, sizeof(ctrl_ang_based_));

  memset(joint_ang_ref_, 0, sizeof(joint_ang_ref_));            ///< 控制指令，基于关节空间
  memset(joint_ang_fdb_, 0, sizeof(joint_ang_fdb_));            ///< 云台关节角度反馈值
  memset(joint_spd_fdb_, 0, sizeof(joint_spd_fdb_));            ///< 云台关节速度反馈值
  memset(last_joint_ang_ref_, 0, sizeof(last_joint_ang_ref_));  ///< 上一次控制指令，基于关节空间
  memset(joint_tor_ref_, 0, sizeof(joint_tor_ref_));            ///< 控制指令，基于关节力矩

  resetPids();
}

void Gimbal::resetPids()
{
  for (size_t i = 0; i < kJointNum; i++) {
    HW_ASSERT(pid_ptr_[i] != nullptr, "pointer to PID %d is nullptr", i);
    pid_ptr_[i]->reset();
  }
};
#pragma endregion

#pragma region 通信数据设置

void Gimbal::setCommDataMotors(bool working_flag)
{
  // 机器人工作时
  // 电机根据期望电流输入发送数据
  JointIdx joint_idxs[2] = {kJointYaw, kJointPitch};
  for (size_t i = 0; i < 2; i++) {
    JointIdx joint_idx = joint_idxs[i];
    HW_ASSERT(motor_ptr_[joint_idx] != nullptr, "pointer to motor %d is nullptr", joint_idx);
    if (working_flag && (!motor_ptr_[joint_idx]->isOffline())) {
      motor_ptr_[joint_idx]->set_input_type(hello_world::motor::InputType::kTorq);
      motor_ptr_[joint_idx]->setInput(joint_tor_ref_[joint_idx]);
    } else {
      pid_ptr_[joint_idx]->reset();
      motor_ptr_[joint_idx]->set_input_type(hello_world::motor::InputType::kCmd);
      motor_ptr_[joint_idx]->setInput(hello_world::motor::DaMiao::Cmd::kCmdDisable);
    }
  }
};
#pragma endregion

#pragma region 注册函数

void Gimbal::registerMotor(Motor *ptr, JointIdx idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kJointNum, "index of motor out of range", idx);
  motor_ptr_[idx] = ptr;
};
void Gimbal::registerPid(Pid *ptr, JointIdx idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kJointNum, "index of PID out of range", idx);
  pid_ptr_[idx] = ptr;
};
void Gimbal::registerImu(Imu *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to imu is nullptr", ptr);
  imu_ptr_ = ptr;
}
void Gimbal::registerTd(Td *ptr, size_t idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to Td is nullptr", ptr);
  HW_ASSERT(idx >= 0 && idx < kJointNum, "index of Td out of range", idx);
  motor_spd_td_ptr_[idx] = ptr;
}
// void Gimbal::registerLaser(Laser *ptr)
// {
//   HW_ASSERT(ptr != nullptr, "pointer to laser is nullptr", ptr);
//   laser_ptr_ = ptr;
// }

#pragma endregion

#pragma region 其他工具函数

#pragma endregion
/* Private function definitions ----------------------------------------------*/

}  // namespace robot