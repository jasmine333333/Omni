/** 
 *******************************************************************************
 * @file      :chassis.cpp
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
#include "chassis.hpp"
/* Private macro -------------------------------------------------------------*/
// DEBUG: 
float wheel_speed_fdb_debug = 0;
float wheel_speed_ref_debug = 0;

// TODO: norm_cmd_和cmd_好像没有关联起来，非常奇怪
// Robot的runOnWorking里产生各个模块的norm_cmd_
namespace robot
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

#pragma region 数据更新
void Chassis::update()
{
  updateData();
  updatePwrState();
};

void Chassis::updateData()
{
  updateWorkTick();
  updateGimbalBoard();
  updateMotor();
  updateCap();
  updateIsPowerOn();
};

/**
 * 更新电源状态
 * 
 * 无论处于任何状态，只要掉电就切换到死亡状态。
 * 死亡状态下，如果上电，则切到复活状态；
 * 复活状态下，如果有轮电机上电完毕（底盘已经准备完毕），且云台板 IMU 准备就绪，则切到工作状态；
 * 工作状态下，保持当前状态；
 * 其他状态，认为是死亡状态。
 */
void Chassis::updatePwrState()
{
  // 无论任何状态，断电意味着要切到死亡状态
  if (!is_power_on_) {
    setPwrState(PwrState::Dead);
    return;
  }

  PwrState current_state = pwr_state_;
  PwrState next_state = current_state;
  if (current_state == PwrState::Dead) {
    // 死亡状态下，如果上电，则切到复活状态
      next_state = PwrState::Resurrection;
      resurrection_time_ = 0;
  } else if (current_state == PwrState::Resurrection) {
    // 复活状态下，如果有轮电机上电完毕（底盘已经准备完毕），且云台板准备就绪，则切到工作状态
    // 1. 为什么要判断云台板准备就绪？
    // 云台板计算零飘完零飘后会告知底盘准备就绪，如果云台板还未准备好，底盘就能够运动会导致云台板的姿态计算结果可能不准确
    // 2. 为什么只要有轮电机上电完毕，就认为底盘就准备好了？
    // 因为底盘逆解不需要所有轮电机都在线也可以运行
    // 在后续正常工作状态下，会对轮电机的状态进行检测
    // 如果有轮电机掉电，会进行专门的处理，比如 pid 清空、can 发无效数据等
    // 3. 所有控制模式都需要 yaw 轴电机的角度，为什么不判断 yaw 轴电机的状态？
    // 因为 yaw 轴电机的状态不影响底盘的运动解算
    // 当 yaw 轴电机离线时，底盘会按照底盘坐标系进行解算（yaw 电机离线后数据清空，默认返回 0，能跑但是疯了，但总比不能跑强）
    // 当 yaw 轴电机上电时，底盘会按照图传坐标系进行解算

    if (is_gimbal_imu_ready_ && is_any_wheel_online_) {
      resurrection_time_++;
    } else {
      resurrection_time_ = 0;
    }
    if (resurrection_time_ > 2000)
    {
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

void Chassis::updateGimbalBoard()
{
  // 当云台板通讯丢失时，认为无云台板
  // 此时，直接认为云台板 IMU 准备就绪，使得可以在无云台板下工作
  HW_ASSERT(gc_comm_ptr_ != nullptr, "pointer to GimbalChassisComm is nullptr", gc_comm_ptr_);
  if (gc_comm_ptr_->isOffline()) {
    is_gimbal_imu_ready_ = true;
  } else {
    is_gimbal_imu_ready_ = gc_comm_ptr_->main_board_data().gp.is_gimbal_imu_ready;
  }
};

void Chassis::updateMotor()
{
  static uint32_t offline_tick[4] = {0};
  Motor *motor_ptr = nullptr;
  WheelMotorIdx wmis[4] = {
      kWheelMotorIdxLeftFront,
      kWheelMotorIdxLeftRear,
      kWheelMotorIdxRightRear,
      kWheelMotorIdxRightFront,
  };
  bool is_all_wheel_online = true;
  bool is_any_wheel_online = false;
  for (size_t i = 0; i < 4; i++) {
    WheelMotorIdx wmi = wmis[i];
    motor_ptr = wheel_motor_ptr_[wmi];
    HW_ASSERT(motor_ptr != nullptr, "pointer to motor %d is nullptr", wmi);
    if (motor_ptr->isOffline()) {
      offline_tick[i]++;
      wheel_speed_fdb_[wmi] = 0.0;
      wheel_current_fdb_[wmi] = 0.0;
      is_all_wheel_online = false;
    } else {
      is_any_wheel_online = true;
      wheel_speed_fdb_[wmi] = motor_ptr->vel();
      wheel_current_fdb_[wmi] = motor_ptr->curr();
    }
  }
  
  is_all_wheel_online_ = is_all_wheel_online;
  is_any_wheel_online_ = is_any_wheel_online;

  HW_ASSERT(yaw_motor_ptr_ != nullptr, "pointer to Yaw motor is nullptr", yaw_motor_ptr_);
  if (yaw_motor_ptr_->isOffline()) {
    theta_i2r_ = 0.0;
  } else {
    theta_i2r_ = -yaw_motor_ptr_->angle();
  }
};

void Chassis::updateCap()
{
  HW_ASSERT(cap_ptr_ != nullptr, "pointer to Capacitor is nullptr", cap_ptr_);
  if (cap_ptr_->isOffline()) {
    is_high_spd_enabled_ = false;
    cap_remaining_energy_ = 0.0f;
  } else {
    is_high_spd_enabled_ = cap_ptr_->isUsingSuperCap();
    cap_remaining_energy_ = cap_ptr_->getRemainingPower();
  }
};

void Chassis::updateIsPowerOn()
{
  is_power_on_ = is_any_wheel_online_ || rfr_data_.is_pwr_on;
  if (!is_power_on_) {
    last_pwr_off_tick_ = work_tick_;
  }
};

#pragma endregion

#pragma region 执行任务
void Chassis::run()
{
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

void Chassis::runOnDead()
{
  resetDataOnDead();
  setCommData(false);
};

void Chassis::runOnResurrection()
{
  resetDataOnResurrection();
  setCommData(false);
};

void Chassis::runOnWorking()
{
  revNormCmd();
  calcWheelSpeedRef();
  calcwheelfeedbackRef();
  calcWheelLimitedSpeedRef();
  calcWheelCurrentRef();
  setCommData(true);
};

void Chassis::standby()
{
  resetDataOnStandby();
  setCommData(false);
};
#pragma endregion

#pragma region 工作状态下，获取控制指令的函数
uint32_t cnt[4] = {0};
void Chassis::revNormCmd()
{
  float smooth_factor = cfg_.cmd_smooth_factor;
  Cmd cmd = norm_cmd_;
  WorkingMode act_working_mode = working_mode_;  // 实际执行的工作模式

  // 当小陀螺模式切换到跟随模式时，保证底盘不会反转，否则会大大消耗功率
  if (!is_gyro2follow_handled_) {
    float theta_i2r = getThetaI2r(false);
    if (!(working_mode_ == WorkingMode::Follow && last_working_mode_ == WorkingMode::Gyro) ||
        (gyro_dir_ == GyroDir::AntiClockwise && (theta_i2r < 0 && theta_i2r > -(5.0f / 6.0f * PI))) ||
        (gyro_dir_ == GyroDir::Clockwise && (theta_i2r > 0 && theta_i2r < (5.0f / 6.0f * PI)))) {
      is_gyro2follow_handled_ = true;
    } else {
      act_working_mode = WorkingMode::Gyro;
    }
  }
  cnt[3]++;
  switch (act_working_mode) {
    case WorkingMode::Gyro: {
      // 陀螺模式下，如果外部没有设置陀螺旋转方向，
      // 则每次进入陀螺模式时，随机选择一个方向
      if (gyro_dir_ == GyroDir::Unspecified) {
        if (last_gyro_dir_ == GyroDir::AntiClockwise)
        {
          gyro_dir_ = GyroDir::Clockwise;
        }
        else if (last_gyro_dir_ == GyroDir::Clockwise)
        {
          gyro_dir_ = GyroDir::AntiClockwise;
        }    
        else
        {
          gyro_dir_ = GyroDir::Clockwise;
        }    
        last_gyro_dir_ = gyro_dir_;
      }
      // 小陀螺模式下，旋转分量为定值
      // TODO(LKY) 之后可以优化为变速小陀螺
      cmd.w = 1.0f * (int8_t)gyro_dir_;
      if (cmd.v_x > fabs(0.1) || cmd.v_y > fabs(0.1)) {
        cmd.w *= 0.8f ;
        }// 是否需要根据底盘速度调整小陀螺速度
      break;
    }
    case WorkingMode::Depart: {
      gyro_dir_ = GyroDir::Unspecified;
      // 分离模式不对 norm_cmd_ 进行额外处理
      break;
    }
    case WorkingMode::Follow: {
      gyro_dir_ = GyroDir::Unspecified;
      // 在转头过程中，底盘不响应跟随转动指令
      if (work_tick_ - last_rev_head_tick_ < 800) {
        break;
      }
      // 跟随模式下，更新跟随目标
      float theta_ref[1] = {0.0f};
      float theta_fdb[1] = {theta_i2r_};
      // 如果云台反转，期望值是PI
      theta_ref[0] = PI * static_cast<float>(rev_head_flag_);
      follow_omega_pid_ptr_->calc(theta_ref, theta_fdb, nullptr, &cmd.w);
      cmd.w = hello_world::Bound(cmd.w, -1.0f, 1.0f);
      static auto data = follow_omega_pid_ptr_->getDatasAt(0);
      data = follow_omega_pid_ptr_->getDatasAt(0);
      break;
    }
    case WorkingMode::dead : {
      cmd.v_x = 0;
      cmd.v_y = 0;
      cmd.w = 0;
    }
    default: {
      break;
    }
  }

  if (is_high_spd_enabled_) {
    cmd *= 1.5;
    smooth_factor = 1.0;
  }

  // TODO(LKY) 修改限幅逻辑，保证限幅后平移速度方向不变
  cmd.v_x = hello_world::Bound(cmd.v_x * cfg_.normal_trans_vel, -cfg_.max_trans_vel, cfg_.max_trans_vel);
  cmd.v_y = hello_world::Bound(cmd.v_y * cfg_.normal_trans_vel, -cfg_.max_trans_vel, cfg_.max_trans_vel);
  cmd.w = hello_world::Bound(cmd.w * cfg_.normal_rot_spd, -cfg_.max_rot_spd, cfg_.max_rot_spd);

  setCmdSmoothly(cmd, smooth_factor);
};
void Chassis::calcWheelSpeedRef()
{
  // 底盘坐标系下，x轴正方向为底盘正前方，y轴正方向为底盘正左方，z轴正方向为底盘正上方
  // 轮子顺序按照象限顺序进行编号：左前，左后，右后，右前
  HW_ASSERT(ik_solver_ptr_ != nullptr, "pointer to IK solver is nullptr", ik_solver_ptr_);
  hello_world::chassis_ik_solver::MoveVec move_vec(cmd_.v_x, cmd_.v_y, cmd_.w);
  float theta_i2r = theta_i2r_;
  ik_solver_ptr_->solve(move_vec, theta_i2r, nullptr);
  ik_solver_ptr_->getRotSpdAll(wheel_speed_ref_);
};
float feedbackspeed[4] = {0};
float slope_ang = 0;
void Chassis::calcwheelfeedbackRef() 
{
  slope_ang = imu_ptr_->getSlopeAng();
  if (slope_ang > 0.2 && slope_ang < 0.5) {
    hello_world::chassis_ik_solver::MoveVec move_vec(imu_ptr_->getGx(), imu_ptr_->getGy(), 0);
    ik_solver_ptr_->solve(move_vec, 0, nullptr);
    ik_solver_ptr_->getRotSpdAll(feedbackspeed);
    for (size_t i = 0; i < 4; i++)
    {
      feedbackspeed[i] = feedbackspeed[i] * 2.3;
    }
}
else {
    for (size_t i = 0; i < 4; i++)
    {
      feedbackspeed[i] = 0;
    }
}

};
void Chassis::calcWheelLimitedSpeedRef()
{
  float up_ref = 120.0f;
  if (working_mode_ == WorkingMode::Gyro)
  {
    up_ref = 60.0f;
  }
  else
  {
    up_ref = 120.0f;
  }
  hello_world::power_limiter::PowerLimiterRuntimeParams runtime_params = {
    .p_ref_max =  up_ref + static_cast<float>(rfr_data_.pwr_limit), // 60.0f,//1.2f * rfr_data_.pwr_limit
    .p_referee_max = static_cast<float>(rfr_data_.pwr_limit),
    .p_ref_min = 0.8f * rfr_data_.pwr_limit,
    .remaining_energy = static_cast<float>(rfr_data_.pwr_buffer),
    .energy_converge = 50.0f,
    .p_slope = 2.0f,
    .danger_energy = 5.0f,
  };

  if (!cap_ptr_->isOffline()) {
    if (use_cap_flag_ == true)
    {
      runtime_params.remaining_energy = cap_ptr_->getRemainingPower();
      runtime_params.energy_converge = 30.0f;
      }
    else
    {
      runtime_params.remaining_energy = cap_ptr_->getRemainingPower();
      runtime_params.energy_converge = 50.0f;
      }
  }
  else
  {
    runtime_params.remaining_energy = static_cast<float>(rfr_data_.pwr_buffer);
    runtime_params.energy_converge = 10.0f;
  }
  
      pwr_limiter_ptr_->updateWheelModel(wheel_speed_ref_, wheel_speed_fdb_,
        feedbackspeed, nullptr);
      pwr_limiter_ptr_->calc(runtime_params, wheel_speed_ref_limited_,nullptr); // 更新运行时参数
};
void Chassis::calcPwrLimitedCurrentRef() {};
void Chassis::calcWheelCurrentRef()
{
  // 计算每个轮子的期望转速
  // 期望转速由 PID 控制器计算，期望转速与实际转速之间的差距由限幅器控制
  WheelPidIdx wpis[4] = {
      kWheelPidIdxLeftFront,
      kWheelPidIdxLeftRear,
      kWheelPidIdxRightRear,
      kWheelPidIdxRightFront,
  };
  MultiNodesPid *pid_ptr = nullptr;
  for (size_t i = 0; i < 4; i++) {
    pid_ptr = wheel_pid_ptr_[wpis[i]];
    HW_ASSERT(pid_ptr != nullptr, "pointer to PID %d is nullptr", wpis[i]);
    pid_ptr->calc(&wheel_speed_ref_limited_[i], &wheel_speed_fdb_[i], &feedbackspeed[i], &wheel_current_ref_[i]);
    // pid_ptr->calc(&wheel_speed_ref_[i], &wheel_speed_fdb_[i], &feedbackspeed[i], &wheel_current_ref_[i]);
    // pid_ptr->calc(&wheel_speed_ref_[i], &wheel_speed_fdb_[i], nullptr, &wheel_current_ref_[i]);
  }
};
void Chassis::calcWheelCurrentLimited() {

};
void Chassis::calcWheelRawInput() {

};
#pragma endregion

#pragma region 数据重置函数
void Chassis::reset()
{
  pwr_state_ = PwrState::Dead;       ///< 电源状态
  last_pwr_state_ = PwrState::Dead;  ///< 上一电源状态

  // 由 robot 设置的数据
  use_cap_flag_ = false;           ///< 是否使用超级电容
  gyro_dir_ = GyroDir::Unspecified;  ///< 小陀螺方向，正为绕 Z 轴逆时针，负为顺时针，
  norm_cmd_.reset();
  rfr_data_ = RfrData();  ///< 底盘 RFR 数据

  working_mode_ = WorkingMode::Depart;       ///< 工作模式
  last_working_mode_ = WorkingMode::Depart;  ///< 上一次工作模式

  // 在 update 函数中更新的数据
  is_power_on_ = false;  ///< 底盘电源是否开启
  // work_tick_ = 0;          ///< 记录底盘模块的运行时间，单位为 ms
  // last_pwr_off_tick_ = 0;  ///< 上一次底盘电源处于关闭状态的时间戳，单位为 ms，实际上是作为上电瞬间的记录

  // 在 runOnWorking 函数中更新的数据
  cmd_.reset();       ///< 控制指令，基于图传坐标系
  last_cmd_.reset();  ///< 上一控制周期的控制指令，基于图传坐标系

  memset(wheel_speed_ref_, 0, sizeof(wheel_speed_ref_));
  memset(wheel_speed_ref_limited_, 0, sizeof(wheel_speed_ref_limited_));
  memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));
  rev_head_flag_ = false;
  // last_rev_head_tick_ = 0;         ///< 上一次转向后退的时间戳

  // gimbal board fdb data  在 update 函数中更新
  is_gimbal_imu_ready_ = false;  ///< 云台主控板的IMU是否准备完毕

  // motor fdb data 在 update 函数中更新
  is_all_wheel_online_ = false;  ///< 所有轮电机是否都处于就绪状态
  is_any_wheel_online_ = false;  ///< 任意电机是否处于就绪状态

  memset(wheel_speed_fdb_, 0, sizeof(wheel_speed_fdb_));      ///< 轮速反馈数据
  memset(wheel_current_fdb_, 0, sizeof(wheel_current_fdb_));  ///< 轮电流反馈数据

  theta_i2r_ = 0.0f;  ///< 图传坐标系绕 Z 轴到底盘坐标系的旋转角度，右手定则判定正反向，单位 rad

  // cap fdb data 在 update 函数中更新
  is_high_spd_enabled_ = false;  ///< 是否开启了高速模式 （开启意味着从电容取电）
  cap_remaining_energy_ = 0.0f;  ///< 剩余电容能量百分比，单位 %

  resetPids();  ///< 重置 PID 控制器参数
};
void Chassis::resetDataOnDead()
{
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  cmd_.reset();       ///< 控制指令，基于图传坐标系
  last_cmd_.reset();  ///< 上一控制周期的控制指令，基于图传坐标系

  memset(wheel_speed_ref_, 0, sizeof(wheel_speed_ref_));
  memset(wheel_speed_ref_limited_, 0, sizeof(wheel_speed_ref_limited_));
  memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));
  rev_head_flag_ = false;
  // last_rev_head_tick_ = 0;         ///< 上一次转向后退的时间戳
  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetPids();  ///< 重置 PID 控制器参数
};
void Chassis::resetDataOnResurrection()
{
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  cmd_.reset();       ///< 控制指令，基于图传坐标系
  last_cmd_.reset();  ///< 上一控制周期的控制指令，基于图传坐标系

  memset(wheel_speed_ref_, 0, sizeof(wheel_speed_ref_));
  memset(wheel_speed_ref_limited_, 0, sizeof(wheel_speed_ref_limited_));
  memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));
  rev_head_flag_ = false;
  // last_rev_head_tick_ = 0;         ///< 上一次转向后退的时间戳
  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetPids();  ///< 重置 PID 控制器参数
};

void Chassis::resetDataOnStandby()
{
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  cmd_.reset();       ///< 控制指令，基于图传坐标系
  last_cmd_.reset();  ///< 上一控制周期的控制指令，基于图传坐标系

  setPwrState(PwrState::Dead);  ///< 电源状态
  memset(wheel_speed_ref_, 0, sizeof(wheel_speed_ref_));
  memset(wheel_speed_ref_limited_, 0, sizeof(wheel_speed_ref_limited_));
  memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));
  rev_head_flag_ = false;
  // last_rev_head_tick_ = 0;         ///< 上一次转向后退的时间戳
  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetPids();  ///< 重置 PID 控制器参数
};
void Chassis::resetPids()
{
  for (size_t i = 0; i < kWheelPidNum; i++) {
    wheel_pid_ptr_[i]->reset();
  }
  follow_omega_pid_ptr_->reset();
};
#pragma endregion

#pragma region 通讯数据设置函数

void Chassis::setCommDataWheels(bool working_flag)
{
  // 轮电机根据期望电流输入发送数据
  WheelMotorIdx wmis[4] = {
      kWheelMotorIdxLeftFront,
      kWheelMotorIdxLeftRear,
      kWheelMotorIdxRightRear,
      kWheelMotorIdxRightFront,
  };
  WheelPidIdx wpis[4] = {
      kWheelPidIdxLeftFront,
      kWheelPidIdxLeftRear,
      kWheelPidIdxRightRear,
      kWheelPidIdxRightFront,
  };

  Motor *motor_ptr = nullptr;
  MultiNodesPid *pid_ptr = nullptr;

  for (size_t i = 0; i < 4; i++) {
    WheelMotorIdx wmi = wmis[i];
    WheelPidIdx wpi = wpis[i];
    motor_ptr = wheel_motor_ptr_[wmi];
    pid_ptr = wheel_pid_ptr_[wpi];
    HW_ASSERT(motor_ptr != nullptr, "pointer to motor %d is nullptr", wmi);
    if (!working_flag || motor_ptr->isOffline()) {
      pid_ptr->reset();
      motor_ptr->setInput(0);
    } else {
      motor_ptr->setInput(wheel_current_ref_[wmi]);
    }
  }
};
void Chassis::setCommDataCap(bool working_flag)
{
  // 电容根据电容充电状态发送数据
  HW_ASSERT(cap_ptr_ != nullptr, "pointer to Capacitor is nullptr", cap_ptr_);
  if (rfr_data_.is_rfr_on && rfr_data_.is_pwr_on) {
    cap_ptr_->setRfrData(rfr_data_.pwr_buffer, rfr_data_.pwr_limit, rfr_data_.current_hp);
  } else {
    cap_ptr_->setRfrData(rfr_data_.pwr_buffer, rfr_data_.pwr_limit, 0);
  }
};

#pragma endregion

#pragma region 注册函数

void Chassis::registerIkSolver(ChassisIkSolver *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to IK solver is nullptr", ptr);
  ik_solver_ptr_ = ptr;
};

void Chassis::registerWheelMotor(Motor *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kWheelMotorNum, "index of motor out of range", idx);
  wheel_motor_ptr_[idx] = ptr;
};

void Chassis::registerYawMotor(Motor *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to Yaw motor is nullptr", ptr);
  yaw_motor_ptr_ = ptr;
}

void Chassis::registerWheelPid(MultiNodesPid *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kWheelPidNum, "index of PID out of range", idx);
  wheel_pid_ptr_[idx] = ptr;
};

void Chassis::registerFollowOmegaPid(MultiNodesPid *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID is nullptr", ptr);
  follow_omega_pid_ptr_ = ptr;
};

void Chassis::registerPwrLimiter(PwrLimiter *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to PwrLimiter is nullptr", ptr);
  pwr_limiter_ptr_ = ptr;
};

void Chassis::registerCap(Cap *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to Capacitor is nullptr", ptr);
  cap_ptr_ = ptr;
};

void Chassis::registerGimbalChassisComm(GimbalChassisComm *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to GimbalChassisComm is nullptr", ptr);
  gc_comm_ptr_ = ptr;
};
void Chassis::registerImu(Imu *ptr)
{
  HW_ASSERT(ptr != nullptr, "IMU pointer is null", ptr);
  imu_ptr_ = ptr;
};

#pragma endregion

#pragma region 特殊接口
void Chassis::setWorkingMode(WorkingMode mode)
{
  // 防止小陀螺切跟随模式时反转，大大消耗功率
  if (mode == WorkingMode::Follow && working_mode_ == WorkingMode::Gyro) {
    is_gyro2follow_handled_ = false;
  }
  if (working_mode_ != mode) {
    last_working_mode_ = working_mode_;
    working_mode_ = mode;
  }
}
/** 
 * @brief       得到图传坐标系绕 Z 轴到底盘坐标系的旋转角度，
 *              右手定则判定正反向，单位 rad
 * @param        actual_head_dir: 是否指定实际车头为底盘坐标系X轴正向。
 *              若需考虑运行过程中的车头定义改变(rev_head_flag)，则设置该参数为false。
 * @retval      图传坐标系绕 Z 轴到底盘坐标系的旋转角度
 * @note        None
 */
float Chassis::getThetaI2r(bool actual_head_dir) const
{
  if (actual_head_dir == true) {
    return theta_i2r_;
  }

  if (rev_head_flag_) {
    return hello_world::AngleNormRad(theta_i2r_ + PI);
  } else {
    return theta_i2r_;
  }
}

/** 
 * @brief       设置旋转方向，要求旋转方向必须有定义
 * @param        dir: 旋转方向，GyroDir
 * @note        用户在设置小陀螺模式后，可调用此函数指定小陀螺旋转方向
 *              非小陀螺模式，用户无需指定小陀螺旋转方向为Unspecified，
 *              底盘状态机会自行做该处理。
 */
void Chassis::setGyroDir(GyroDir dir)
{
  if (dir != GyroDir::Unspecified) {
    gyro_dir_ = dir;
  }
}
#pragma endregion

/* Private function definitions ----------------------------------------------*/
}  // namespace robot