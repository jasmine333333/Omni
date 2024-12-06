/** 
 *******************************************************************************
 * @file      : shooter.cpp
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
#include "shooter.hpp"

#include <cstring>
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

void Shooter::update()
{
  updateData();
  updatePwrState();
}

void Shooter::updateData()
{
  updateWorkTick();

  updateMotorData();
};

void Shooter::updatePwrState()
{
  is_power_on_ = rfr_data_.is_power_on || is_any_motor_pwr_on_;

  // 无论任何状态，断电意味着要切到死亡状态
  if (!is_power_on_) {
    setPwrState(PwrState::Dead);
    return;
  }

  PwrState current_state = pwr_state_;
  PwrState next_state = current_state;
  if (current_state == PwrState::Dead) {
    // 死亡状态下，如果上电，则切到复活状态
    if (is_power_on_) {
      next_state = PwrState::Resurrection;
    }
  } else if (current_state == PwrState::Resurrection) {
    // 复活状态下，如果摩擦轮准备就绪且拨盘准备就绪，则切到工作状态
    // Q1: 为什么要判断摩擦轮是否准备就绪？
    // A1: 1.) 摩擦轮上电控制时瞬时电流很大；2.) 需要摩擦轮正常启动之后才能启动拨盘，避免弹丸卡在摩擦轮中，导致摩擦轮堵转
    // Q2: 为什么要判断拨盘是否准备就绪？
    // A2: 复活模式下需要根据拨盘反馈角度，设置拨盘的目标角度；如果拨盘未上电，拨盘反馈角度为 0 ，使用此值来计算目标角度会导致拨盘控制出错，可能导致超发、连发等问题；如果拨盘为非绝对编码，需要将拨盘复位（反向转直到电机堵转，并重置电机零点）
    if (feed_status_.is_ang_inited && fric_status_.is_inited) {
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

void Shooter::updateMotorData()
{
  bool is_any_motor_pwr_on = false;
  Motor *motor_ptr = nullptr;

  motor_ptr = motor_ptr_[kMotorIdxFeed];
  HW_ASSERT(motor_ptr != nullptr, "ptr to feed motor is nullptr", motor_ptr);
  feed_status_.last_ang_fdb = feed_status_.ang_fdb;
  if (motor_ptr->isOffline()) {
    feed_status_.ang_fdb = 0;
    feed_status_.spd_fdb = 0;
    feed_status_.cur_fdb = 0;

    feed_status_.resetHoldStatus();
    feed_status_.resetStuckStatus();
  } else {
    feed_status_.ang_fdb = motor_ptr->angle();
    feed_status_.spd_fdb = motor_ptr->vel();
    feed_status_.cur_fdb = motor_ptr->curr();

    is_any_motor_pwr_on = true;

    updateMotorFeedHoldStatus();
    updateMotorFeedStuckStatus();
  }

  MotorIdx mi_frics[2] = {kMotorIdxFricLeft, kMotorIdxFricRight};
  FricStatus::FricIdx fi_fric[2] = {FricStatus::kFricIdxLeft, FricStatus::kFricIdxRight};
  for (size_t i = 0; i < 2; i++) {
    motor_ptr = motor_ptr_[mi_frics[i]];
    HW_ASSERT(motor_ptr != nullptr, "ptr to fric motor %d is nullptr", mi_frics[i]);
    fric_status_.last_spd_fdb[fi_fric[i]] = fric_status_.spd_fdb[fi_fric[i]];
    if (motor_ptr->isOffline()) {
      fric_status_.spd_fdb[fi_fric[i]] = 0;
      fric_status_.cur_fdb[fi_fric[i]] = 0;

      fric_status_.resetHoldStatus();
      fric_status_.resetStuckStatus();
    } else {
      fric_status_.spd_fdb[fi_fric[i]] = motor_ptr->vel();
      fric_status_.cur_fdb[fi_fric[i]] = motor_ptr->curr();

      is_any_motor_pwr_on = true;

      updateMotorFricStuckStatus();
      updateMotorFricHoldStatus();
    };
  }

  is_any_motor_pwr_on_ = is_any_motor_pwr_on;
};

void Shooter::updateRfrData(const RfrInputData &inp_data)
{
  // 记录上一不同的数据
  if (rfr_data_.heat != inp_data.heat) {
    rfr_data_.last_heat_ = rfr_data_.heat;
  }
  if (rfr_data_.bullet_speed != inp_data.bullet_speed) {
    // 弹速需要限制在最小和最大值之间，超过此值意味着数据有误
    if (inp_data.bullet_speed >= cfg_.min_bullet_speed && inp_data.bullet_speed <= cfg_.max_bullet_speed) {
      rfr_data_.last_bullet_speed = rfr_data_.bullet_speed;
      rfr_data_.bullet_speed = rfr_data_.last_bullet_speed * 0.5f + inp_data.bullet_speed * 0.5f;
    }
  }
  // 更新当前数据
  rfr_data_.is_power_on = inp_data.is_power_on;
  rfr_data_.heat_limit = inp_data.heat_limit;
  rfr_data_.heat = inp_data.heat;
  rfr_data_.heat_cooling_ps = inp_data.heat_cooling_ps;
};

void Shooter::updateMotorFeedStuckStatus()
{
  if (feed_status_.cur_fdb <= -cfg_.feed_stuck_curr_thre) {
    feed_status_.stuck_duration += interval_ticks_;
    if (feed_status_.stuck_duration >= 90) {
      feed_status_.stuck_status = StuckStatus::Backward;
    }
  } else if (feed_status_.cur_fdb >= cfg_.feed_stuck_curr_thre) {
    feed_status_.stuck_duration += interval_ticks_;
    if (feed_status_.stuck_duration >= 90) {
      feed_status_.stuck_status = StuckStatus::Farward;
    }
  } else {
    feed_status_.stuck_duration = 0;
    feed_status_.stuck_status = StuckStatus::None;
  }
}

void Shooter::updateMotorFricStuckStatus()
{
  if (fric_status_.cur_fdb[FricStatus::kFricIdxLeft] <= -cfg_.fric_stuck_curr_thre ||
      fric_status_.cur_fdb[FricStatus::kFricIdxRight] <= -cfg_.fric_stuck_curr_thre) {
    fric_status_.stuck_duration += interval_ticks_;
    if (fric_status_.stuck_duration >= 90) {
      fric_status_.stuck_status = StuckStatus::Backward;
    }
  } else if (fric_status_.cur_fdb[FricStatus::kFricIdxLeft] >= cfg_.fric_stuck_curr_thre ||
             fric_status_.cur_fdb[FricStatus::kFricIdxRight] >= cfg_.fric_stuck_curr_thre) {
    fric_status_.stuck_duration += interval_ticks_;
    if (fric_status_.stuck_duration >= 90) {
      fric_status_.stuck_status = StuckStatus::Backward;
    }
  } else {
    fric_status_.stuck_duration = 0;
    fric_status_.stuck_status = StuckStatus::None;
  }
};

void Shooter::updateMotorFeedHoldStatus()
{
  float feed_fdb_delta = hello_world::PeriodDataSub(feed_status_.ang_fdb, feed_status_.last_ang_fdb, 2 * PI);
  if (fabsf(feed_fdb_delta) < 0.001) {
    feed_status_.ang_hold_duration += interval_ticks_;
  } else {
    feed_status_.ang_hold_duration = 0;
  }
};

void Shooter::updateMotorFricHoldStatus()
{
  bool hold_flag[2] = {false, false};

  for (size_t i = 0; i < 2; i++) {
    // ! 注意：这里的 spd_fdb_delta 和 spd_err 的容许值需要根据实际情况调整
    float spd_fdb_delta = fric_status_.spd_fdb[i] - fric_status_.last_spd_fdb[i];
    float spd_err = fric_status_.spd_ref - fric_status_.spd_fdb[i];
    hold_flag[i] = fabsf(spd_fdb_delta) < fabsf(cfg_.fric_spd_delta_thre) && fabsf(spd_err) < fabsf(cfg_.fric_spd_err_thre);
  }

  if (hold_flag[0] && hold_flag[1]) {
    fric_status_.spd_hold_duration += interval_ticks_;
  } else {
    fric_status_.spd_hold_duration = 0;
  }
};

#pragma endregion

#pragma region 执行任务
void Shooter::run()
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
void Shooter::runOnDead()
{
  resetDataOnDead();
  setCommData(false);
};

void Shooter::runOnResurrection()
{
  resurrection_duration_ += interval_ticks_;
  resetDataOnResurrection();
  calcFricSpdRefOnResurrection();
  calcFeedAngRefOnResurrection();

  // calcMotorInput();
  if (feed_status_.is_ang_inited == false) {
    calcMotorFeedInput();
  }
  calcMotorFricInput();
  setCommData(true);
};

void Shooter::runOnWorking()
{
  uint32_t work_interval = work_tick_ - last_work_tick_;

  fake_heat_ -= rfr_data_.heat_cooling_ps * work_interval * 0.001f;
  fake_heat_ = std::max(fake_heat_, 0.0f);

  if (shoot_flag_) {
    continue_shoot_duration_ += work_interval;
  } else {
    continue_shoot_duration_ = 0;
  }

  switch (working_mode_) {
    case WorkingMode::Normal:
      if (continue_shoot_duration_ > 100) {
        // 连发模式，每5/6秒发射一颗弹丸（前哨站旋转装甲板的间隔）
        trigger_signal_ = shoot_flag_ && limitTriggerByFreq(833);
      } else {
        trigger_signal_ = shoot_flag_ && limitTriggerByFreq(200);
      }
      trigger_signal_ = trigger_signal_ && limitTriggerByHeat(1.5);
      break;
    case WorkingMode::Crazy:
      trigger_signal_ = shoot_flag_ && limitTriggerByFreq(100);
      break;
    case WorkingMode::FricBackward:
      trigger_signal_ = false;
      break;
    default:
      trigger_signal_ = false;
      break;
  }

  if (trigger_signal_) {
    last_trigger_tick_ = work_tick_;
    fake_heat_ += cfg_.heat_per_bullet;
  }

  calcFricSpdRefOnWorking();
  calcFeedAngRefOnWorking();
  calcMotorInput();
  setCommData(true);
}
void Shooter::standby()
{
  fake_heat_ = 0.0f;

  trigger_signal_ = false;

  continue_shoot_duration_ = 0;

  feed_status_.ang_ref = 0;
  feed_status_.cur_ref = 0;

  fric_status_.spd_ref = 0;
  memset(fric_status_.cur_ref, 0, sizeof(fric_status_.cur_ref));

  setCommData(false);
}
bool Shooter::limitTriggerByFreq(uint32_t interval) { return (work_tick_ - last_trigger_tick_) > interval; };
bool Shooter::limitTriggerByHeat(float safe_num_remain_bullet)
{
  float heat = rfr_data_.heat > fake_heat_ ? rfr_data_.heat : fake_heat_;
  float num_allowed_trigger = (rfr_data_.heat_limit - heat) / cfg_.heat_per_bullet;
  return num_allowed_trigger >= safe_num_remain_bullet;
};

void Shooter::calcFeedAngRefOnResurrection()
{
  Motor *motor_ptr = motor_ptr_[kMotorIdxFeed];
  HW_ASSERT(motor_ptr != nullptr, "pointer to Feed Motor is nullptr", motor_ptr_);
  HW_ASSERT(pid_ptr_ != nullptr, "pointer to Feed PID is nullptr", pid_ptr_);

  // 当拨盘初始化之后哪怕再进入该函数也直接退出
  if (feed_status_.is_ang_inited == true) {
    return;
  }

  // 拨盘以缓慢的速度反向旋转，直至接触到限位【期望状态】或因为其他原因卡住【意外状态，无法检测】
  // 假设现在所有的拨盘电机都不是多圈绝对双编码的，需要一个复位操作
  // 理想状态是将限位设置为 0°
  const float delta = 5.0f / 180.0f * PI;
  feed_status_.ang_ref = hello_world::AngleNormRad(feed_status_.ang_fdb - delta);

  if (resurrection_duration_ <= 500) {
    return;
  }

  // 以下状态不会保持(每个控制周期都会根据反馈值重新判断)
  // 反向接触到限位可能有两种表现情况：
  // 1. 电流较大，出现堵转现象并被程序发现
  // 2. 电流较小，反馈角度长时间没有变化
  if (feed_status_.stuck_status == StuckStatus::Backward || feed_status_.ang_hold_duration > 100) {
    feed_status_.is_ang_inited = true;
    motor_ptr->setAngleValue(0.0f);
    feed_status_.ang_ref = 0.0f;

    feed_status_.stuck_status = StuckStatus::None;
    feed_status_.stuck_duration = 0;

    return;
  }
};
void Shooter::calcFeedAngRefOnWorking()
{
  if (working_mode_ == WorkingMode::FricBackward) {
    return;
  }

  if (trigger_signal_) {
    if (feed_status_.is_ang_ref_inited == false) {
      feed_status_.ang_ref = searchFeedAngRef(feed_status_.ang_fdb, true, cfg_.feed_ang_per_blt);
      feed_status_.is_ang_ref_inited = true;
    } else {
      // 触发信号有效时，计算目标角度
      feed_status_.ang_ref += cfg_.feed_ang_per_blt;
    }
  }

  if (feed_status_.stuck_status == StuckStatus::Farward) {
    // 拨盘前向堵转时，回退目标角度
    feed_status_.ang_ref -= cfg_.feed_ang_per_blt;
    // 清空堵转计时器，给电机充分时间回退
    // 清除前向堵转标志位，避免连续回退
    feed_status_.resetStuckStatus();
  }

  if (feed_status_.stuck_status == StuckStatus::Backward) {
    // 拨盘后向堵转时，将目标角度设置为当前反馈角度略微往前一点
    feed_status_.ang_ref = feed_status_.ang_fdb + 0.01f;
    // 清空堵转计时器，给电机充分时间前向转动
    // 清除前向堵转标志位，避免连续回退
    feed_status_.resetStuckStatus();
    feed_status_.is_ang_ref_inited = false;
  }

  feed_status_.ang_ref = hello_world::AngleNormRad(feed_status_.ang_ref);
};
void Shooter::calcFricSpdRefOnResurrection()
{
  // 当摩擦轮卡住时，摩擦轮停转，防止电机堵转并出现损坏
  if (fric_status_.stuck_status != StuckStatus::None) {
    fric_status_.spd_ref = 0.0f;
  } else {
    if (working_mode_ == WorkingMode::FricBackward) {
      fric_status_.spd_ref = -fric_spd_ref_tbs_;
    } else {
      fric_status_.spd_ref = fric_spd_ref_tbs_;
      if (fric_status_.spd_hold_duration > 200) {
        fric_status_.is_inited = true;
      }
    }
  }
};
void Shooter::calcFricSpdRefOnWorking()
{
  if (working_mode_ == WorkingMode::FricBackward) {
    // 倒转模式是为了将卡在摩擦轮中间的弹丸回退出来，转速不易过快
    // TODO(ZSC): 此数值可能需要后续细调
    fric_status_.spd_ref = -fric_spd_ref_tbs_ * 0.1;
  } else {
    // 正常模式、嗜血模式下，摩擦轮的转速正常
    // 当摩擦轮卡住时，摩擦轮停转，防止电机堵转并出现损坏
    if (fric_status_.stuck_status != StuckStatus::None) {
      fric_status_.spd_ref = 0.0f;
    } else {
      // 弹速更新且超过安全阈值时，降低转速
      if (rfr_data_.bullet_speed > 15.7) {
        // TODO(ZSC): 此数值可能需要后续细调
        fric_spd_ref_tbs_ -= 10.0f;
      }
      fric_status_.spd_ref = fric_spd_ref_tbs_;
    }
  }
};

void Shooter::calcMotorFeedInput()
{
  // 弹仓电机
  Motor *motor_ptr = motor_ptr_[kMotorIdxFeed];
  HW_ASSERT(motor_ptr != nullptr, "pointer to Feed Motor is nullptr", motor_ptr_);
  Pid *pid_ptr = pid_ptr_[kPidIdxFeed];
  HW_ASSERT(pid_ptr_ != nullptr, "pointer to Feed PID is nullptr", pid_ptr);

  if (motor_ptr->isOffline()) {
    feed_status_.cur_ref = 0.0f;
  } else {
    float ref = feed_status_.ang_ref;
    float fdb[2] = {feed_status_.ang_fdb, feed_status_.spd_fdb};
    pid_ptr->calc(&ref, fdb, nullptr, &feed_status_.cur_ref);
  }
};
void Shooter::calcMotorFricInput()
{
  Motor *motor_ptr = nullptr;
  Pid *pid_ptr = nullptr;
  // 摩擦轮
  MotorIdx mi_frics[2] = {kMotorIdxFricLeft, kMotorIdxFricRight};
  PidIdx pi_frics[2] = {kPidIdxFricLeft, kPidIdxFricRight};
  FricStatus::FricIdx fi_frics[2] = {FricStatus::kFricIdxLeft, FricStatus::kFricIdxRight};

  for (size_t i = 0; i < 2; i++) {
    motor_ptr = motor_ptr_[mi_frics[i]];
    pid_ptr = pid_ptr_[pi_frics[i]];

    HW_ASSERT(motor_ptr != nullptr, "pointer to Fric %d Motor is nullptr", mi_frics[i]);
    HW_ASSERT(pid_ptr_ != nullptr, "pointer to Fric %d PID is nullptr", pi_frics[i]);

    if (motor_ptr->isOffline()) {
      fric_status_.cur_ref[fi_frics[i]] = 0.0f;
    } else {
      float ref = fric_status_.spd_ref;
      float fdb[2] = {fric_status_.spd_fdb[fi_frics[i]], fric_status_.cur_fdb[fi_frics[i]]};
      pid_ptr->calc(&ref, fdb, nullptr, &fric_status_.cur_ref[fi_frics[i]]);
    }
  }
};
void Shooter::calcMotorInput()
{
  calcMotorFeedInput();
  calcMotorFricInput();
};
#pragma endregion

#pragma region 数据重置

/** 
 * @brief 供外部调用的复位函数
 * 
 * 会重置大部分数据，包括电源状态、控制模式、工作模式、电机指令等
 */
void Shooter::reset()
{
  pwr_state_ = PwrState::Dead;
  last_pwr_state_ = PwrState::Dead;

  ctrl_mode_ = CtrlMode::Manual;

  working_mode_ = WorkingMode::Normal;
  is_power_on_ = false;
  is_any_motor_pwr_on_ = false;

  fake_heat_ = 0.0f;

  trigger_signal_ = false;
  last_trigger_tick_ = 0;

  fric_spd_ref_tbs_ = cfg_.tgt_fric_spd_ref;

  continue_shoot_duration_ = 0;

  feed_status_.is_ang_inited = false;
  feed_status_.is_ang_ref_inited = false;
  feed_status_.resetHoldStatus();
  feed_status_.resetStuckStatus();

  feed_status_.ang_ref = 0;
  feed_status_.ang_fdb = 0;
  feed_status_.last_ang_fdb = 0;
  feed_status_.spd_fdb = 0;
  feed_status_.cur_fdb = 0;
  feed_status_.cur_ref = 0;

  fric_status_.is_inited = false;
  fric_status_.resetHoldStatus();
  fric_status_.resetStuckStatus();
  fric_status_.spd_ref = 0;
  memset(fric_status_.spd_fdb, 0, sizeof(fric_status_.spd_fdb));
  memset(fric_status_.cur_fdb, 0, sizeof(fric_status_.cur_fdb));
  memset(fric_status_.cur_ref, 0, sizeof(fric_status_.cur_ref));
};

/** 
 * @brief 复位数据(在死亡状态下)
 * 
 * 仅重置部分数据，包括电源状态、电机指令等。
 * 不重置控制模式、工作模式等外部传递进来的指令。
 * 不重置由外部传递活内部通讯组件维护的状态数据。
 * 
 * @note 该函数仅供内部调用，外部调用请使用 reset() 函数
 */
void Shooter::resetDataOnDead()
{
  setPwrState(PwrState::Dead);

  fake_heat_ = 0.0f;

  trigger_signal_ = false;

  continue_shoot_duration_ = 0;
  resurrection_duration_ = 0;

  feed_status_.is_ang_inited = false;
  feed_status_.is_ang_ref_inited = false;
  feed_status_.resetHoldStatus();
  feed_status_.resetStuckStatus();

  feed_status_.ang_ref = 0;
  feed_status_.cur_ref = 0;

  fric_status_.is_inited = false;
  fric_status_.resetHoldStatus();
  fric_status_.resetStuckStatus();
  fric_status_.spd_ref = 0;
  memset(fric_status_.cur_ref, 0, sizeof(fric_status_.cur_ref));
};
/** 
 * @brief 复位数据(在复活状态下)
 * 
 * 仅重置部分数据，包括电源状态、电机指令等。
 * 不重置控制模式、工作模式等外部传递进来的指令。
 * 不重置由外部传递活内部通讯组件维护的状态数据。
 * 不重置在复活状态下需要用到的数据。
 * 
 * @note 该函数仅供内部调用，外部调用请使用 reset() 函数
 */
void Shooter::resetDataOnResurrection()
{
  setPwrState(PwrState::Resurrection);

  // 以下变量仅在工作模式下被调用
  fake_heat_ = 0.0f;
  trigger_signal_ = false;
  continue_shoot_duration_ = 0;
  feed_status_.is_ang_ref_inited = false;
};

/**
 * @brief 重置所有的PID控制器
 * 
 * 这个函数遍历所有的PID控制器，如果控制器存在，就调用其reset方法进行重置。
 * 
 * @note 这个函数不会创建或删除任何PID控制器，只是重置它们的状态。
 */
void Shooter::resetPids()
{
  for (size_t i = 0; i < kPidNum; i++) {
    Pid *pid_ptr = pid_ptr_[i];
    if (pid_ptr != nullptr) {
      pid_ptr->reset();
    }
  }
};
#pragma endregion

#pragma region 通信数据设置函数

/**
 * @brief 设置通信数据
 * 
 * 这个函数设置摩擦轮和弹仓电机的电流期望值。如果电机在线，就设置电流期望值为计算得到的值；
 * 如果电机离线，就设置电流期望值为0。
 * 
 * @note 这个函数不会创建或删除任何电机或离线检查器，只是设置它们的状态。
 */
void Shooter::setCommData(bool is_working)
{
  float motor_cur_ref[3] = {
      fric_status_.cur_ref[FricStatus::kFricIdxLeft],
      fric_status_.cur_ref[FricStatus::kFricIdxRight],
      feed_status_.cur_ref,
  };
  MotorIdx mi_frics[3] = {kMotorIdxFricLeft, kMotorIdxFricRight, kMotorIdxFeed};
  Motor *motor_ptr = nullptr;
  for (size_t i = 0; i < 3; i++) {
    motor_ptr = motor_ptr_[mi_frics[i]];
    HW_ASSERT(motor_ptr != nullptr, "ptr to fric motor %d is nullptr", mi_frics[i]);
    if ((!motor_ptr->isOffline()) && is_working) {
      motor_ptr->setInput(motor_cur_ref[i]);
    } else {
      motor_ptr->setInput(0);
    };
  }
};

#pragma endregion

#pragma region 注册函数

void Shooter::registerMotor(Motor *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kMotorNum, "index of motor out of range", idx);
  motor_ptr_[idx] = ptr;
};
void Shooter::registerPid(Pid *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kPidNum, "index of PID out of range", idx);
  pid_ptr_[idx] = ptr;
};
#pragma endregion

#pragma region 其他工具函数
float Shooter::searchFeedAngRef(float fdb_ang, float offset, bool is_farward, float ang_per_bullet) const
{
  float delta = 0;
  float ref_ang = offset;
  for (size_t i = 0; i < 6; i++) {
    ref_ang += ang_per_bullet;
    delta = hello_world::PeriodDataSub(ref_ang, fdb_ang, 2 * PI);
    if ((!is_farward) && (-ang_per_bullet < delta) && (delta <= 0)) {
      break;
    } else if (is_farward && (0 < delta) && (delta < ang_per_bullet)) {
      break;
    }
  }
  return ref_ang;
};
#pragma endregion

/* Private function definitions ----------------------------------------------*/
}  // namespace robot