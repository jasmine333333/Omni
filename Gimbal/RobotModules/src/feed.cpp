/* Includes ------------------------------------------------------------------*/
#include "feed.hpp"
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
void Feed::update()
{
  updateData();
  updatePwrState();
};

void Feed::updateData()
{
  updateWorkTick();

  updateMotorData();
};

void Feed::updatePwrState()
{
  // 从电机状态获取电源状态，裁判系统发上来的信息可能因为固件版本等问题出现错误
  is_power_on_ = is_any_motor_pwr_on_;

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
    // A1: 1.) 摩擦轮上电控制时瞬时电流很大；
    //     2.) 需要摩擦轮正常启动之后才能启动拨盘，避免弹丸卡在摩擦轮中，导致摩擦轮堵转
    // Q2: 为什么要判断拨盘是否准备就绪？
    // A2: 复活模式下需要根据拨盘反馈角度，设置拨盘的目标角度；如果拨盘未上电，拨盘反馈角度为 0 ，
    //     使用此值来计算目标角度会导致拨盘控制出错，可能导致超发、连发等问题；
    //     如果拨盘为非绝对编码，需要将拨盘复位（反向转直到电机堵转，并重置电机零点）
    if (feed_status_.is_ang_inited && is_fric_inited_) {
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

void Feed::updateMotorData()
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

  is_any_motor_pwr_on_ = is_any_motor_pwr_on;
};

/** 
 * @brief       更新拨弹盘所需的裁判系统数据
 * @param        &inp_data: 结构体引用
 * @note        在Robot的updateGimbalChassisCommData中调用
 */
void Feed::updateRfrData(const FeedRfrInputData &inp_data)
{
  // 记录上一不同的数据
  if (rfr_data_.heat != inp_data.heat) {
    rfr_data_.last_heat_ = rfr_data_.heat;
  }

  // 更新当前数据
  rfr_data_.is_rfr_on = inp_data.is_rfr_on;
  rfr_data_.is_power_on = inp_data.is_power_on;
  rfr_data_.is_new_bullet_shot = inp_data.is_new_bullet_shot;
  rfr_data_.heat_limit = inp_data.heat_limit;
  rfr_data_.heat = inp_data.heat;
  rfr_data_.heat_cooling_ps = inp_data.heat_cooling_ps;
};

void Feed::updateMotorFeedStuckStatus()
{
  if (feed_status_.cur_fdb <= -cfg_.feed_stuck_curr_thre) {
    feed_status_.stuck_duration += interval_ticks_;
    if (feed_status_.stuck_duration >= 200) {
      feed_status_.stuck_status = StuckStatus::Backward;
    }
  } else if (feed_status_.cur_fdb >= cfg_.feed_stuck_curr_thre) {
    feed_status_.stuck_duration += interval_ticks_;
    if (feed_status_.stuck_duration >= 200) {
      feed_status_.stuck_status = StuckStatus::Forward;
    }
  } else {
    feed_status_.stuck_duration = 0;
    feed_status_.stuck_status = StuckStatus::None;
  }
};

void Feed::updateMotorFeedHoldStatus()
{
  float feed_fdb_delta = hello_world::PeriodDataSub(feed_status_.ang_fdb, feed_status_.last_ang_fdb, 2 * PI);
  if (fabsf(feed_fdb_delta) < 0.001) {
    feed_status_.ang_hold_duration += interval_ticks_;
  } else {
    feed_status_.ang_hold_duration = 0;
  }
};

#pragma endregion

#pragma region 执行任务

void Feed::run()
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

void Feed::runOnDead()
{
  resetDataOnDead();
  setCommData(false);
};

void Feed::runOnResurrection()
{
  resurrection_duration_ += interval_ticks_;
  resetDataOnResurrection();
  calcFeedAngRefOnResurrection();
  if (feed_status_.is_ang_inited == false) {
    calcMotorFeedInput();
  }
  setCommData(true);
};

void Feed::runOnWorking()
{
  genTriggerSignal();
  calcFakeHeat();
  calcFeedAngRefOnWorking();
  calcMotorFeedInput();
  setCommData(true);
};

void Feed::standby()
{
  fake_heat_ = 0.0f;

  trigger_signal_ = false;

  feed_status_.ang_ref = 0;
  feed_status_.cur_ref = 0;

  setCommData(false);
};

void Feed::genTriggerSignal()
{
  if (feed_status_.stuck_status == StuckStatus::Forward) {
    trigger_signal_ = false;
    return;
  }
  bool shoot_flag = false;
  if (ctrl_mode_ == CtrlMode::Manual) {
    shoot_flag = manual_shoot_flag_;
  } else if (ctrl_mode_ == CtrlMode::Auto) {
    shoot_flag = (vision_shoot_flag_ == Vision::ShootFlag::kShootOnce);
  }
  switch (working_mode_) {
    case WorkingMode::Normal:
      trigger_signal_ = shoot_flag && limitTriggerByFreq(200);
      trigger_signal_ = trigger_signal_ && limitTriggerByHeat(1.5);
      break;
    case WorkingMode::Crazy:
      trigger_signal_ = shoot_flag && limitTriggerByFreq(100);
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
  }
}

void Feed::calcFakeHeat()
{
  if (rfr_data_.is_rfr_on) {
    // 裁判系统在线，以裁判系统的发弹数据包是否更新判断有无发弹，以此维护假热量
    // 此处假热量的作用是防止裁判系统热量更新延迟导致意外超热量死亡
    fake_heat_ += cfg_.heat_per_bullet * static_cast<float>(rfr_data_.is_new_bullet_shot);
  } else {
    // 裁判系统离线，用拨弹盘是否拨动维护假热量
    // 此处假热量的作用是完全替代裁判系统热量
    // 双发会导致fake_heat_偏低，空拨会导致fake_heat偏高，暂时没有好的处理方法
    // 发生卡弹，热量回退
    if (feed_status_.stuck_status == StuckStatus::Forward) {
      fake_heat_ -= cfg_.heat_per_bullet;
    }
    fake_heat_ += cfg_.heat_per_bullet * static_cast<float>(trigger_signal_);
  }
  // 热量冷却
  fake_heat_ -= rfr_data_.heat_cooling_ps * interval_ticks_ * 0.001f;
  fake_heat_ = std::max(fake_heat_, 0.0f);
}

bool Feed::limitTriggerByFreq(uint32_t interval) { return (work_tick_ - last_trigger_tick_) > interval; };
bool Feed::limitTriggerByHeat(float safe_num_remain_bullet)
{
  float heat = rfr_data_.heat > fake_heat_ ? rfr_data_.heat : fake_heat_;
  float num_allowed_trigger = (rfr_data_.heat_limit - heat) / cfg_.heat_per_bullet;
  return num_allowed_trigger >= safe_num_remain_bullet;
};

void Feed::calcFeedAngRefOnResurrection()
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

    feed_status_.resetStuckStatus();
    return;
  }
};

void Feed::calcFeedAngRefOnWorking()
{
  if (working_mode_ == WorkingMode::FricBackward) {
    return;
  }

  if (feed_status_.stuck_status == StuckStatus::Forward) {
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

  // 非堵转情况下，才会生成trigger_signal_
  if (trigger_signal_) {
    if (feed_status_.is_ang_ref_inited == false) {
      feed_status_.ang_ref = searchFeedAngRef(feed_status_.ang_fdb, true, cfg_.feed_ang_per_blt);
      feed_status_.is_ang_ref_inited = true;
    } else {
      // 触发信号有效时，计算目标角度
      feed_status_.ang_ref += cfg_.feed_ang_per_blt;
    }
  }

  feed_status_.ang_ref = hello_world::AngleNormRad(feed_status_.ang_ref);
};

void Feed::calcMotorFeedInput()
{
  // 拨弹电机
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
#pragma endregion

#pragma region 数据重置

/** 
 * @brief 供外部调用的复位函数
 * 
 * 会重置大部分数据，包括电源状态、控制模式、工作模式、电机指令等
 */
void Feed::reset()
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
void Feed::resetDataOnDead()
{
  setPwrState(PwrState::Dead);

  fake_heat_ = 0.0f;

  trigger_signal_ = false;

  resurrection_duration_ = 0;

  feed_status_.is_ang_inited = false;
  feed_status_.is_ang_ref_inited = false;
  feed_status_.resetHoldStatus();
  feed_status_.resetStuckStatus();

  feed_status_.ang_ref = 0;
  feed_status_.cur_ref = 0;

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
void Feed::resetDataOnResurrection()
{
  setPwrState(PwrState::Resurrection);

  // 以下变量仅在工作模式下被调用
  fake_heat_ = 0.0f;
  trigger_signal_ = false;
  feed_status_.is_ang_ref_inited = false;
};

/**
 * @brief 重置所有的PID控制器
 * 
 * 这个函数遍历所有的PID控制器，如果控制器存在，就调用其reset方法进行重置。
 * 
 * @note 这个函数不会创建或删除任何PID控制器，只是重置它们的状态。
 */
void Feed::resetPids()
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
 * 这个函数设置拨弹电机的电流期望值。如果电机在线，就设置电流期望值为计算得到的值；
 * 如果电机离线，就设置电流期望值为0。
 * 
 * @note 这个函数不会创建或删除任何电机或离线检查器，只是设置它们的状态。
 */
void Feed::setCommData(bool is_working)
{
  float motor_cur_ref = feed_status_.cur_ref;

  Motor *motor_ptr = motor_ptr_[kMotorIdxFeed];
  Pid *pid_ptr = pid_ptr_[kPidIdxFeed];

  HW_ASSERT(motor_ptr != nullptr, "pointer to Feed Motor is nullptr", motor_ptr);
  HW_ASSERT(pid_ptr != nullptr, "pointer to Feed PID is nullptr", pid_ptr);

  if ((!motor_ptr->isOffline()) && is_working) {
    motor_ptr->setInput(motor_cur_ref);
  } else {
    pid_ptr->reset();
    motor_ptr->setInput(0);
  }
};

#pragma endregion

#pragma region 注册函数
void Feed::registerMotor(Motor *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kMotorNum, "index of motor out of range", idx);
  motor_ptr_[idx] = ptr;
};
void Feed::registerPid(Pid *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kPidNum, "index of PID out of range", idx);
  pid_ptr_[idx] = ptr;
};
#pragma endregion

#pragma region 其他工具函数
float Feed::searchFeedAngRef(float fdb_ang, float offset, bool is_farward, float ang_per_bullet) const
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
}  // namespace robot