/* Includes ------------------------------------------------------------------*/
#include "fric.hpp"

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

void Fric::update()
{
  updateData();
  updatePwrState();
};

void Fric::updateData()
{
  updateWorkTick();
  
  updateMotorData();
};

void Fric::updatePwrState()
{
  // 从电机状态获取电源状态，裁判系统发上来的信息可能因为固件版本等问题出现错误
  is_power_on_ = is_any_motor_pwr_on_;

  if (!is_power_on_){
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
    if (fric_status_.is_inited){
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

void Fric::updateMotorData()
{
  bool is_any_motor_pwr_on = false;
  Motor *motor_ptr = nullptr;

  MotorIdx mi_frics[2] = {kMotorIdxFricLeft, kMotorIdxFricRight};
  FricStatus::FricIdx fi_fric[2] = {FricStatus::kFricIdxLeft, FricStatus::kFricIdxRight};
  for (size_t i = 0; i < 2; i++){
    motor_ptr = motor_ptr_[mi_frics[i]];
    HW_ASSERT(motor_ptr != nullptr, "ptr to fric motor %d is nullptr", mi_frics[i]);
    fric_status_.last_spd_fdb[fi_fric[i]] = fric_status_.spd_fdb[fi_fric[i]];
    if (motor_ptr->isOffline()){
      fric_status_.spd_fdb[fi_fric[i]] = 0;
      fric_status_.cur_fdb[fi_fric[i]] = 0;

      fric_status_.resetHoldStatus();
      fric_status_.resetStuckStatus();
    }else{
      fric_status_.spd_fdb[fi_fric[i]] = motor_ptr->vel();
      fric_status_.cur_fdb[fi_fric[i]] = motor_ptr->curr();

      is_any_motor_pwr_on = true;

      updateMotorFricStuckStatus();
      updateMotorFricHoldStatus();
    };
  }

  is_any_motor_pwr_on_ = is_any_motor_pwr_on;
};

void Fric::updateRfrData(const FricRfrInputData &inp_data)
{
  if (rfr_data_.bullet_speed != inp_data.bullet_speed) {
    // 弹速需要限制在最小和最大值之间，超过此值意味着数据有误
    if (inp_data.bullet_speed >= cfg_.min_bullet_speed && inp_data.bullet_speed <= cfg_.max_bullet_speed) {
      rfr_data_.last_bullet_speed = rfr_data_.bullet_speed;
      rfr_data_.bullet_speed = rfr_data_.last_bullet_speed * 0.5f + inp_data.bullet_speed * 0.5f;
    }
  }
  // 更新当前数据
  rfr_data_.is_power_on = inp_data.is_power_on;
};

void Fric::updateMotorFricStuckStatus()
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

void Fric::updateMotorFricHoldStatus()
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

void Fric::run()
{
  if (pwr_state_ == PwrState::Dead){
    runOnDead();
  }else if (pwr_state_ == PwrState::Resurrection){
    runOnResurrection();
  }else if (pwr_state_ == PwrState::Working){
    runOnWorking();
  }
};

void Fric::runOnDead()
{
  resetDataOnDead();
  setCommData(false);
};

void Fric::runOnResurrection()
{
  resurrection_duration_ += interval_ticks_;
  resetDataOnResurrection();
  calcFricSpdRefOnResurrection();
  calcMotorFricInput();
  setCommData(true);
};

void Fric::runOnWorking()
{
  calcFricSpdRefOnWorking();
  calcMotorFricInput();
  setCommData(true);
};

void Fric::standby()
{
  fric_status_.spd_ref = 0;
  memset(fric_status_.cur_ref, 0, sizeof(fric_status_.cur_ref));

  setCommData(false);
};

void Fric::calcFricSpdRefOnResurrection()
{
  // 当摩擦轮卡住时，摩擦轮停转，防止电机堵转并出现损坏
  if (working_mode_ == ShooterWorkingMode::Stop || fric_status_.stuck_status != StuckStatus::None) {
    fric_status_.spd_ref = 0.0f;
    return;
  }
  if (working_mode_ == WorkingMode::FricBackward) {
    fric_status_.spd_ref = cfg_.tgt_fric_spd_ref_backward;
  } else {
    fric_status_.spd_ref = cfg_.tgt_fric_spd_ref;
    if (fric_status_.spd_hold_duration > 200) {
      fric_status_.is_inited = true;
    }
  }
};

void Fric::calcFricSpdRefOnWorking()
{
  if (working_mode_ == WorkingMode::FricBackward) {
    // 倒转模式是为了将卡在摩擦轮中间的弹丸回退出来，转速不易过快
    // TODO: 此数值可能需要后续细调
    fric_status_.spd_ref = cfg_.tgt_fric_spd_ref_backward;
    return;
  }
  // 正常模式、嗜血模式下，摩擦轮的转速正常
  // 当摩擦轮卡住时，摩擦轮停转，防止电机堵转并出现损坏
  if (working_mode_ == WorkingMode::Stop || fric_status_.stuck_status != StuckStatus::None) {
    fric_status_.spd_ref = 0.0f;
    return;
  }
  // 弹速更新且超出期望阈值时，调整摩擦轮转速使弹速收敛
  float rfr_blt_spd = rfr_data_.bullet_speed;
  if (rfr_blt_spd != rfr_data_.last_bullet_speed && rfr_blt_spd != 0) {
    if (rfr_blt_spd > cfg_.tgt_max_bullet_speed) {
      cfg_.tgt_fric_spd_ref -= 5.0f;
    } else if (rfr_blt_spd < cfg_.tgt_min_bullet_speed) {
      cfg_.tgt_fric_spd_ref += 5.0f;
    }
  }
  fric_status_.spd_ref = cfg_.tgt_fric_spd_ref;
};

void Fric::calcMotorFricInput()
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
#pragma endregion

#pragma region 数据重置

/** 
 * @brief 供外部调用的复位函数
 * 
 * 会重置大部分数据，包括电源状态、控制模式、工作模式、电机指令等
 */
void Fric::reset()
{
  pwr_state_ = PwrState::Dead;
  last_pwr_state_ = PwrState::Dead;

  working_mode_ = WorkingMode::Normal;
  is_power_on_ = false;
  is_any_motor_pwr_on_ = false;

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
void Fric::resetDataOnDead()
{
  setPwrState(PwrState::Dead);

  resurrection_duration_ = 0;

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
void Fric::resetDataOnResurrection()
{
  setPwrState(PwrState::Resurrection);

  // 以下变量仅在工作模式下被调用
  fric_status_.is_inited = false;
};

/**
 * @brief 重置所有的PID控制器
 * 
 * 这个函数遍历所有的PID控制器，如果控制器存在，就调用其reset方法进行重置。
 * 
 * @note 这个函数不会创建或删除任何PID控制器，只是重置它们的状态。
 */
void Fric::resetPids()
{
  for (size_t i = 0; i < kPidNum; i++) {
    Pid *pid_ptr = pid_ptr_[i];
    if (pid_ptr != nullptr) {
      pid_ptr->reset();
    }
  }
};

#pragma endregion

#pragma region 设置通信组件数据

/**
 * @brief 设置通信数据
 * 
 * 这个函数设置摩擦轮和弹仓电机的电流期望值。如果电机在线，就设置电流期望值为计算得到的值；
 * 如果电机离线，就设置电流期望值为0。
 * 
 * @note 这个函数不会创建或删除任何电机或离线检查器，只是设置它们的状态。
 */
void Fric::setCommData(bool is_working)
{
  float motor_cur_ref[2] = {
      fric_status_.cur_ref[FricStatus::kFricIdxLeft],
      fric_status_.cur_ref[FricStatus::kFricIdxRight],
  };
  MotorIdx mi_frics[2] = {kMotorIdxFricLeft, kMotorIdxFricRight};
  PidIdx pi_frics[2] = {kPidIdxFricLeft, kPidIdxFricRight};
  Motor *motor_ptr = nullptr;
  Pid *pid_ptr = nullptr;
  for (size_t i = 0; i < 2; i++) {
    motor_ptr = motor_ptr_[mi_frics[i]];
    pid_ptr = pid_ptr_[pi_frics[i]];
    HW_ASSERT(motor_ptr != nullptr, "ptr to fric motor %d is nullptr", mi_frics[i]);
    if ((!motor_ptr->isOffline()) && is_working) {
      pid_ptr->reset();
      motor_ptr->setInput(motor_cur_ref[i]);
    } else {
      motor_ptr->setInput(0);
    };
  }
};

#pragma endregion

#pragma region 注册函数

void Fric::registerMotor(Motor *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kMotorNum, "index of motor out of range", idx);
  motor_ptr_[idx] = ptr;
};
void Fric::registerPid(Pid *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kPidNum, "index of PID out of range", idx);
  pid_ptr_[idx] = ptr;
};
#pragma endregion

/* Private function definitions ----------------------------------------------*/
} // namespace robot