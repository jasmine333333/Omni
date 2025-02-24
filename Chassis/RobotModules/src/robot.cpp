/** 
 *******************************************************************************
 * @file      :robot.cpp
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
#include "robot.hpp"
#include "usart.h"
/* Private macro -------------------------------------------------------------*/

namespace robot
{
/* Private constants ---------------------------------------------------------*/
const hello_world::referee::RobotPerformanceData kDefaultRobotPerformanceData = {
    .robot_id = static_cast<uint8_t>(hello_world::referee::ids::RobotId::kRedStandard3),

    .robot_level = 0,   ///< 机器人等级
    .current_hp = 200,  ///< 机器人当前血量
    .maximum_hp = 200,  ///< 机器人血量上限

    .shooter_barrel_cooling_value = 40,  ///< 机器人枪口热量每秒冷却值
    .shooter_barrel_heat_limit = 100,    ///< 机器人枪口热量上限

    .chassis_power_limit = 55,  ///< 底盘功率限制，若不选择底盘或发射机构类型，则在七分钟比赛阶段开始后，未选择的底盘性能类型将被默认选择为“血量优先”

    .power_management_gimbal_output = 1,   ///< 电源管理模块 gimbal 口输出：0-关闭，1-开启
    .power_management_chassis_output = 1,  ///< 电源管理模块 chassis 口输出：0-关闭，1-开启
    .power_management_shooter_output = 1,  ///< 电源管理模块 shooter 口输出：0-关闭，1-开启
};
const hello_world::referee::RobotPowerHeatData kDefaultRobotPowerHeatData = {
    .chassis_voltage = 24000,  ///< 电源管理模块的chassis口输出电压，单位：mV
    .chassis_current = 0,      ///< 电源管理模块的chassis口输出电流，单位：mA
    .chassis_power = 0,        ///< 底盘功率，单位：W
    .buffer_energy = 60,       ///< 缓冲能量，单位：J

    .shooter_17mm_1_barrel_heat = 0,  ///< 第一个17mm发射机构的枪口热量
    .shooter_17mm_2_barrel_heat = 0,  ///< 第二个17mm发射机构的枪口热量
    .shooter_42mm_barrel_heat = 0,    ///< 42mm发射机构的枪口热量
};
const hello_world::referee::RobotShooterData kDefaultRobotShooterData = {
    .bullet_type =static_cast<uint8_t>(hello_world::referee::BulletType::k17mm),  ///< 弹丸类型，@see BulletType

    .shooter_id = static_cast<uint8_t>(hello_world::referee::ShooterId::k17mm1),  ///< 发射机构ID，@see ShooterId

    .launching_frequency = 0,  ///< 弹丸射频，单位：Hz
    .bullet_speed = 15.5f,     ///< 弹丸初速度，单位：m/s
};
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

#pragma region 数据更新
// 状态机主要接口函数
void Robot::update()
{
  updateData();
  updatePwrState();
};

void Robot::updateData()
{
  updateWorkTick();
  updateImuData();
  updateRfrData();
  updateRcData();
};

void Robot::updateImuData()
{
  HW_ASSERT(imu_ptr_ != nullptr, "IMU pointer is null", imu_ptr_);
  imu_ptr_->update();
  if (imu_ptr_->isNormWorking()) {
    is_imu_caled_offset_ = true;
  }
};

void Robot::updateRcData()
{
  HW_ASSERT(rc_ptr_ != nullptr, "RC pointer is null", rc_ptr_);
  if (manual_ctrl_src_ == ManualCtrlSrc::Rc) {
    if (rc_ptr_->isUsingKeyboardMouse()) {
      setManualCtrlSrc(ManualCtrlSrc::Kb);
    }
  } else if (manual_ctrl_src_ == ManualCtrlSrc::Kb) {
    if (rc_ptr_->isRcSwitchChanged()) {  // 不检测摇杆变化，防止控制源来回切换
      setManualCtrlSrc(ManualCtrlSrc::Rc);
    }
  }
};

void Robot::updateRfrData()
{
  HW_ASSERT(referee_ptr_ != nullptr, "RFR pointer is null", referee_ptr_);
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null", chassis_ptr_);

  Chassis::RfrData chassis_rfr_data;

  PerformancePkg::Data rpp_data = kDefaultRobotPerformanceData;
  PowerHeatPkg::Data rph_data = kDefaultRobotPowerHeatData;
  ShooterPkg::Data rsp_data = kDefaultRobotShooterData;

  bool is_new_bullet_shot = false;

  if (!referee_ptr_->isOffline()) {
    rpp_data = rfr_performance_pkg_ptr_->getData();
    rph_data = rfr_power_heat_pkg_ptr_->getData();
    rsp_data = rfr_shooter_pkg_ptr_->getData();
    if (!rfr_shooter_pkg_ptr_->isHandled()) {  // 检测到了一颗新的弹丸发射
      rfr_shooter_pkg_ptr_->setHandled();
      is_new_bullet_shot = true;
    }
  }

  if (!referee_ptr_->isOffline()) {
    robot_id_ = (RobotId)rpp_data.robot_id;    
  } else {
    robot_id_ = RobotId::kRedStandard3;
  }
  ui_drawer_.setSenderId(robot_id_);

  chassis_rfr_data.is_rfr_on = (!referee_ptr_->isOffline());
  chassis_rfr_data.is_pwr_on = rpp_data.power_management_chassis_output;
  chassis_rfr_data.pwr_limit = rpp_data.chassis_power_limit;
  chassis_rfr_data.voltage = rph_data.chassis_voltage;
  chassis_rfr_data.pwr_buffer = rph_data.buffer_energy;
  chassis_rfr_data.pwr = rph_data.chassis_power;
  chassis_rfr_data.current_hp = rpp_data.current_hp;
  // chassis_rfr_data.current_hp = 0;
  chassis_ptr_->setRfrData(chassis_rfr_data);

  GimbalChassisComm::RefereeData::ChassisPart &gimbal_rfr_data = gc_comm_ptr_->referee_data().cp;
  gimbal_rfr_data.is_rfr_on = (!referee_ptr_->isOffline());
  gimbal_rfr_data.is_rfr_shooter_power_on = rpp_data.power_management_shooter_output;
  gimbal_rfr_data.is_rfr_gimbal_power_on = rpp_data.power_management_gimbal_output;
  gimbal_rfr_data.robot_id = (RobotId)rpp_data.robot_id;

  // TODO: 换成实际的枪口
  gimbal_rfr_data.bullet_speed = rsp_data.bullet_speed;
  gimbal_rfr_data.shooter_heat = rph_data.shooter_17mm_1_barrel_heat;
  gimbal_rfr_data.shooter_cooling = rpp_data.shooter_barrel_cooling_value;
  gimbal_rfr_data.shooter_heat_limit = rpp_data.shooter_barrel_heat_limit;
  gimbal_rfr_data.is_new_bullet_shot = is_new_bullet_shot;
};


void Robot::updatePwrState()
{
  PwrState pre_state = pwr_state_;
  PwrState next_state = pre_state;
  if (pre_state == PwrState::Dead) {
    // 主控板程序在跑就意味着有电，所以直接从死亡状态进入复活状态
    next_state = PwrState::Resurrection;
  } else if (pre_state == PwrState::Resurrection) {
    if (is_imu_caled_offset_  && work_tick_ > 1000) {
      next_state = PwrState::Working;
    }
  } else if (pre_state == PwrState::Working) {
    // 工作状态下，保持当前状态
  } else {
    // 其他状态，认为是死亡状态
    next_state = PwrState::Dead;
  }
  setPwrState(next_state);
};

#pragma endregion

#pragma region 执行任务

void Robot::run()
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

  setCommData();
  sendCommData();

};

void Robot::standby()
{
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null", chassis_ptr_);
  chassis_ptr_->update();
  chassis_ptr_->standby();
};

void Robot::runOnDead()
{
  resetDataOnDead();
  standby();
};

void Robot::runOnResurrection()
{
  resetDataOnResurrection();
  standby();
};

void Robot::runOnWorking()
{
  if (buzzer_ptr_ != nullptr) {
    buzzer_ptr_->play();
  }
  genModulesCmd();
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null", chassis_ptr_);
  chassis_ptr_->update();
  chassis_ptr_->run();
};

#pragma endregion

#pragma region 生成控制指令
void Robot::genModulesCmd()
{
  // TODO: 这里应该生成各个模块的指令，包括底盘指令、云台指令、底盘状态等等
  // 指令应该通过各个模块的接口发送给各个模块
  if (manual_ctrl_src_ == ManualCtrlSrc::Rc) {
    genModulesCmdFromRc();
  } else if (manual_ctrl_src_ == ManualCtrlSrc::Kb) {
    genModulesCmdFromKb();
  }
};

void Robot::genModulesCmdFromRc()
{
  // TODO: 这里应该生成各个模块的指令，包括底盘指令、云台指令、底盘状态等等
  // 指令应该通过各个模块的接口发送给各个模块
  RcSwitchState l_switch = rc_ptr_->rc_l_switch();
  RcSwitchState r_switch = rc_ptr_->rc_r_switch();
  float rc_wheel = rc_ptr_->rc_wheel();
  Chassis::GyroDir gyro_dir = Chassis::GyroDir::Unspecified;
  bool use_cap_flag = false;

  Chassis::WorkingMode chassis_working_mode = Chassis::WorkingMode::Follow;

  CtrlMode gimbal_ctrl_mode = CtrlMode::Manual;
  Gimbal::WorkingMode gimbal_working_mode = Gimbal::WorkingMode::Normal;
  CtrlMode feed_ctrl_mode = CtrlMode::Manual;
  Shooter::WorkingMode shooter_working_mode = Shooter::WorkingMode::Stop;
  Feed::WorkingMode feed_working_mode = Feed::WorkingMode::Normal;

  bool shoot_flag = false;

  // TODO: 后续需要加入慢拨模式
  if (r_switch == RcSwitchState::kDown)
  {
    chassis_working_mode = Chassis::WorkingMode::dead;
    gimbal_working_mode = Gimbal::WorkingMode::dead;
    shooter_working_mode = Shooter::WorkingMode::Stop;
    feed_working_mode = Feed::WorkingMode::Stop;
  }
  else{
  if (l_switch == RcSwitchState::kUp) {
    // * 左上
    chassis_working_mode = Chassis::WorkingMode::Depart;
    gimbal_working_mode = Gimbal::WorkingMode::Normal;
    if (r_switch == RcSwitchState::kUp) {
      gimbal_ctrl_mode = CtrlMode::Auto;
      shooter_working_mode = Shooter::WorkingMode::Normal;
      feed_ctrl_mode = CtrlMode::Manual;
      shoot_flag = (rc_wheel > 0.9f);
      // * 左上右上
    } else if (r_switch == RcSwitchState::kMid) {
      gimbal_ctrl_mode = CtrlMode::Auto ;
      feed_ctrl_mode = CtrlMode::Auto;
      shooter_working_mode = Shooter::WorkingMode::Normal;
      shoot_flag = (rc_wheel > 0.9f);
       // * 左上右中
    } else if (r_switch == RcSwitchState::kDown) {
      // * 左上右下
    }
  } else if (l_switch == RcSwitchState::kMid) {
    // * 左中
    chassis_working_mode = Chassis::WorkingMode::Follow;
    gimbal_working_mode = Gimbal::WorkingMode::Normal;
    gimbal_ctrl_mode = CtrlMode::Manual;
    feed_ctrl_mode = CtrlMode::Manual;
    if (r_switch == RcSwitchState::kUp) {
      // * 左中右上
      shooter_working_mode = Shooter::WorkingMode::Stop;

    } else if (r_switch == RcSwitchState::kMid) {
      // * 左中右中
      use_cap_flag = true;
      shooter_working_mode = Shooter::WorkingMode::Normal;
      shoot_flag = (rc_wheel > 0.9f);

    } else if (r_switch == RcSwitchState::kDown) {
      // * 左中右下
    }
  } else if (l_switch == RcSwitchState::kDown) {
    // * 左下
    chassis_working_mode = Chassis::WorkingMode::Gyro;
    gimbal_working_mode = Gimbal::WorkingMode::Normal;
    gimbal_ctrl_mode = CtrlMode::Manual;
    shooter_working_mode = Shooter::WorkingMode::Stop;
    if (r_switch == RcSwitchState::kUp) {
      // * 左下右上
      use_cap_flag = true;
      gyro_dir = Chassis::GyroDir::Clockwise;
    } else if (r_switch == RcSwitchState::kMid) {
      // * 左下右中
      gyro_dir = Chassis::GyroDir::Clockwise;
    } else if (r_switch == RcSwitchState::kDown) {
      // * 左下右下
      gyro_dir = Chassis::GyroDir::AntiClockwise;
    }
  }
  }

  chassis_ptr_->setWorkingMode(chassis_working_mode);
  chassis_ptr_->setGyroDir(gyro_dir);
  chassis_ptr_->setUseCapFlag(use_cap_flag);

  ChassisCmd chassis_cmd = {0};
  chassis_cmd.v_x = hello_world::Bound(rc_ptr_->rc_rv(), -1, 1);
  chassis_cmd.v_y = hello_world::Bound(-rc_ptr_->rc_rh(), -1, 1);
  chassis_cmd.w = 0;
  chassis_ptr_->setNormCmd(chassis_cmd);

  gimbal_ptr_->setCtrlMode(gimbal_ctrl_mode);
  gimbal_ptr_->setWorkingMode(gimbal_working_mode);

  GimbalCmd gimbal_cmd;
  gimbal_cmd.pitch = hello_world::Bound(rc_ptr_->rc_lv(), -1, 1);
  gimbal_cmd.yaw = hello_world::Bound(-rc_ptr_->rc_lh(), -1, 1);  // 右手系，z轴竖直向上，左转为正
  gimbal_ptr_->setNormCmdDelta(gimbal_cmd);

  shooter_ptr_->setWorkingMode(shooter_working_mode);

  feed_ptr_->setCtrlMode(feed_ctrl_mode);
  feed_ptr_->setWorkingMode(feed_working_mode);
  feed_ptr_->setShootFlag(shoot_flag);
};

void Robot::genModulesCmdFromKb()
{
  // TODO: 这里应该生成各个模块的指令，包括底盘指令、云台指令、底盘状态等等
  // 指令应该通过各个模块的接口发送给各个模块
  Chassis::WorkingMode chassis_working_mode = chassis_ptr_->getWorkingMode();

  CtrlMode gimbal_ctrl_mode = CtrlMode::Manual;
  Gimbal::WorkingMode gimbal_working_mode = Gimbal::WorkingMode::Normal;
  bool rev_head_flag = false;

  CtrlMode feed_ctrl_mode = CtrlMode::Manual;
  Shooter::WorkingMode shooter_working_mode = Shooter::WorkingMode::Normal;
  Feed::WorkingMode feed_working_mode = Feed::WorkingMode::Normal;
  bool shoot_flag = false;


  if (rc_ptr_->key_Q()) {
    chassis_working_mode = Chassis::WorkingMode::Gyro;
  } else if (rc_ptr_->key_E()) {
    chassis_working_mode = Chassis::WorkingMode::Follow;
  } else if (rc_ptr_->key_X()) {

  } else {
    if (chassis_working_mode == Chassis::WorkingMode::Depart) {
      chassis_working_mode = Chassis::WorkingMode::Follow;
    }
  }
  if (rc_ptr_->key_B()) {
    rev_head_flag = true;
  }
  if (rc_ptr_->key_CTRL()) {
    
  }
  if (rc_ptr_->key_Z()) {
    shooter_working_mode = Shooter::WorkingMode::FricBackward;
  }
  if (rc_ptr_->key_R()) {
  }
  if (rc_ptr_->key_F()) {
  }

  if (rc_ptr_->mouse_l_btn()) {
    shoot_flag = true;
  }
  if (rc_ptr_->mouse_r_btn()) {
    feed_ctrl_mode = CtrlMode::Auto;
    gimbal_ctrl_mode = CtrlMode::Auto;
  }

  ChassisCmd chassis_cmd = {0};
  chassis_cmd.v_x = (int8_t)rc_ptr_->key_W() - (int8_t)rc_ptr_->key_S();
  chassis_cmd.v_y = (int8_t)rc_ptr_->key_A() - (int8_t)rc_ptr_->key_D();
  chassis_cmd.w = 0;
  chassis_ptr_->setNormCmd(chassis_cmd);
  chassis_ptr_->setWorkingMode(chassis_working_mode);
  chassis_ptr_->setUseCapFlag(rc_ptr_->key_SHIFT());
  if (rev_head_flag) chassis_ptr_->revHead();

  GimbalCmd gimbal_cmd;
  gimbal_cmd.pitch = hello_world::Bound(-0.01 * rc_ptr_->mouse_y(), -1, 1);
  gimbal_cmd.yaw = hello_world::Bound(-0.01 * rc_ptr_->mouse_x(), -1, 1);
  gimbal_ptr_->setNormCmdDelta(gimbal_cmd);
  gimbal_ptr_->setCtrlMode(gimbal_ctrl_mode);
  gimbal_ptr_->setWorkingMode(gimbal_working_mode);
  gimbal_ptr_->setRevHeadFlag(rev_head_flag);

  feed_ptr_->setCtrlMode(feed_ctrl_mode);
  feed_ptr_->setWorkingMode(feed_working_mode);
  feed_ptr_->setShootFlag(shoot_flag);

  shooter_ptr_->setWorkingMode(shooter_working_mode);

};

#pragma endregion

#pragma region 数据重置函数
void Robot::reset()
{
  pwr_state_ = PwrState::Dead;       ///< 电源状态
  last_pwr_state_ = PwrState::Dead;  ///< 上一电源状态

  manual_ctrl_src_ = ManualCtrlSrc::Rc;       ///< 手动控制源
  last_manual_ctrl_src_ = ManualCtrlSrc::Rc;  ///< 上一手动控制源
};
void Robot::resetDataOnDead()
{
  pwr_state_ = PwrState::Dead;       ///< 电源状态
  last_pwr_state_ = PwrState::Dead;  ///< 上一电源状态

  manual_ctrl_src_ = ManualCtrlSrc::Rc;       ///< 手动控制源
  last_manual_ctrl_src_ = ManualCtrlSrc::Rc;  ///< 上一手动控制源
};
void Robot::resetDataOnResurrection() {};

#pragma endregion

#pragma region 通信数据设置函数
void Robot::setCommData()
{
  // TODO(ZSC): 可能以后会在这里分工作状态
  setGimbalChassisCommData();
  setUiDrawerData();
  // TODO(ZSC): 可能的其他通讯数据设置函数
  // 其他通讯模块的数据由各个子模块负责设置
  // 主控板非工作模式时，这些数据保持默认值
};

void Robot::setGimbalChassisCommData()
{
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null", gc_comm_ptr_);
  HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal pointer is null", gimbal_ptr_);
  HW_ASSERT(shooter_ptr_ != nullptr, "Shooter pointer is null", shooter_ptr_);

  // main board

  // gimbal
  GimbalChassisComm::GimbalData::ChassisPart &gimbal_data = gc_comm_ptr_->gimbal_data().cp;
  gimbal_data.turn_back_flag = gimbal_ptr_->getRevHeadFlag();
  gimbal_data.yaw_delta = gimbal_ptr_->getNormCmdDelta().yaw;
  gimbal_data.pitch_delta = gimbal_ptr_->getNormCmdDelta().pitch;
  gimbal_data.ctrl_mode = gimbal_ptr_->getCtrlMode();
  gimbal_data.working_mode = gimbal_ptr_->getWorkingMode();


  // shooter
  GimbalChassisComm::ShooterData::ChassisPart &shooter_data = gc_comm_ptr_->shooter_data().cp;
  shooter_data.setShootFlag(feed_ptr_->getShootFlag());
  shooter_data.ctrl_mode = feed_ptr_->getCtrlMode();
  shooter_data.working_mode = shooter_ptr_->getWorkingMode();
};

void Robot::setUiDrawerData() {

  // ui_drawer_.setSenderId(RobotId::kRedStandard3);
  // Chassis
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null", chassis_ptr_);
  ui_drawer_.setChassisWorkState(chassis_ptr_->getPwrState());
  // ui_drawer_.setChassisCtrlMode();
  ui_drawer_.setChassisWorkingMode(chassis_ptr_->getWorkingMode());
  // ui_drawer_.setChassisManualCtrlSrc(chassis_ptr_->getManualCtrlSrc());
  ui_drawer_.setChassisHeadDir(chassis_ptr_->getThetaI2r());

  // Gimbal
  HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal FSM pointer is null", gimbal_ptr_);
  // ui_drawer_.setGimbalWorkState(chassis_ptr_->getPwrState());
  ui_drawer_.setGimbalCtrlMode(gimbal_ptr_->getCtrlMode());
  ui_drawer_.setGimbalWorkingMode(gimbal_ptr_->getWorkingMode());
  // ui_drawer_.setGimbalManualCtrlSrc(manual_ctrl_src_);
  ui_drawer_.setGimbalJointAngPitchFdb(gc_comm_ptr_->gimbal_data().gp.pitch_fdb);
  ui_drawer_.setGimbalJointAngPitchRef(gc_comm_ptr_->gimbal_data().gp.pitch_ref);
  ui_drawer_.setGimbalJointAngYawFdb(gc_comm_ptr_->gimbal_data().gp.yaw_fdb);
  ui_drawer_.setGimbalJointAngYawRef(gc_comm_ptr_->gimbal_data().gp.yaw_ref);

  // Shooter
  HW_ASSERT(shooter_ptr_ != nullptr, "Shooter FSM pointer is null", shooter_ptr_);
  ui_drawer_.setShooterWorkState(chassis_ptr_->getPwrState());
  ui_drawer_.setShooterCtrlMode(feed_ptr_->getCtrlMode());
  ui_drawer_.setShooterWorkingMode(shooter_ptr_->getWorkingMode());
  // ui_drawer_.setShooterManualCtrlSrc(chassis_ptr_->ge);
  // ui_drawer_.setHeat(gc_comm_ptr_->);
  // ui_drawer_.setHeatLimit(shooter_ptr_->heat_limit());

  // ui_drawer_.setFeedAngFdb(gc_comm_ptr_->shooter_data().gp.feed_ang_fdb);
  // ui_drawer_.setFeedAngRef(gc_comm_ptr_->shooter_data().gp.feed_ang_ref);
  ui_drawer_.setFeedStuckFlag(gc_comm_ptr_->shooter_data().gp.feed_stuck_state);
  ui_drawer_.setFricSpdFdb(gc_comm_ptr_->shooter_data().gp.fric_spd_fdb);
  ui_drawer_.setFricSpdRef(gc_comm_ptr_->shooter_data().gp.fric_spd_ref);
  ui_drawer_.setFricStuckFlag(gc_comm_ptr_->shooter_data().gp.is_fric_stuck_);


  // // Cap
  // HW_ASSERT(cap_ptr_ != nullptr, "Cap pointer is null", cap_ptr_);
  // ui_drawer_.setCapPwrPercent(cap_ptr_->getRemainingPowerPercent());

  // vision
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null", gc_comm_ptr_);
  bool is_vision_valid =gc_comm_ptr_->vision_data().gp.is_enemy_detected != 0;
  ui_drawer_.setVisTgtX(gc_comm_ptr_->vision_data().gp.vtm_x, is_vision_valid);
  ui_drawer_.setVisTgtY(gc_comm_ptr_->vision_data().gp.vtm_y, is_vision_valid);

  if (rc_ptr_->key_V()) {
    ui_drawer_.refresh();
  }

  
};

#pragma endregion

#pragma region 通信数据发送函数

void Robot::sendCommData()
{
  sendCanData();
  sendUsartData();
};
void Robot::sendCanData()
{
  if (work_tick_ > 1500)
  {
    sendWheelsMotorData();
  }
  
  if (work_tick_ % 10 == 0) {
    sendCapData();
  }
  if (work_tick_ > 1010) {
    sendGimbalChassisCommData();
  }
};
void Robot::sendWheelsMotorData()
{
  WheelMotorIdx motor_idx[4] = {
      kWheelMotorIdxLeftFront,   ///< 左前轮电机下标
      kWheelMotorIdxLeftRear,    ///< 左后轮电机下标
      kWheelMotorIdxRightRear,   ///< 右后轮电机下标
      kWheelMotorIdxRightFront,  ///< 右前轮电机下标
  };
  TxDevIdx tx_dev_idx[4] = {
      TxDevIdx::kMotorWheelLeftFront,   ///< 左前轮电机下标
      TxDevIdx::kMotorWheelLeftRear,    ///< 左后轮电机下标
      TxDevIdx::kMotorWheelRightRear,   ///< 右后轮电机下标
      TxDevIdx::kMotorWheelRightFront,  ///< 右前轮电机下标
  };
  Motor *dev_ptr = nullptr;
  for (size_t i = 0; i < 4; i++) {
    WheelMotorIdx midx = motor_idx[i];
    dev_ptr = motor_wheels_ptr_[midx];
    HW_ASSERT(dev_ptr != nullptr, "Motor pointer %d is null", midx);
    if (dev_ptr == nullptr) {
      continue;
    }
    tx_dev_mgr_pairs_[(uint32_t)tx_dev_idx[i]].setTransmitterNeedToTransmit();
  }
};
void Robot::sendCapData()
{
  HW_ASSERT(cap_ptr_ != nullptr, "Cap pointer is null", cap_ptr_);
  if (cap_ptr_ == nullptr) {
    return;
  }
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kCap].setTransmitterNeedToTransmit();
};
void Robot::sendGimbalChassisCommData()
{
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null", gc_comm_ptr_);
  if (gc_comm_ptr_ == nullptr) {
    return;
  }
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kGimbalChassis].setTransmitterNeedToTransmit();
};
void Robot::sendUsartData()
{
  if (work_tick_ % 5 == 0) {
    sendRefereeData();
  }
};
uint32_t debug_tick[3] = {0};
void Robot::sendRefereeData()
{
  debug_tick[0]++;
  if (ui_drawer_.encode(rfr_tx_data_, rfr_tx_data_len_)) {
    HAL_UART_Transmit_DMA(&huart6, rfr_tx_data_, rfr_tx_data_len_);
    debug_tick[1] ++;
  }
  // tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kReferee].setTransmitterNeedToTransmit();
};
#pragma endregion

#pragma region 注册函数

void Robot::registerChassis(Chassis *ptr)
{
  HW_ASSERT(ptr != nullptr, "Chassis pointer is null", ptr);
  chassis_ptr_ = ptr;
};
void Robot::registerFeed(Feed *ptr)
{
  HW_ASSERT(ptr != nullptr, "Feed pointer is null", ptr);
  feed_ptr_ = ptr;
};
void Robot::registerGimbal(Gimbal *ptr)
{
  HW_ASSERT(ptr != nullptr, "Gimbal pointer is null", ptr);
  gimbal_ptr_ = ptr;
};
void Robot::registerShooter(Shooter *ptr)
{
  HW_ASSERT(ptr != nullptr, "Shooter pointer is null", ptr);
  shooter_ptr_ = ptr;
};
void Robot::registerBuzzer(Buzzer *ptr)
{
  HW_ASSERT(ptr != nullptr, "Buzzer pointer is null", ptr);
  buzzer_ptr_ = ptr;
};
void Robot::registerImu(Imu *ptr)
{
  HW_ASSERT(ptr != nullptr, "IMU pointer is null", ptr);
  imu_ptr_ = ptr;
};

void Robot::registerMotorWheels(Motor *dev_ptr, uint8_t idx, CanTxMgr *tx_mgr_ptr)
{
  HW_ASSERT(dev_ptr != nullptr, "Motor pointer is null", dev_ptr);
  HW_ASSERT(idx >= 0 && idx < kWheelMotorNum, "Motor index is out of range", idx);
  if (dev_ptr == nullptr || idx >= kWheelMotorNum || motor_wheels_ptr_[idx] == dev_ptr || tx_mgr_ptr == nullptr) {
    return;
  }
  WheelMotorIdx midx[4] = {
      WheelMotorIdx::kWheelMotorIdxLeftFront,
      WheelMotorIdx::kWheelMotorIdxLeftRear,
      WheelMotorIdx::kWheelMotorIdxRightRear,
      WheelMotorIdx::kWheelMotorIdxRightFront,
  };
  TxDevIdx tidx[4] = {
      TxDevIdx::kMotorWheelLeftFront,
      TxDevIdx::kMotorWheelLeftRear,
      TxDevIdx::kMotorWheelRightRear,
      TxDevIdx::kMotorWheelRightFront,
  };
  motor_wheels_ptr_[midx[idx]] = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)tidx[idx]].transmitter_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)tidx[idx]].tx_mgr_ptr_ = tx_mgr_ptr;
};
void Robot::registerCap(Cap *dev_ptr, CanTxMgr *tx_mgr_ptr)
{
  HW_ASSERT(dev_ptr != nullptr, "Cap pointer is null", dev_ptr);
  HW_ASSERT(tx_mgr_ptr != nullptr, "CanTxMgr pointer is null", tx_mgr_ptr);
  if (dev_ptr == nullptr || cap_ptr_ == dev_ptr || tx_mgr_ptr == nullptr) {
    return;
  }

  cap_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kCap].transmitter_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kCap].tx_mgr_ptr_ = tx_mgr_ptr;
};
void Robot::registerGimbalChassisComm(GimbalChassisComm *dev_ptr, CanTxMgr *tx_mgr_ptr)
{
  HW_ASSERT(dev_ptr != nullptr, "GimbalChassisComm pointer is null", dev_ptr);
  HW_ASSERT(tx_mgr_ptr != nullptr, "CanTxMgr pointer is null", tx_mgr_ptr);
  if (dev_ptr == nullptr || gc_comm_ptr_ == dev_ptr || tx_mgr_ptr == nullptr) {
    return;
  }

  gc_comm_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kGimbalChassis].transmitter_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kGimbalChassis].tx_mgr_ptr_ = tx_mgr_ptr;
};

void Robot::registerReferee(Referee *dev_ptr, UartTxMgr *tx_mgr_ptr)
{
  HW_ASSERT(dev_ptr != nullptr, "Referee pointer is null", dev_ptr);
  HW_ASSERT(tx_mgr_ptr != nullptr, "UartRxMgr pointer is null", tx_mgr_ptr);
  if (dev_ptr == nullptr || referee_ptr_ == dev_ptr || tx_mgr_ptr == nullptr) {
    return;
  }

  // auto iter = transmitter_ptr_switch_map_.find(referee_ptr_);
  // if (iter != transmitter_ptr_switch_map_.end()) {
  //   transmitter_ptr_switch_map_.erase(iter);
  // }
  referee_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kReferee].transmitter_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kReferee].tx_mgr_ptr_ = tx_mgr_ptr;
};

void Robot::registerRc(DT7 *ptr)
{
  HW_ASSERT(ptr != nullptr, "DT7 pointer is null", ptr);
  if (ptr == nullptr || rc_ptr_ == ptr) {
    return;
  }
  rc_ptr_ = ptr;
}

void Robot::registerPerformancePkg(PerformancePkg *ptr)
{
  HW_ASSERT(ptr != nullptr, "PerformancePkg pointer is null", ptr);
  if (ptr == nullptr || rfr_performance_pkg_ptr_ == ptr) {
    return;
  }

  rfr_performance_pkg_ptr_ = ptr;
};
void Robot::registerPowerHeatPkg(PowerHeatPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, "PowerHeatPkg pointer is null", ptr);
  if (ptr == nullptr || rfr_power_heat_pkg_ptr_ == ptr) {
    return;
  }
  rfr_power_heat_pkg_ptr_ = ptr;
};
void Robot::registerShooterPkg(ShooterPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, "ShooterPkg pointer is null", ptr);
  if (ptr == nullptr || rfr_shooter_pkg_ptr_ == ptr) {
    return;
  }
  rfr_shooter_pkg_ptr_ = ptr;
};
#pragma endregion
/* Private function definitions ----------------------------------------------*/
}  // namespace robot