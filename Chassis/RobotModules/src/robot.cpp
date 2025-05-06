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

      .robot_level = 0,  ///< 机器人等级
      .current_hp = 200, ///< 机器人当前血量
      .maximum_hp = 200, ///< 机器人血量上限

      .shooter_barrel_cooling_value = 40, ///< 机器人枪口热量每秒冷却值
      .shooter_barrel_heat_limit = 100,   ///< 机器人枪口热量上限

      .chassis_power_limit = 55, ///< 底盘功率限制，若不选择底盘或发射机构类型，则在七分钟比赛阶段开始后，未选择的底盘性能类型将被默认选择为“血量优先”

      .power_management_gimbal_output = 1,  ///< 电源管理模块 gimbal 口输出：0-关闭，1-开启
      .power_management_chassis_output = 1, ///< 电源管理模块 chassis 口输出：0-关闭，1-开启
      .power_management_shooter_output = 1, ///< 电源管理模块 shooter 口输出：0-关闭，1-开启
  };
  const hello_world::referee::RobotPowerHeatData kDefaultRobotPowerHeatData = {
      .buffer_energy = 60, ///< 缓冲能量，单位：J

      .shooter_17mm_1_barrel_heat = 0, ///< 第一个17mm发射机构的枪口热量
      .shooter_17mm_2_barrel_heat = 0, ///< 第二个17mm发射机构的枪口热量
      .shooter_42mm_barrel_heat = 0,   ///< 42mm发射机构的枪口热量
  };
  const hello_world::referee::CompRobotsHpData kDefaultRobotCompRobotsHpData = {};
  const hello_world::referee::RobotHurtData kDefaultRobotHurtData = {
      .module_id = 0, ///< 模块ID
      .hp_deduction_reason = static_cast<uint8_t>(
          hello_world::referee::HpDeductionReason::
              kArmorHit), ///< 扣血原因
  };

  const hello_world::referee::RobotShooterData kDefaultRobotShooterData = {
      .bullet_type = static_cast<uint8_t>(hello_world::referee::BulletType::k17mm), ///< 弹丸类型，@see BulletType

      .shooter_id = static_cast<uint8_t>(hello_world::referee::ShooterId::k17mm1), ///< 发射机构ID，@see ShooterId

      .launching_frequency = 0, ///< 弹丸射频，单位：Hz
      .bullet_speed = 23.7f,    ///< 弹丸初速度，单位：m/s
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
    if (imu_ptr_->isOffsetCalcFinished())
    {
      is_imu_caled_offset_ = true;
    }
  };

  void Robot::updateRcData()
  {
    HW_ASSERT(rc_ptr_ != nullptr, "RC pointer is null", rc_ptr_);
    if (manual_ctrl_src_ == ManualCtrlSrc::Rc)
    {
      if (rc_ptr_->isUsingKeyboardMouse())
      {
        setManualCtrlSrc(ManualCtrlSrc::Kb);
      }
    }
    else if (manual_ctrl_src_ == ManualCtrlSrc::Kb)
    {
      if (rc_ptr_->isRcSwitchChanged())
      { // 不检测摇杆变化，防止控制源来回切换
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
    CompRobotsHpPkg::Data rchp_data = kDefaultRobotCompRobotsHpData;
    RobotHurtPkg::Data rht_data = kDefaultRobotHurtData; // 初始化受伤数据为默认值

    GimbalChassisComm::RefereeData::ChassisPart &gimbal_rfr_data = gc_comm_ptr_->referee_data().cp;

    if (!referee_ptr_->isOffline())
    {
      rpp_data = rfr_performance_pkg_ptr_->getData();
      rph_data = rfr_power_heat_pkg_ptr_->getData();
      rsp_data = rfr_shooter_pkg_ptr_->getData();
      rchp_data = rfr_comp_robots_hp_pkg_ptr_->getData();
      rht_data = rfr_robot_hurt_pkg_ptr_->getData();
      if (!rfr_shooter_pkg_ptr_->isHandled())
      { // 检测到了一颗新的弹丸发射
        rfr_shooter_pkg_ptr_->setHandled();
        gimbal_rfr_data.is_new_bullet_shot++;
        bullet_num_++;
        if (gimbal_rfr_data.is_new_bullet_shot > 3)
        {
          gimbal_rfr_data.is_new_bullet_shot = 0;
        }
      }
    }

    if (!referee_ptr_->isOffline())
    {
      robot_id_ = (RobotId)rpp_data.robot_id;
    }
    else
    {
      robot_id_ = RobotId::kRedStandard3;
    }
    ui_drawer_.setSenderId(robot_id_);

    // 判断基地是否被攻击
    if (last_base_hp > base_hp)
    {
      base_attack_flag = true;
      last_base_attack_tick = work_tick_;
    }
    else
    {
      if (work_tick_ - last_base_attack_tick > 2000)
      {
        base_attack_flag = false;
      }
    }
    last_base_hp = base_hp;

    hurt_module_id = rht_data.module_id;
    hurt_reason = rht_data.hp_deduction_reason;

    chassis_rfr_data.is_rfr_on = (!referee_ptr_->isOffline());
    chassis_rfr_data.is_pwr_on = rpp_data.power_management_chassis_output;
    chassis_rfr_data.pwr_limit = rpp_data.chassis_power_limit;
    chassis_rfr_data.pwr_buffer = rph_data.buffer_energy;
    chassis_rfr_data.current_hp = rpp_data.current_hp;
    // chassis_rfr_data.current_hp = 0;
    chassis_ptr_->setRfrData(chassis_rfr_data);

    // 判断机器人是否掉血
    if (chassis_rfr_data.current_hp < last_hp)
    {
      robot_attacked_flag = true;
      last_robot_attacked_tick = work_tick_;
    }
    else
    {
      if (work_tick_ - last_robot_attacked_tick > 500)
      {
        robot_attacked_flag = false;
      }
    }
    // chassis_rfr_data.last_hp = chassis_rfr_data.current_hp;
    last_hp = chassis_rfr_data.current_hp;

    gimbal_rfr_data.is_rfr_on = (!referee_ptr_->isOffline());
    gimbal_rfr_data.is_rfr_shooter_power_on = rpp_data.power_management_shooter_output;
    gimbal_rfr_data.is_rfr_gimbal_power_on = rpp_data.power_management_gimbal_output;
    gimbal_rfr_data.robot_id = (RobotId)rpp_data.robot_id;

    // TODO: 换成实际的枪口
    gimbal_rfr_data.bullet_speed = rsp_data.bullet_speed;
    gimbal_rfr_data.shooter_heat = rph_data.shooter_17mm_1_barrel_heat;
    gimbal_rfr_data.shooter_cooling = rpp_data.shooter_barrel_cooling_value;
    gimbal_rfr_data.shooter_heat_limit = rpp_data.shooter_barrel_heat_limit;
  };

  void Robot::updatePwrState()
  {
    PwrState pre_state = pwr_state_;
    PwrState next_state = pre_state;
    if (pre_state == PwrState::Dead)
    {
      // 主控板程序在跑就意味着有电，所以直接从死亡状态进入复活状态
      next_state = PwrState::Resurrection;
    }
    else if (pre_state == PwrState::Resurrection)
    {
      if (is_imu_caled_offset_ && work_tick_ > 1000)
      {
        next_state = PwrState::Working;
      }
    }
    else if (pre_state == PwrState::Working)
    {
      // 工作状态下，保持当前状态
    }
    else
    {
      // 其他状态，认为是死亡状态
      next_state = PwrState::Dead;
    }
    setPwrState(next_state);
  };

#pragma endregion

#pragma region 执行任务

  void Robot::run()
  {
    if (pwr_state_ == PwrState::Dead)
    {
      runOnDead();
    }
    else if (pwr_state_ == PwrState::Resurrection)
    {
      runOnResurrection();
    }
    else if (pwr_state_ == PwrState::Working)
    {
      runOnWorking();
    }
    else
    {
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
    if (buzzer_ptr_ != nullptr)
    {
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
    if (manual_ctrl_src_ == ManualCtrlSrc::Rc)
    {
      genModulesCmdFromRc();
    }
    else if (manual_ctrl_src_ == ManualCtrlSrc::Kb)
    {
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
    else
    {
      if (l_switch == RcSwitchState::kUp)
      {
        // * 左上
        chassis_working_mode = Chassis::WorkingMode::Depart;
        gimbal_working_mode = Gimbal::WorkingMode::Normal;
        if (r_switch == RcSwitchState::kUp)
        {
          gimbal_ctrl_mode = CtrlMode::Auto;
          shooter_working_mode = Shooter::WorkingMode::Normal;
          feed_ctrl_mode = CtrlMode::Manual;
          shoot_flag = (rc_wheel > 0.9f);
          // * 左上右上
        }
        else if (r_switch == RcSwitchState::kMid)
        {
          gimbal_ctrl_mode = CtrlMode::Auto;
          feed_ctrl_mode = CtrlMode::Auto;
          shooter_working_mode = Shooter::WorkingMode::Normal;
          shoot_flag = (rc_wheel > 0.9f);
          // * 左上右中
        }
        else if (r_switch == RcSwitchState::kDown)
        {
          // * 左上右下
        }
      }
      else if (l_switch == RcSwitchState::kMid)
      {
        // * 左中
        chassis_working_mode = Chassis::WorkingMode::Follow;
        gimbal_working_mode = Gimbal::WorkingMode::Normal;
        gimbal_ctrl_mode = CtrlMode::Manual;
        feed_ctrl_mode = CtrlMode::Manual;
        if (r_switch == RcSwitchState::kUp)
        {
          // * 左中右上
          shooter_working_mode = Shooter::WorkingMode::Stop;
        }
        else if (r_switch == RcSwitchState::kMid)
        {
          // * 左中右中
          use_cap_flag = true;
          shooter_working_mode = Shooter::WorkingMode::Normal;
          shoot_flag = (rc_wheel > 0.9f);
        }
        else if (r_switch == RcSwitchState::kDown)
        {
          // * 左中右下
        }
      }
      else if (l_switch == RcSwitchState::kDown)
      {
        // * 左下
        chassis_working_mode = Chassis::WorkingMode::Gyro;
        gimbal_working_mode = Gimbal::WorkingMode::Normal;
        gimbal_ctrl_mode = CtrlMode::Manual;
        shooter_working_mode = Shooter::WorkingMode::Stop;
        if (r_switch == RcSwitchState::kUp)
        {
          // * 左下右上
          use_cap_flag = true;
          // gyro_dir = Chassis::GyroDir::Clockwise;
        }
        else if (r_switch == RcSwitchState::kMid)
        {
          // * 左下右中
          // gyro_dir = Chassis::GyroDir::AntiClockwise;
        }
        else if (r_switch == RcSwitchState::kDown)
        {
          // * 左下右下
          // gyro_dir = Chassis::GyroDir::AntiClockwise;
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
    gimbal_cmd.yaw = hello_world::Bound(-rc_ptr_->rc_lh(), -1, 1); // 右手系，z轴竖直向上，左转为正
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
    bool auto_shoot_flag = false;
    CtrlMode feed_ctrl_mode = CtrlMode::Manual;
    Shooter::WorkingMode shooter_working_mode = Shooter::WorkingMode::Normal;
    Feed::WorkingMode feed_working_mode = Feed::WorkingMode::Normal;
    bool shoot_flag = false;

    // 检测Z是否按下
    bool is_Z_pressed = rc_ptr_->key_Z();
    if (is_Z_pressed)
    {
      navigate_flag = true; // 启用巡航模式标志
    }
    else
    {
      navigate_flag = false; // 关闭巡航模式标志
    }
    if (navigate_flag == true)
    {
      chassis_working_mode = Chassis::WorkingMode::Gyro; // 设置底盘为小陀螺模式
      gimbal_ctrl_mode = CtrlMode::Auto;
      feed_ctrl_mode = CtrlMode::Auto;
    }
    else if (navigate_flag == false)
    {
      if (last_navigate_flag == true) // 如果巡航模式刚刚关闭
      {
        chassis_working_mode = Chassis::WorkingMode::Follow; // 设置底盘为跟随模式
        gimbal_ctrl_mode = CtrlMode::Manual;
        feed_ctrl_mode = CtrlMode::Manual;
      }
    }
    last_navigate_flag = navigate_flag; // 更新上一次巡航模式标志

    if (rc_ptr_->key_Q())
    {
      chassis_working_mode = Chassis::WorkingMode::Gyro;
    }
    else if (rc_ptr_->key_E())
    {
      chassis_working_mode = Chassis::WorkingMode::Follow;
    }
    else
    {
      if (chassis_working_mode == Chassis::WorkingMode::Depart)
      {
        chassis_working_mode = Chassis::WorkingMode::Follow;
      }
    }
    if (rc_ptr_->key_C())
    {
    }
    if (rc_ptr_->key_B() && (work_tick_ - last_rev_work_tick_ > 200))
    {
      rev_head_flag = true;
      last_rev_work_tick_ = work_tick_;
    }
    else
    {
      rev_head_flag = false;
    }

    // 打符模式
    if (rc_ptr_->key_F())
    {
      buff_mode_ = 1;
    }
    else
    {
      buff_mode_ = 0;
    }

    if (rc_ptr_->mouse_l_btn())
    {
      shoot_flag = true;
    }
    if (rc_ptr_->mouse_r_btn())
    {
      gimbal_ctrl_mode = CtrlMode::Auto;
      feed_ctrl_mode = CtrlMode::Manual;
    }

    bool is_X_pressed = rc_ptr_->key_X();
    if (is_X_pressed)
    {
      gimbal_ctrl_mode = CtrlMode::Auto;
      feed_ctrl_mode = CtrlMode::Auto;
    }

    GimbalCmd gimbal_cmd;
    gimbal_cmd.pitch = hello_world::Bound(0.01 * rc_ptr_->mouse_y(), -1, 1); // 去掉了负号，配合最新裁判端
    gimbal_cmd.yaw = hello_world::Bound(-0.01 * rc_ptr_->mouse_x(), -1, 1);
    gimbal_ptr_->setNormCmdDelta(gimbal_cmd);
    gimbal_ptr_->setCtrlMode(gimbal_ctrl_mode);
    gimbal_ptr_->setWorkingMode(gimbal_working_mode);
    gimbal_ptr_->setRevHeadFlag(rev_head_flag);
    gimbal_ptr_->setnavigateFlag(navigate_flag);

    ChassisCmd chassis_cmd = {0};
    chassis_cmd.v_x = (int8_t)rc_ptr_->key_W() - (int8_t)rc_ptr_->key_S();
    chassis_cmd.v_y = (int8_t)rc_ptr_->key_A() - (int8_t)rc_ptr_->key_D();
    chassis_cmd.w = 0;
    chassis_ptr_->setNormCmd(chassis_cmd);
    chassis_ptr_->setWorkingMode(chassis_working_mode);
    chassis_ptr_->setUseCapFlag(rc_ptr_->key_SHIFT());
    if (rev_head_flag)
      chassis_ptr_->revHead();

    feed_ptr_->setCtrlMode(feed_ctrl_mode);
    feed_ptr_->setWorkingMode(feed_working_mode);
    feed_ptr_->setShootFlag(shoot_flag);

    shooter_ptr_->setWorkingMode(shooter_working_mode);
  };

#pragma endregion

#pragma region 数据重置函数
  void Robot::reset()
  {
    pwr_state_ = PwrState::Dead;      ///< 电源状态
    last_pwr_state_ = PwrState::Dead; ///< 上一电源状态

    manual_ctrl_src_ = ManualCtrlSrc::Rc;      ///< 手动控制源
    last_manual_ctrl_src_ = ManualCtrlSrc::Rc; ///< 上一手动控制源
  };
  void Robot::resetDataOnDead()
  {
    pwr_state_ = PwrState::Dead;      ///< 电源状态
    last_pwr_state_ = PwrState::Dead; ///< 上一电源状态

    manual_ctrl_src_ = ManualCtrlSrc::Rc;      ///< 手动控制源
    last_manual_ctrl_src_ = ManualCtrlSrc::Rc; ///< 上一手动控制源
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
    gimbal_data.buff_mode_flag = buff_mode_;
    gimbal_data.yaw_delta = gimbal_ptr_->getNormCmdDelta().yaw;
    gimbal_data.pitch_delta = gimbal_ptr_->getNormCmdDelta().pitch;
    gimbal_data.ctrl_mode = gimbal_ptr_->getCtrlMode();
    gimbal_data.working_mode = gimbal_ptr_->getWorkingMode();
    gimbal_data.navigation_flag = gimbal_ptr_->getnavigateFlag();

    // shooter
    GimbalChassisComm::ShooterData::ChassisPart &shooter_data = gc_comm_ptr_->shooter_data().cp;
    shooter_data.setShootFlag(feed_ptr_->getShootFlag());
    shooter_data.ctrl_mode = feed_ptr_->getCtrlMode();
    shooter_data.working_mode = shooter_ptr_->getWorkingMode();
  };

  void Robot::setUiDrawerData()
  {

    // ui_drawer_.setSenderId(RobotId::kRedStandard3);
    // Chassis
    HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null", chassis_ptr_);
    ui_drawer_.setChassisWorkState(chassis_ptr_->getPwrState());
    // ui_drawer_.setChassisCtrlMode(chassis_ptr_->);
    ui_drawer_.setChassisWorkingMode(chassis_ptr_->getWorkingMode());
    ui_drawer_.setChassisHeadDir(chassis_ptr_->getThetaI2r());

    // Gimbal
    HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal FSM pointer is null", gimbal_ptr_);
    ui_drawer_.setGimbalCtrlMode(gimbal_ptr_->getCtrlMode());
    ui_drawer_.setGimbalWorkingMode(gimbal_ptr_->getWorkingMode());
    ui_drawer_.setGimbalJointAngPitchFdb(gc_comm_ptr_->gimbal_data().gp.pitch_fdb);

    // Shooter
    HW_ASSERT(shooter_ptr_ != nullptr, "Shooter FSM pointer is null", shooter_ptr_);
    ui_drawer_.setHeat(gc_comm_ptr_->referee_data().cp.shooter_heat);
    ui_drawer_.setHeatLimit(gc_comm_ptr_->referee_data().cp.shooter_heat_limit);

    ui_drawer_.setFeedStuckFlag(gc_comm_ptr_->shooter_data().gp.feed_stuck_state);
    ui_drawer_.setFricStuckFlag(gc_comm_ptr_->shooter_data().gp.is_fric_stuck_);

    ui_drawer_.setBulletNum(bullet_num_);

    ui_drawer_.setBaseAttack(base_attack_flag);

    // navigate
    ui_drawer_.setNavigateFlag(navigate_flag);

    if (hurt_reason ==
            static_cast<uint8_t>(HurtReason::kArmorHit) ||
        hurt_reason ==
            static_cast<uint8_t>(HurtReason::kArmorCollision))
    {
      if (robot_attacked_flag == true)
      {
        ui_drawer_.setisArmorHit(true); // 受伤标志置为真
        ui_drawer_.setHurtModuleid(hurt_module_id);
      }
      else
      {
        ui_drawer_.setisArmorHit(false); // 受伤标志置为真
      }
    }
    else
    {
      ui_drawer_.setisArmorHit(false);
      ui_drawer_.setHurtModuleid(0);
    }

    // Cap
    HW_ASSERT(cap_ptr_ != nullptr, "Cap pointer is null", cap_ptr_);
    ui_drawer_.setCapPwrPercent(cap_ptr_->getRemainingPower());

    // vision
    HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null", gc_comm_ptr_);
    bool is_vision_valid = gc_comm_ptr_->vision_data().gp.is_enemy_detected;
    ui_drawer_.setVisTgtX(gc_comm_ptr_->vision_data().gp.vtm_x, is_vision_valid);
    ui_drawer_.setVisTgtY(gc_comm_ptr_->vision_data().gp.vtm_y, is_vision_valid);
    ui_drawer_.setisvisionvalid(is_vision_valid);

    // if (work_tick_ %500 == 0)
    // {
    //   ui_drawer_.refresh();
    // }

    if (rc_ptr_->key_R())
    {
      ui_drawer_.refresh();
    }

    // referee_ptr_->setTxPkg(ui_drawer_.))
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

    if (work_tick_ % 10 == 0)
    {
      sendCapData();
    }
    if (work_tick_ > 1010)
    {
      sendGimbalChassisCommData();
    }
  };
  void Robot::sendWheelsMotorData()
  {
    for (size_t i = 0; i < 4; i++)
    {
      HW_ASSERT(motor_wheels_ptr_[i] != nullptr, "Motor pointer %d is null", midx);
      motor_wheels_ptr_[i]->setNeedToTransmit();
    }
  };
  void Robot::sendCapData()
  {
    HW_ASSERT(cap_ptr_ != nullptr, "Cap pointer is null", cap_ptr_);
    cap_ptr_->setNeedToTransmit();
  };
  void Robot::sendGimbalChassisCommData()
  {
    HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null", gc_comm_ptr_);
    gc_comm_ptr_->setNeedToTransmit();
  };
  void Robot::sendUsartData()
  {
    if (work_tick_ % 5 == 0)
    {
      sendRefereeData();
    }
  };
  void Robot::sendRefereeData()
  {
    if (ui_drawer_.encode(rfr_tx_data_, rfr_tx_data_len_))
    {
      HAL_UART_Transmit_DMA(&huart6, rfr_tx_data_, rfr_tx_data_len_);
    }
    // HW_ASSERT(referee_ptr_ != nullptr, "GimbalChassisComm pointer is null", referee_ptr_);
    // if (referee_ptr_ == nullptr) {
    //   return;
    // }
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

  void Robot::registerMotorWheels(Motor *dev_ptr, uint8_t idx)
  {
    HW_ASSERT(dev_ptr != nullptr, "Motor pointer is null", dev_ptr);
    motor_wheels_ptr_[idx] = dev_ptr;
  };
  void Robot::registerCap(Cap *dev_ptr)
  {
    HW_ASSERT(dev_ptr != nullptr, "Cap pointer is null", dev_ptr);
    cap_ptr_ = dev_ptr;
  };
  void Robot::registerGimbalChassisComm(GimbalChassisComm *dev_ptr)
  {
    HW_ASSERT(dev_ptr != nullptr, "GimbalChassisComm pointer is null", dev_ptr);
    gc_comm_ptr_ = dev_ptr;
  };

  void Robot::registerReferee(Referee *dev_ptr)
  {
    HW_ASSERT(dev_ptr != nullptr, "RFR pointer is null", dev_ptr);
    referee_ptr_ = dev_ptr;
  };

  void Robot::registerRc(DT7 *ptr)
  {
    HW_ASSERT(ptr != nullptr, "DT7 pointer is null", ptr);
    rc_ptr_ = ptr;
  }

  void Robot::registerPerformancePkg(PerformancePkg *ptr)
  {
    HW_ASSERT(ptr != nullptr, "PerformancePkg pointer is null", ptr);
    if (ptr == nullptr || rfr_performance_pkg_ptr_ == ptr)
    {
      return;
    }

    rfr_performance_pkg_ptr_ = ptr;
  };
  void Robot::registerPowerHeatPkg(PowerHeatPkg *ptr)
  {
    HW_ASSERT(ptr != nullptr, "PowerHeatPkg pointer is null", ptr);
    if (ptr == nullptr || rfr_power_heat_pkg_ptr_ == ptr)
    {
      return;
    }
    rfr_power_heat_pkg_ptr_ = ptr;
  };
  void Robot::registerShooterPkg(ShooterPkg *ptr)
  {
    HW_ASSERT(ptr != nullptr, "ShooterPkg pointer is null", ptr);
    if (ptr == nullptr || rfr_shooter_pkg_ptr_ == ptr)
    {
      return;
    }
    rfr_shooter_pkg_ptr_ = ptr;
  };
  void Robot::registerCompRobotsHpPkg(CompRobotsHpPkg *ptr)
  {
    HW_ASSERT(ptr != nullptr, "CompRobotsHpPkg pointer is null", ptr);
    if (ptr == nullptr || rfr_comp_robots_hp_pkg_ptr_ == ptr)
    {
      return;
    }
    rfr_comp_robots_hp_pkg_ptr_ = ptr;
  };
  void Robot::registerRobotHurtPkg(RobotHurtPkg *ptr)
  {
    HW_ASSERT(ptr != nullptr, "RobotHurtPkg pointer is null", ptr);
    if (ptr == nullptr || rfr_robot_hurt_pkg_ptr_ == ptr)
    {
      return;
    }
    rfr_robot_hurt_pkg_ptr_ = ptr;
  };
#pragma endregion
  /* Private function definitions ----------------------------------------------*/
} // namespace robot