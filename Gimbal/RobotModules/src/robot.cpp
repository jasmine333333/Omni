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
#include "can.h"
#include "rfr_pkg/rfr_id.hpp"
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
    updateGimbalChassisCommData();
    updateVisionData(); 
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


  void Robot::updateGimbalChassisCommData()
  {
    GimbalChassisComm::RefereeData::ChassisPart &referee_data = gc_comm_ptr_->referee_data().cp;

    hello_world::module::Feed::RfrInputData feed_rfr_input_data;
    hello_world::module::Fric::RfrInputData fric_rfr_input_data;
    
    feed_rfr_input_data.is_rfr_on = referee_data.is_rfr_on;
    feed_rfr_input_data.is_power_on = referee_data.is_rfr_shooter_power_on;
    if (referee_data.is_new_bullet_shot != last_is_new_bullet_shot_){
      feed_rfr_input_data.is_new_bullet_shot = true;
      fric_rfr_input_data.is_new_bullet_shot = true;
    } else {
      feed_rfr_input_data.is_new_bullet_shot = false;
      fric_rfr_input_data.is_new_bullet_shot = false;
    }
    last_is_new_bullet_shot_ = referee_data.is_new_bullet_shot;
    feed_rfr_input_data.heat_limit = referee_data.shooter_heat_limit;
    feed_rfr_input_data.heat = referee_data.shooter_heat;
    feed_rfr_input_data.heat_cooling_ps = referee_data.shooter_cooling;
    feed_ptr_->updateRfrData(feed_rfr_input_data);
    
    fric_rfr_input_data.bullet_spd = referee_data.bullet_speed;
    fric_rfr_input_data.is_power_on = referee_data.is_rfr_shooter_power_on;
    fric_ptr_->updateRfrData(fric_rfr_input_data);

    gimbal_ptr_->updateIsRfrPwrOn(referee_data.is_rfr_gimbal_power_on);
  };

  void Robot::updateVisionData()
  {
    if (vision_ptr_->isOffline())
    {
      gimbal_ptr_->setVisionTargetDetected(false);
      feed_ptr_->setVisionShootFlag(Vision::ShootFlag::kNoShoot);
      return;
    }
    if (gimbal_ptr_->getCtrlMode() == CtrlMode::Auto && feed_ptr_->getCtrlMode() == hello_world::module::CtrlMode::kManual)
    {
      feed_ptr_->setTriggerLimit(true, true, 4, 50);
      gimbal_ptr_->setVisionTargetDetected(vision_ptr_->getIsEnemyDetected());
    }
    else if (gimbal_ptr_->getCtrlMode() == CtrlMode::Auto && feed_ptr_->getCtrlMode() == hello_world::module::CtrlMode::kAuto)
    {
      // feed_ptr_->setTriggerLimit(true, true, 4, 500);//test todelete 为打符设置的低频发弹模式
      feed_ptr_->setTriggerLimit(true, true, 4, 50);
      feed_ptr_->setVisionShootFlag(vision_ptr_->getShootFlag());
      gimbal_ptr_->setVisionTargetDetected(vision_ptr_->getIsEnemyDetected());
    }
    else
    {
      feed_ptr_->setTriggerLimit(true, true, 4, 50);
    }
  }

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
      if (is_imu_caled_offset_)
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

  void Robot::runOnDead()
  {
    laser_ptr_->disable();
    resetDataOnDead();
    standby();
  };

  void Robot::runOnResurrection()
  {
    laser_ptr_->disable();
    resetDataOnResurrection();
    standby();
  };

  void Robot::runOnWorking()
  {
    laser_ptr_->disable();
    genModulesCmd();

    HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal FSM pointer is null", gimbal_ptr_);
    gimbal_ptr_->update();
    gimbal_ptr_->run();

    HW_ASSERT(fric_ptr_ != nullptr, "Fric FSM pointer is null", fric_ptr_);
    fric_ptr_->update();
    fric_ptr_->run();

    transmitFricStatus();

    HW_ASSERT(feed_ptr_ != nullptr, "Feed FSM pointer is null", feed_ptr_);
    feed_ptr_->update();
    feed_ptr_->run();
    // feed_ptr_->setTriggerLimit(true, true, 4, 50);//todo
  };

  void Robot::standby()
  {

    HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal FSM pointer is null", gimbal_ptr_);
    gimbal_ptr_->update();
    gimbal_ptr_->standby();

    HW_ASSERT(fric_ptr_ != nullptr, "Fric FSM pointer is null", fric_ptr_);
    fric_ptr_->update();
    fric_ptr_->standby();

    HW_ASSERT(feed_ptr_ != nullptr, "Feed FSM pointer is null", feed_ptr_);
    feed_ptr_->update();
    feed_ptr_->standby();
  }

  uint32_t mode_cnt[3] = {0};
  void Robot::genModulesCmd()
  {
    HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null", gc_comm_ptr_);

    GimbalChassisComm::MainBoardData::ChassisPart &main_board_data = gc_comm_ptr_->main_board_data().cp;
    GimbalChassisComm::GimbalData::ChassisPart &gimbal_data = gc_comm_ptr_->gimbal_data().cp;
    GimbalChassisComm::ShooterData::ChassisPart &shooter_data = gc_comm_ptr_->shooter_data().cp;
    GimbalChassisComm::RefereeData::ChassisPart &referee_data = gc_comm_ptr_->referee_data().cp;

    // gimbal
    CtrlMode gimbal_ctrl_mode = gimbal_data.ctrl_mode;
    if (gimbal_ctrl_mode == CtrlMode::Auto) {
      if (!(vision_ptr_->isDetectedInView()) || !(vision_ptr_->getIsEnemyDetected())) {
        gimbal_ctrl_mode = CtrlMode::Manual;
      }
    }

    if (gimbal_ctrl_mode == CtrlMode::Manual)
    {
      laser_ptr_->enable();
      gimbal_ptr_->setCtrlMode(CtrlMode::Manual);
      gimbal_ptr_->setRevHeadFlag(gimbal_data.turn_back_flag);
      gimbal_ptr_->setNavigationFlag(gimbal_data.navigation_flag);
      gimbal_ptr_->setBuffMode(gimbal_data.buff_mode_flag);
      gimbal_ptr_->setNormCmdDelta(gimbal_data.yaw_delta, gimbal_data.pitch_delta);
    }
    else if (gimbal_ctrl_mode == CtrlMode::Auto)
    {
      laser_ptr_->disable();
      gimbal_ptr_->setCtrlMode(CtrlMode::Auto);
      gimbal_ptr_->setVisionCmd(vision_ptr_->getPoseRefYaw(), vision_ptr_->getPoseRefPitch());
      // gimbal_ptr_->setNormCmdDelta(gimbal_data.yaw_delta, gimbal_data.pitch_delta);
    }
    gimbal_ptr_->setWorkingMode(gimbal_data.working_mode);

    feed_ptr_->setManualShootFlag(shooter_data.shoot_flag(true));

    if (shooter_data.ctrl_mode == CtrlMode::Manual)
    {
      feed_ptr_->setCtrlMode(hello_world::module::CtrlMode::kManual);
    }
    else
    {
      feed_ptr_->setCtrlMode(hello_world::module::CtrlMode::kAuto);
    }

    if (shooter_data.working_mode == ShooterWorkingMode::Normal)
    {
      fric_ptr_->setWorkingMode(hello_world::module::Fric::WorkingMode::kShoot);
    }
    else if (shooter_data.working_mode == ShooterWorkingMode::Stop)
    {
      fric_ptr_->setWorkingMode(hello_world::module::Fric::WorkingMode::kStop);
    }
    
    if (gimbal_ptr_->getBuffMode() == true)
    {
      feed_ptr_->setTriggerLimit(true, true, 4, 500);
    }
    else
    {
      feed_ptr_->setTriggerLimit(true, true, 4, 50);
    }
     
  };

  void Robot::transmitFricStatus()
  {
    feed_ptr_->setFricStatus(fric_ptr_->getStatus());
  };
#pragma endregion

#pragma region 数据重置函数
  void Robot::reset()
  {
    pwr_state_ = PwrState::Dead;      ///< 电源状态
    last_pwr_state_ = PwrState::Dead; ///< 上一电源状态
    last_is_new_bullet_shot_ = 0;
  };
  void Robot::resetDataOnDead()
  {
    pwr_state_ = PwrState::Dead;      ///< 电源状态
    last_pwr_state_ = PwrState::Dead; ///< 上一电源状态
    last_is_new_bullet_shot_ = 0;
  };
  void Robot::resetDataOnResurrection() {};

#pragma endregion

#pragma region 通信数据设置函数
  void Robot::setCommData()
  {
    // TODO(ZSC): 可能以后会在这里分工作状态
    setGimbalChassisCommData();
    setVisionCommData();
    // TODO(ZSC): 可能的其他通讯数据设置函数
    // 其他通讯模块的数据由各个子模块负责设置
    // 主控板非工作模式时，这些数据保持默认值
  };

  void Robot::setVisionCommData()
  {
    HW_ASSERT(vision_ptr_ != nullptr, "Vision pointer is null", vision_ptr_);
    HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal pointer is null", gimbal_ptr_);

    Vision::WorkState vision_work_State = Vision::WorkState::kStandby;
    if (gimbal_ptr_->getCtrlMode() == CtrlMode::Auto || feed_ptr_->getCtrlMode() == hello_world::module::CtrlMode::kAuto)
    {
      vision_work_State = Vision::WorkState::kNormal;
    }

    GimbalChassisComm::RefereeData::ChassisPart &referee_data = gc_comm_ptr_->referee_data().cp;
    if (referee_data.bullet_speed < 15)
    {
      vision_ptr_->setBulletSpeed(vision_ptr_->getDefaultBulletSpeed());
    }
    else
    {
      vision_ptr_->setBulletSpeed(referee_data.bullet_speed);
    }

    hello_world::referee::ids::RobotId robot_id = gc_comm_ptr_->referee_data().cp.robot_id;
    hello_world::referee::RfrId rfr_id = static_cast<hello_world::referee::RfrId>(robot_id);

    if (hello_world::referee::ids::GetTeamColor(rfr_id) == hello_world::referee::ids::TeamColor::kRed)
    {
      vision_ptr_->setTargetColor(Vision::TargetColor::kBlue);
    }
    else if (hello_world::referee::ids::GetTeamColor(rfr_id) == hello_world::referee::ids::TeamColor::kBlue)
    {
      vision_ptr_->setTargetColor(Vision::TargetColor::kRed);
    }
    else
    {
      vision_ptr_->setTargetColor(Vision::TargetColor::kPurple);
    }

    vision_ptr_->setPose(gimbal_ptr_->getJointRollAngFdb(), gimbal_ptr_->getJointPitchAngFdb(), gimbal_ptr_->getJointYawAngFdb());

    vision_ptr_->setWorkState(vision_work_State);
  }

  void Robot::setGimbalChassisCommData()
  {
    HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null", gc_comm_ptr_);
    HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal pointer is null", gimbal_ptr_);

    HW_ASSERT(feed_ptr_ != nullptr, "Feed pointer is null", feed_ptr_);
    HW_ASSERT(fric_ptr_ != nullptr, "Fric pointer is null", fric_ptr_);

    // main board
    GimbalChassisComm::MainBoardData::GimbalPart &main_board_data = gc_comm_ptr_->main_board_data().gp;
    main_board_data.is_gimbal_imu_ready = is_imu_caled_offset_;

    // gimbal
    GimbalChassisComm::GimbalData::GimbalPart &gimbal_data = gc_comm_ptr_->gimbal_data().gp;
    gc_comm_ptr_->vision_data().gp.is_enemy_detected = vision_ptr_->getIsEnemyDetected();
    gc_comm_ptr_->vision_data().gp.vtm_x = vision_ptr_->getVtmX();
    gc_comm_ptr_->vision_data().gp.vtm_y = vision_ptr_->getVtmY();
    // gc_comm_ptr_->gimbal_data().gp.pitch_fdb = gimbal_ptr_->getJointPitchAngFdb();
    gc_comm_ptr_->gimbal_data().gp.pitch_fdb = gimbal_ptr_->getJointPitchAngFdb();

    // shooter
    GimbalChassisComm::ShooterData::GimbalPart &shooter_data = gc_comm_ptr_->shooter_data().gp;

    //vision

  };

  void Robot::sendCommData()
  {
    sendCanData();
    sendUsartData();
  };
  void Robot::sendCanData()
  {
    sendFricsMotorData();
    sendGimbalChassisCommData();
    sendGimbalMotorData();
    sendFeedMotorData();
  };
  void Robot::sendFricsMotorData()
  {
    for (size_t i = 0; i < 2; i++)
    {
      HW_ASSERT(fric_motor_ptr_[i] != nullptr, "Motor pointer is null", midx);
      fric_motor_ptr_[i]->setNeedToTransmit();
    }
  };

  void Robot::sendFeedMotorData()
  {
    HW_ASSERT(feed_motor_ptr_ != nullptr, "Motor pointer is null", motor_ptr_[MotorIdx::kMotorIdxFeed]);
    feed_motor_ptr_->setNeedToTransmit();
  }
  void Robot::sendGimbalMotorData()
  {
    for (size_t i = 0; i < 2; i++)
    {
      HW_ASSERT(gimbal_motor_ptr_[i] != nullptr, "Motor pointer is null", motor_ptr_[motor_idx[i]]);
      gimbal_motor_ptr_[i]->setNeedToTransmit();
    }
  };
  void Robot::sendGimbalChassisCommData()
  {
    gc_comm_ptr_->setNeedToTransmit();  
  };
  void Robot::sendUsartData()
  {
    if (work_tick_ % 100)
    {
      sendVisionData();
    }
  };
  void Robot::sendVisionData()
  {
    vision_ptr_->setNeedToTransmit();
  };
#pragma endregion

#pragma region 注册函数

  void Robot::registerGimbal(Gimbal *ptr)
  {
    HW_ASSERT(ptr != nullptr, "Gimbal pointer is null", ptr);
    gimbal_ptr_ = ptr;
  };
  void Robot::registerFeed(Feed *ptr)
  {
    HW_ASSERT(ptr != nullptr, "Feed pointer is null", ptr);
    feed_ptr_ = ptr;
  };
  void Robot::registerFric(Fric *ptr)
  {
    HW_ASSERT(ptr != nullptr, "Fric pointer is null", ptr);
    fric_ptr_ = ptr;
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
  
  void Robot::registerFeedMotor(Motor *dev_ptr)
  {
    HW_ASSERT(dev_ptr != nullptr, "Motor pointer is null", dev_ptr);
    feed_motor_ptr_ = dev_ptr;
  };
  void Robot::registerFricMotor(Motor *dev_ptr,uint8_t index)
  {
    HW_ASSERT(dev_ptr != nullptr, "Motor pointer is null", dev_ptr);
    HW_ASSERT(index < 2, "Motor index out of range", dev_ptr);
    fric_motor_ptr_[index] = dev_ptr;
  };
  void Robot::registerGimbalMotor(Motor *dev_ptr,uint8_t index)
  {
    HW_ASSERT(dev_ptr != nullptr, "Motor pointer is null", dev_ptr);
    HW_ASSERT(index < 2, "Motor index out of range", dev_ptr);
    gimbal_motor_ptr_[index] = dev_ptr;
  };
  void Robot::registerGimbalChassisComm(GimbalChassisComm *dev_ptr)
  {
    HW_ASSERT(dev_ptr != nullptr, "GimbalChassisComm pointer is null", dev_ptr);
    gc_comm_ptr_ = dev_ptr;
  };

  void Robot::registerVision(Vision *dev_ptr)
  {
    HW_ASSERT(dev_ptr != nullptr, "pointer to Vision is nullptr", dev_ptr);
    vision_ptr_ = dev_ptr;
  };
  void Robot::registerLaser(Laser *ptr)
  {
    HW_ASSERT(ptr != nullptr, "pointer to laser is nullptr", ptr);
    if (ptr == nullptr) {
      return;
    }
    laser_ptr_ = ptr;
  }

#pragma endregion
  /* Private function definitions ----------------------------------------------*/
} // namespace robot