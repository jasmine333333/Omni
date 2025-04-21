/** 
 *******************************************************************************
 * @file      :robot.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_MODULE_ROBOT_HPP_
#define ROBOT_MODULE_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/

#include "buzzer.hpp"
#include "can_tx_mgr.hpp"
#include "fsm.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "feed.hpp"
#include "fric_2motor.hpp"
#include "motor.hpp"
#include "tick.hpp"
#include "transmitter.hpp"
#include "tx_mgr.hpp"
#include "uart_tx_mgr.hpp"
#include "imu.hpp"
#include "vision.hpp"
#include "laser.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Robot : public Fsm
{
 public:
  typedef hello_world::buzzer::Buzzer Buzzer;
  typedef hello_world::motor::Motor Motor;
  typedef hello_world::comm::Transmitter Transmitter;
  typedef hello_world::comm::CanTxMgr CanTxMgr;
  typedef hello_world::comm::UartTxMgr UartTxMgr;
  typedef hello_world::comm::TxMgr TxMgr;
  typedef hello_world::vision::Vision Vision;
  typedef hello_world::module::Feed Feed;
  typedef hello_world::module::Fric Fric;
  typedef hello_world::laser::Laser Laser;

  typedef robot::Gimbal Gimbal;

  typedef robot::GimbalChassisComm GimbalChassisComm;
  typedef hello_world::imu::Imu Imu;

 public:
  Robot() {};
  ~Robot() {};

  // 状态机主要接口函数
  void update() override;

  void run() override;

  void reset() override;

  void standby() override;

  void registerGimbal(Gimbal *ptr);
  void registerFeed(Feed *ptr);
  void registerFric(Fric *ptr);

  void registerBuzzer(Buzzer *ptr);
  void registerImu(Imu *ptr);
  void registerFeedMotor(Motor *dev_ptr);
  void registerFricMotor(Motor *dev_ptr, uint8_t index);
  void registerGimbalMotor(Motor *dev_ptr, uint8_t index);
  void registerGimbalChassisComm(GimbalChassisComm *dev_ptr);
  void registerVision(Vision *dev_ptr);
  void registerLaser(Laser *ptr);

 private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateImuData();
  void updateGimbalChassisCommData();
  void updateVisionData();

  void updatePwrState();

  // 各工作状态任务执行函数
  void runOnDead();
  void runOnResurrection();
  void runOnWorking();

  void transmitFricStatus();
  void genModulesCmd();

  // 设置通讯组件数据函数
  void setCommData();
  void setGimbalChassisCommData();
  void setVisionCommData();

  void sendCommData();
  void sendCanData();
  void sendFricsMotorData();
  void sendFeedMotorData();
  void sendGimbalMotorData();
  void sendGimbalChassisCommData();
  void sendUsartData();
  void sendVisionData();

  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();

  // IMU 数据在 update 函数中更新
  bool is_imu_caled_offset_ = false;  ///< IMU 数据是否计算完零飘

  uint8_t last_is_new_bullet_shot_ = 0;

  // 主要模块状态机组件指针
  Gimbal *gimbal_ptr_ = nullptr;    ///< 云台模块指针
  Feed *feed_ptr_ = nullptr;        ///< 拨盘模块指针
  Fric *fric_ptr_ = nullptr;        ///< 摩擦轮模块指针
  
  // 无通信功能的组件指针
  Buzzer *buzzer_ptr_ = nullptr;  ///< 蜂鸣器指针
  Imu *imu_ptr_ = nullptr;        ///< IMU 指针
  Laser *laser_ptr_ = nullptr;    ///< 红点激光指针

  // 电机指针
  Motor *feed_motor_ptr_ = nullptr;  ///< 电机指针
  Motor *fric_motor_ptr_[2] = {nullptr};  ///< 电机指针
  Motor *gimbal_motor_ptr_[2] = {nullptr};  ///< 电机指针

  // 收发数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr;  ///< 云台底盘通信模块指针 收发数据
  Vision *vision_ptr_ = nullptr;              ///< 视觉模块指针 收发数据

};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* ROBOT_MODULE_ROBOT_HPP_ */
