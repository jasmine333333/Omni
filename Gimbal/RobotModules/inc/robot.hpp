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
#include "usr_imu.hpp"
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
  typedef robot::Imu Imu;

  class TxDevMgrPair
  {
   public:
    TxDevMgrPair(Transmitter *transmitter_ptr = nullptr, TxMgr *tx_mgr_ptr = nullptr) : transmitter_ptr_(transmitter_ptr), tx_mgr_ptr_(tx_mgr_ptr) {};
    TxDevMgrPair(const TxDevMgrPair &other) = default;
    TxDevMgrPair &operator=(const TxDevMgrPair &other) = default;
    TxDevMgrPair(TxDevMgrPair &&other) = default;
    TxDevMgrPair &operator=(TxDevMgrPair &&other) = default;

    ~TxDevMgrPair() = default;

    void setTransmitterNeedToTransmit(void)
    {
      if (transmitter_ptr_ == nullptr || tx_mgr_ptr_ == nullptr) {
        return;
      }

      tx_mgr_ptr_->setTransmitterNeedToTransmit(transmitter_ptr_);
    }

   private:
    friend class Robot;
    Transmitter *transmitter_ptr_ = nullptr;
    TxMgr *tx_mgr_ptr_ = nullptr;
  };

  enum class TxDevIdx : uint8_t {
    kGimbalChassis,   ///< 云台与底盘通信设备下标
    kMotorFricLeft,   ///< 左摩擦轮电机通信设备下标
    kMotorFricRight,  ///< 右摩擦轮电机通信设备下标
    kMotorFeed,       ///< 拨盘电机通信设备下标
    kMotorYaw,        ///< YAW 轴电机通信设备下标
    kMotorPitch,      ///< PITCH 轴电机通信设备下标
    kVision,          ///< 视觉通信设备下标
    kNum,             ///< 通信数量
  };

  enum MotorIdx : uint8_t {
    kMotorIdxFricLeft,   ///< 左摩擦轮电机下标
    kMotorIdxFricRight,  ///< 右摩擦轮电机下标
    kMotorIdxFeed,       ///< 拨盘电机下标
    kMotorIdxYaw,        ///< YAW 轴电机下标
    kMotorIdxPitch,      ///< PITCH 轴电机下标
    kMotorNum,           ///< 电机数量
  };

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
  void registerMotor(Motor *dev_ptr, uint8_t idx, CanTxMgr *tx_mgr_ptr);
  void registerGimbalChassisComm(GimbalChassisComm *dev_ptr, CanTxMgr *tx_mgr_ptr);
  void registerVision(Vision *dev_ptr, UartTxMgr *tx_mgr_ptr);
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

  // 主要模块状态机组件指针
  Gimbal *gimbal_ptr_ = nullptr;    ///< 云台模块指针
  Feed *feed_ptr_ = nullptr;        ///< 拨盘模块指针
  Fric *fric_ptr_ = nullptr;        ///< 摩擦轮模块指针

  // 无通信功能的组件指针
  Buzzer *buzzer_ptr_ = nullptr;  ///< 蜂鸣器指针
  Imu *imu_ptr_ = nullptr;        ///< IMU 指针
  Laser *laser_ptr_ = nullptr;    ///< 红点激光指针

  // 只接收数据的组件指针

  // 只发送数据的组件指针
  Motor *motor_ptr_[kMotorNum] = {nullptr};  ///< 四轮电机指针 只发送数据

  // 收发数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr;  ///< 云台底盘通信模块指针 收发数据
  Vision *vision_ptr_ = nullptr;              ///< 视觉模块指针 收发数据

  // 通讯组件列表
  TxDevMgrPair tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kNum] = {{nullptr}};  ///< 发送设备管理器对数组
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* ROBOT_MODULE_ROBOT_HPP_ */
