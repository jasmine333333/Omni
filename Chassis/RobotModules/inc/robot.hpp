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
#ifndef ROBOT_MODULES_ROBOT_HPP_
#define ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
#include "buzzer.hpp"
#include "can_tx_mgr.hpp"
#include "chassis.hpp"
#include "feed.hpp"
#include "fsm.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "motor.hpp"
#include "referee.hpp"
#include "shooter.hpp"
#include "super_cap.hpp"
#include "tick.hpp"
#include "transmitter.hpp"
#include "uart_rx_mgr.hpp"
#include "uart_tx_mgr.hpp"
#include "imu.hpp"
#include "ui_drawer.hpp"  // Ensure this header file defines the UiDrawer class

/* Exported macro ------------------------------------------------------------*/
namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Robot : public Fsm
{
 public:
  typedef hello_world::buzzer::Buzzer Buzzer;
  typedef hello_world::cap::SuperCap Cap;
  typedef hello_world::comm::Transmitter Transmitter;
  typedef hello_world::comm::CanTxMgr CanTxMgr;
  typedef hello_world::comm::UartTxMgr UartTxMgr;
  typedef hello_world::comm::TxMgr TxMgr;
  typedef hello_world::motor::Motor Motor;
  typedef hello_world::remote_control::DT7 DT7;
  typedef hello_world::remote_control::SwitchState RcSwitchState;

  typedef hello_world::referee::RobotPerformancePackage PerformancePkg;
  typedef hello_world::referee::RobotPowerHeatPackage PowerHeatPkg;
  typedef hello_world::referee::RobotShooterPackage ShooterPkg;
  typedef hello_world::referee::Referee Referee;
  typedef hello_world::referee::CompRobotsHpPackage CompRobotsHpPkg;
  typedef hello_world::referee::RobotHurtPackage RobotHurtPkg;
  typedef hello_world::referee::HpDeductionReason HurtReason;
  typedef hello_world::referee::RobotBuffPackage BuffPkg;

  typedef hello_world::referee::ids::RobotId RobotId;
  typedef robot::Chassis Chassis;
  typedef robot::Gimbal Gimbal;
  typedef hello_world::imu::Imu Imu;
  typedef robot::Shooter Shooter;
  typedef robot::Feed Feed;
  typedef robot::UiDrawer UiDrawer;

 public:
  Robot() {};
  ~Robot() {};

  // 状态机主要接口函数
  void update() override;
  void run() override;
  void reset() override;
  void standby() override;

  void registerChassis(Chassis *ptr);
  void registerFeed(Feed *ptr);
  void registerGimbal(Gimbal *ptr);
  void registerShooter(Shooter *ptr);
  void registerBuzzer(Buzzer *ptr);
  void registerImu(Imu *ptr);
  void registerMotorWheels(Motor *motor_ptr, uint8_t idx);
  void registerCap(Cap *ptr);
  void registerGimbalChassisComm(GimbalChassisComm *ptr);
  void registerReferee(Referee *ptr);
  void registerRc(DT7 *ptr);

  void registerPerformancePkg(PerformancePkg *ptr);
  void registerPowerHeatPkg(PowerHeatPkg *ptr);
  void registerShooterPkg(ShooterPkg *ptr);
  void registerCompRobotsHpPkg(CompRobotsHpPkg *ptr);
  void registerRobotHurtPkg(RobotHurtPkg *ptr);
  void registerBuffPkg(BuffPkg *ptr);
  private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateImuData();
  void updateRfrData();
  void updateRcData();
  void updatePwrState();

  // 各工作状态任务执行函数
  void runOnDead();
  void runOnResurrection();
  void runOnWorking();

  void genModulesCmd();

  void genModulesCmdFromRc();
  void genModulesCmdFromKb();

  // 设置通讯组件数据函数
  void setCommData();
  void setGimbalChassisCommData();
  void setUiDrawerData();

  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();

  // 发送通讯组件数据函数
  void sendCommData();
  void sendCanData();
  void sendWheelsMotorData();
  void sendCapData();
  void sendGimbalChassisCommData();
  void sendRefereeData();
  void sendUsartData();

  uint8_t rfr_tx_data_[255] = {0};  ///< 机器人交互数据包发送缓存
  size_t rfr_tx_data_len_ = 0;      ///< 机器人交互数据包发送缓存长度

  RobotId robot_id_ = RobotId::kRedStandard3;
  uint16_t base_hp = 5000;  ///< 基地血量
  uint16_t last_base_hp = 5000;  ///< 上一次基地血量
  bool base_attack_flag = false;  ///< 基地受攻击标志位
  uint32_t last_base_attack_tick = 0;  ///< 上一次基地受攻击的时间戳
  uint32_t base_attack_tick = 0;  ///< 基地受攻击的时间戳

  bool robot_attacked_flag = false;  ///< 机器人受攻击标志位
  uint8_t hurt_module_id = 0;  ///< 受伤模块ID
  uint8_t hurt_reason = 6;    ///< 扣血原因
  uint16_t last_hp = 100;     ///< 上次血量【裁判系统告知，离线时采用默认值】
  uint32_t robot_attacked_tick = 0;  ///< 机器人受攻击的时间戳
  uint32_t last_robot_attacked_tick = 0;  ///< 上一次机器人受攻击的时间戳
  bool variable_gyro_flag = false;

  
  void setManualCtrlSrc(ManualCtrlSrc src)
  {
    if (src != manual_ctrl_src_) {
      last_manual_ctrl_src_ = manual_ctrl_src_;
      manual_ctrl_src_ = src;
    }
  };

  ManualCtrlSrc getManualCtrlSrc()
  {
    if (manual_ctrl_src_ == ManualCtrlSrc::Rc) {
      return ManualCtrlSrc::Rc;
    } else if (manual_ctrl_src_ == ManualCtrlSrc::Kb) {
      return ManualCtrlSrc::Kb;
    }
  };
  // IMU 数据在 update 函数中更新
  bool is_imu_caled_offset_ = false;  ///< IMU 数据是否计算完零飘

  // RC 数据在 update 函数中更新
  ManualCtrlSrc manual_ctrl_src_ = ManualCtrlSrc::Rc;       ///< 手动控制源
  ManualCtrlSrc last_manual_ctrl_src_ = ManualCtrlSrc::Rc;  ///< 上一手动控制源

  UiDrawer ui_drawer_ ;                                                   ///< UI 绘制器

  uint16_t bullet_num_ = 0;  ///< 子弹数量

  uint32_t last_rev_work_tick_ = 0;
  uint32_t last_rev_chassis_tick_ = 0;

  uint8_t buff_mode_ = false;  ///< buff 模式
  uint8_t last_buff_mode_ = false;  ///< 上一个 buff 模式
  
  bool navigate_flag = 0; //巡航模式
  bool last_navigate_flag = 0; //上一个巡航flag

  // 主要模块状态机组件指针
  Chassis *chassis_ptr_ = nullptr;  ///< 底盘模块指针
  Gimbal *gimbal_ptr_ = nullptr;    ///< 云台模块指针
  Feed *feed_ptr_ = nullptr;        ///< 送弹模块指针
  Shooter *shooter_ptr_ = nullptr;  ///< 发射模块指针

  // 无通信功能的组件指针
  Buzzer *buzzer_ptr_ = nullptr;  ///< 蜂鸣器指针
  Imu *imu_ptr_ = nullptr;        ///< 底盘 IMU 指针

  // 只接收数据的组件指针
  DT7 *rc_ptr_ = nullptr;  ///< DT7 指针 只接收数据

  // 只发送数据的组件指针
  Cap *cap_ptr_ = nullptr;  ///< 底盘超级电容指针 只发送数据

  Motor *motor_wheels_ptr_[Chassis::kWheelMotorNum] = {nullptr};  ///< 四轮电机指针 只发送数据

  // 收发数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr;           ///< 云台底盘通信模块指针 收发数据
  Referee *referee_ptr_ = nullptr;                     ///< 裁判系统指针 收发数据
  PerformancePkg *rfr_performance_pkg_ptr_ = nullptr;  ///< 裁判系统性能包指针 收发数据
  PowerHeatPkg *rfr_power_heat_pkg_ptr_ = nullptr;     ///< 裁判系统电源和热量包指针 收发数据
  ShooterPkg *rfr_shooter_pkg_ptr_ = nullptr;          ///< 裁判系统射击包指针 收发数据
  CompRobotsHpPkg *rfr_comp_robots_hp_pkg_ptr_ = nullptr;  ///< 裁判系统机器人血量包指针 收发数据
  RobotHurtPkg *rfr_robot_hurt_pkg_ptr_ = nullptr;  ///< 裁判系统机器人受伤包指针 收发数据
  BuffPkg *rfr_buff_pkg_ptr_ = nullptr;  ///< 裁判系统机器人buff包指针 收发数据
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* ROBOT_MODULE_ROBOT_HPP_ */
