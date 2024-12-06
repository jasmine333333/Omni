/** 
 *******************************************************************************
 * @file      : shooter.hpp
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
#ifndef ROBOT_MODULE_SHOOTER_HPP_
#define ROBOT_MODULE_SHOOTER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "module_fsm.hpp"
#include "module_state.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "usr_offline_checker.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace robot
{
/* Exported constants --------------------------------------------------------*/

enum class ShooterStuckStatus : uint8_t {
  None = 0u,  ///< 没有卡住
  Farward,    ///< 向前卡住
  Backward,   ///< 向后卡住
};

/* Exported types ------------------------------------------------------------*/

struct ShooterRfrInputData {
  bool is_power_on = false;    ///< 发射机构电源是否开启
  float heat_limit = 100.0;    ///< 发射机构热量限制
  float heat = 0;              ///< 发射机构热量
  float heat_cooling_ps = 10;  ///< 发射机构热量的每秒冷却值
  float bullet_speed = 15.5f;  ///< 发射机构检测到的弹丸射速 m/s
};

struct ShooterRfrData {
  bool is_power_on = false;         ///< 发射机构电源是否开启
  float heat_limit = 100.0;         ///< 发射机构热量限制
  float heat = 0;                   ///< 发射机构热量
  float last_heat_ = 0;             ///< 上一次发射机构热量
  float heat_cooling_ps = 10;       ///< 发射机构热量的每秒冷却值
  float bullet_speed = 15.5f;       ///< 发射机构检测到的弹丸射速
  float last_bullet_speed = 15.5f;  ///< 上一次发射机构检测到的弹丸射速
};

struct ShooterConfig {
  float min_bullet_speed = 14.0f;   ///< 最小弹丸射速 m/s
  float max_bullet_speed = 16.5f;   ///< 最大弹丸射速 m/s
  float tgt_fric_spd_ref = 640.0f;  ///< 达到目标弹速的摩擦轮期望速度

  float feed_ang_ref_offset;        ///< 拨盘电机目标角度偏移量
  float feed_ang_per_blt = PI / 3;  ///< 1发弹丸拨盘的转动角度
  float heat_per_bullet;            ///< 1发弹耗热量

  float feed_stuck_curr_thre = 14.0f;  ///< 用于判断堵转的电流阈值
  float fric_stuck_curr_thre = 14.0f;  ///< 用于判断摩擦轮堵转的电流阈值
  float fric_spd_delta_thre = 10.0f;   ///< 用于判断摩擦轮速度保持恒定的阈值 (rad)
  float fric_spd_err_thre = 5.0f;      ///< 用于判断摩擦轮速度跟上期望转速的阈值 (rad)
};

class Shooter : public robot::Fsm
{
 public:
  typedef hello_world::pid::MultiNodesPid Pid;
  typedef hello_world::motor::Motor Motor;

  typedef robot::OfflineChecker OfflineChecker;

  typedef ShooterWorkingMode WorkingMode;

  typedef ShooterRfrInputData RfrInputData;
  typedef ShooterRfrData RfrData;
  typedef ShooterStuckStatus StuckStatus;
  typedef ShooterConfig Config;

  /** 电机ID */
  enum MotorIdx : uint8_t {
    kMotorIdxFricLeft,   ///< 摩擦轮左侧电机
    kMotorIdxFricRight,  ///< 摩擦轮右侧电机
    kMotorIdxFeed,       ///< 拨盘电机
    kMotorNum,           ///< 电机数量
  };

  /** PID ID */
  enum PidIdx : uint8_t {
    kPidIdxFricLeft,   ///< 摩擦轮左侧 PID
    kPidIdxFricRight,  ///< 摩擦轮右侧 PID
    kPidIdxFeed,       ///< 拨盘 PID
    kPidNum,           ///< PID 数量
  };

  Shooter() = default;
  Shooter(const Config &config) { cfg_ = config; };
  ~Shooter() {};

  Config &getConfig() { return cfg_; }
  const Config &getConfig() const { return cfg_; }

  void update() override;

  void run() override;

  void reset() override;

  void standby() override;

  void setCtrlMode(CtrlMode mode) { ctrl_mode_ = mode; }
  CtrlMode getCtrlMode() const { return ctrl_mode_; }

  void setWorkingMode(WorkingMode mode) { working_mode_ = mode; }
  WorkingMode getWorkingMode() const { return working_mode_; }

  void setShootFlag(bool flag) { shoot_flag_ = flag; }
  void setAutoShootFlag(bool flag) { auto_shoot_flag_ = flag; }

  void updateRfrData(const RfrInputData &inp_data);

  bool getShootFlag() const { return shoot_flag_; }
  bool getAutoShootFlag() const { return auto_shoot_flag_; }

  float getBulletSpeed() const { return rfr_data_.bullet_speed; }
  // 注册组件指针
  void registerMotor(Motor *ptr, int idx);
  void registerPid(Pid *ptr, int idx);

 private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateMotorData();
  void updateMotorFeedStuckStatus();
  void updateMotorFricStuckStatus();
  void updateMotorFeedHoldStatus();
  void updateMotorFricHoldStatus();

  void updatePwrState();

  // 执行任务
  void runOnDead();
  void runOnResurrection();
  void runOnWorking();

  bool limitTriggerByFreq(uint32_t interval);
  bool limitTriggerByHeat(float safe_num_remain_bullet);

  void calcFeedAngRefOnResurrection();
  void calcFeedAngRefOnWorking();
  void calcFricSpdRefOnResurrection();
  void calcFricSpdRefOnWorking();
  void calcMotorInput();
  void calcMotorFeedInput();
  void calcMotorFricInput();

  float searchFeedAngRef(float fdb_ang, float offset, bool is_farward, float ang_per_bullet = PI / 3) const;
  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();
  void resetPids();

  // 设置通讯组件数据函数
  void setCommData(bool is_working);

  Config cfg_;

  // 由 robot 设置的数据
  bool shoot_flag_ = false;       ///< 手动射击标志位，由外部设置，由 runOnWorking 函数读取并执行
  bool auto_shoot_flag_ = false;  ///< 自动射击标志位，由外部设置，由 runOnWorking 函数读取并执行

  CtrlMode ctrl_mode_ = CtrlMode::Manual;           ///< 控制模式
  WorkingMode working_mode_ = WorkingMode::Normal;  ///< 工作模式

  // 内部管理数据
  bool is_power_on_ = false;          ///< 发射机构电源是否开启
  bool is_any_motor_pwr_on_ = false;  ///< 是否有任意电机在线
  float fake_heat_ = 0.0f;            ///< 枪口热量【伪】 自行记录的枪口热量

  bool trigger_signal_ = false;     ///< 拨弹信号，由 limitTriggerByFrequence 和 limitTriggerByHeat 函数生成
  uint32_t last_trigger_tick_ = 0;  ///< 上一次拨弹的时刻

  float fric_spd_ref_tbs_ = 640.0f;  ///< 摩擦轮目标速度【达到理想弹丸射速】【只有reset函数能重置】，普通弹丸600，荧光弹丸640

  uint32_t continue_shoot_duration_ = 0;  ///< 连续射击持续时间

  uint32_t resurrection_duration_ = 0;  ///< 复活阶段持续时间

  // 拨盘
  struct FeedStatus {
    // update 函数中更新的数据
    bool is_ang_inited = false;  ///< 拨盘初始化是否完成

    bool is_ang_ref_inited = false;  ///< 拨盘电机目标角度是否初始化

    StuckStatus stuck_status = StuckStatus::None;  ///< 卡住状态

    uint32_t ang_hold_duration = 0;  ///< 拨盘就绪持续时间，当缓慢回转时可能达不到电流阈值，需要额外的检测手段来判断是否拨盘就绪
    uint32_t stuck_duration = 0;  ///< 卡住持续时间

    float ang_ref;  ///< 拨盘电机目标角度

    // 从电机中拿到的数据
    float ang_fdb;       ///< 拨盘电机反馈角度 单位 rad
    float last_ang_fdb;  ///< 上一控制周期拨盘电机反馈角度 单位 rad
    float spd_fdb;       ///< 拨盘电机反馈速度 单位 rad/s
    float cur_fdb;       ///< 拨盘电机反馈电流 单位 A
    float cur_ref;       ///< 拨盘电机输入信号 单位 A

    void resetStuckStatus()
    {
      stuck_status = StuckStatus::None;
      stuck_duration = 0;
    };
    void resetHoldStatus() { ang_hold_duration = 0; };
  } feed_status_;  ///< 拨盘状态

  // 摩擦轮
  struct FricStatus {
    enum FricIdx : uint8_t {
      kFricIdxLeft,
      kFricIdxRight,

    };
    // update 函数中更新的数据
    bool is_inited = false;  ///< 摩擦轮初始化是否完成

    StuckStatus stuck_status = StuckStatus::None;  ///< 摩擦轮卡住状态

    uint32_t spd_hold_duration = 0;  ///< 摩擦轮就绪持续时间
    uint32_t stuck_duration = 0;     ///< 摩擦轮卡住持续时间

    float spd_ref;  ///< 摩擦轮目标速度 单位 rad/s

    // 从电机中拿到的数据
    float spd_fdb[2];       ///< 摩擦轮反馈速度 单位 rad/s
    float last_spd_fdb[2];  ///< 上一控制周期摩擦轮反馈速度 单位 rad/s
    float cur_fdb[2];       ///< 摩擦轮反馈电流 单位 A
    float cur_ref[2];       ///< 摩擦轮输入信号

    void resetStuckStatus()
    {
      spd_hold_duration = 0;
      stuck_status = StuckStatus::None;
    };
    void resetHoldStatus() { spd_hold_duration = 0; };
  } fric_status_;  ///< 摩擦轮状态

  // rfr fdb data 在 update 函数中更新的数据
  RfrData rfr_data_;  ///< 裁判系统中与发射机构相关的数据

  // 各组件指针
  // 无通信功能的组件指针
  Pid *pid_ptr_[kPidNum] = {nullptr};  ///< PID 指针
  // 只接收数据的组件指针
  // 接收、发送数据的组件指针
  Motor *motor_ptr_[kMotorNum] = {nullptr};  ///< 电机指针 接收、发送数据
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* ROBOT_MODULE_SHOOTER_HPP_ */
