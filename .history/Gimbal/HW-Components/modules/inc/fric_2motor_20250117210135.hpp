/**
 *******************************************************************************
 * @file      : fric_2motor.cpp
 * @brief     : 单级共速双摩擦轮模块，暂时供步兵、哨兵、无人机使用。
 * @history   :
 *  Version     Date            Author                       Note
 *  V0.9.0      2024.12.12      ZhouShichan, LouKaiyang      1. 初版编写完成
 *******************************************************************************
 * @attention : 后续可能推出 n 级 n 摩擦轮模块，级数和每一级的摩擦轮数量可以自定义，
 *              同时允许每个摩擦轮单独指定期望速度。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_MODULES_FRIC_HPP_
#define HW_COMPONENTS_MODULES_FRIC_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "module_fsm.hpp"
#include "motor.hpp"
#include "pid.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace hello_world
{
namespace module
{
namespace fric_impl
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* 工作模式（枚举类） */
enum FricWorkingMode : uint8_t {
  kStop,      // 停转模式
  kShoot,     // 正常射击模式
  kBackward,  // 倒转模式
};
/* 卡弹状态（枚举类） */
enum class FricStuckStatus : uint8_t {
  kNone = 0u,  // 没有卡住
  kForward,    // 向前卡住
  kBackward,   // 向后卡住
};

/* 以下两个编号类的作用是方便用户在注册电机时指定编号，
 * 内部运行时不使用它们来索引，而是直接 0, 1, 2... */

/* 摩擦轮电机编号（枚举类）
  不写成"Left、Right"是因为可能有竖直双摩擦轮的情况
*/
enum class FricMotorIdx2Motor : uint8_t {
  kFirst = 0u,
  kSecond,
  kNum,
};

/* 摩擦轮PID编号（枚举类） */
enum class FricPidIdx2Motor : uint8_t {
  kFirst = 0u,
  kSecond,
  kSameSpd,
  kNum,
};

/* 裁判系统反馈数据 */
struct FricRfrInputData {
  bool is_power_on = false;          // 摩擦轮电源是否开启
  float is_new_bullet_shot = false;  // 是否有一颗新弹丸射出
  float bullet_spd;                  // 发射机构检测到的弹丸速度, 无默认值, m/s
};

/* 优化项：弹速闭环 */
struct BulletSpdClosedLoop {
  bool is_enabled = false;       // 是否开启弹速闭环
  float min_reasonable_blt_spd;  // 最小合理弹丸速度, >0, 无默认值, m/s, 小于该值认为裁判系统反馈数据错误
  float max_reasonable_blt_spd;  // 最大合理弹丸速度, >0, 无默认值, m/s, 大于该值认为裁判系统反馈数据错误
  float min_target_blt_spd;      // 弹丸速度期望值区间下限, >0, 无默认值, m/s
  float max_target_blt_spd;      // 弹丸速度期望值区间上限, >0, 无默认值, m/s
  float spd_gradient = 5.0f;     // 摩擦轮转速调整梯度, >=0, 默认值 5 rad/s
};

struct FricConfig2Motor {
  float default_spd_ref;                     // 摩擦轮期望速度预设值, >0, 无默认值, rad/s
  float default_spd_ref_backward = -100.0f;  // 摩擦轮反转目标速度, <0, 默认值 -100 rad/s, 反转模式是为了将卡在摩擦轮中间的弹丸回退出来，转速不易过快
  float stuck_curr_thre = 14.0f;             // 用于判断摩擦轮堵转的电流阈值, >0, 默认值 14 A
  float spd_delta_thre = 10.0f;              // 用于判断摩擦轮速度保持恒定的阈值, >0, 默认值 10 rad/s
  float spd_err_thre = 5.0f;                 // 用于判断摩擦轮速度跟上期望转速的阈值, >0, 默认值 5 rad/s
  float spd_stop_thre = 100.0f;              // 摩擦轮Stop模式，转速小于该阈值后，停止控制电机, >0, 默认值 100 rad/s
  uint16_t resurrection_duration = 500;      // 复活模式持续时间
  /* 优化项，建议开启 */
  bool opt_spd_same_pid_enabled = false;  // 是否使用双摩擦轮同速PID(期望为0，反馈输入为两轮差速，输出分别正负作用到两个电机上)
  BulletSpdClosedLoop opt_blt_spd_cl;     // 弹速闭环优化
};

/* 被放置于 hello_world::module 命名空间下 */
class Fric : public ModuleFsm
{
 public:
  typedef pid::MultiNodesPid Pid;
  typedef motor::Motor Motor;
  typedef FricWorkingMode WorkingMode;
  typedef FricStuckStatus StuckStatus;
  typedef FricRfrInputData RfrInputData;
  typedef FricConfig2Motor Config;
  typedef FricMotorIdx2Motor MotorIdx;
  typedef FricPidIdx2Motor PidIdx;

  /* 唯一构造函数 */
  explicit Fric(const Config &config);

  /* 析构函数 */
  ~Fric() {};

  /* 运行工作 */
  void update() override;
  void reset() override;
  void standby() override;

  /* 外部更新裁判系统数据 */
  void updateRfrData(const FricRfrInputData &inp_data);

  /* 设定控制指令 */
  void setWorkingMode(WorkingMode mode) { working_mode_ = mode; }

  /* 读取内部数据 */
  bool getStatus() const { return status_.is_ready; }  // 摩擦轮是否准备好发弹
  Config &getConfig() { return cfg_; }
  const Config &getConfig() const { return cfg_; }
  WorkingMode getWorkingMode() const { return working_mode_; }

  /* 注册组件指针 */
  void registerMotor(Motor *ptr, MotorIdx idx);
  void registerPid(Pid *ptr, PidIdx idx);

 private:
  /* 数据更新和工作状态更新，由 update 函数调用 */
  void updateData();
  void updateMotorData();
  void updateStuckStatus();
  void updateSpdStatus();

  void updatePwrState();

  /* 执行任务 */
  void runOnDead() override;
  void runOnResurrection() override;
  void runOnWorking() override;
  void runAlways() override;

  void calcSpdRef();
  void calcMotorInput();

  /* 重置数据函数 */
  void resetDataOnDead();
  void resetDataOnResurrection();
  void resetPids();

  /* 设置通信组件数据函数 */
  void setCommData(bool is_working);

  /* 设置配置函数（内部使用，不允许用户在线修改配置） */
  void setConfig(const Config &config);
  Config cfg_;

  /* 由robot 设置的数据 */
  WorkingMode working_mode_ = WorkingMode::kStop;  // 工作模式

  /* 内部管理数据 */
  bool is_power_on_ = false;          // 发射机构电源是否开启
  bool is_any_motor_pwr_on_ = false;  // 是否有任意电机在线
  uint32_t resurrection_tick_ = 0;    // 复活模式持续时间

  /* 摩擦轮状态 */
  struct Status {
    bool is_ready = false;                          // 摩擦轮是否准备好发弹
    uint32_t stuck_duration = 0;                    // 摩擦轮卡住持续时间
    StuckStatus stuck_status = StuckStatus::kNone;  // 摩擦轮卡住状态
    uint32_t spd_ready_duration = 0;                // 摩擦轮就绪持续时间
    float actual_spd_ref;                           // 摩擦轮实际期望速度, 单位 rad/s
    float normal_spd_ref;                           // 摩擦轮正常期望速度, 单位 rad/s
    /* 从电机中拿到的数据 */
    float spd_fdb[2];       // 摩擦轮反馈速度 单位 rad/s
    float last_spd_fdb[2];  // 上一控制周期摩擦轮反馈速度 单位 rad/s
    float cur_ref[2];       // 摩擦轮输入信号
    float cur_fdb[2];       // 摩擦轮反馈电流 单位 A
    /* 工具函数 */
    void resetStuckStatus()
    {
      stuck_duration = 0;
      stuck_status = StuckStatus::kNone;
    };
    void resetSpdStatus() { spd_ready_duration = 0; };
  } status_;

  /* 内部维护的裁判系统反馈数据 */
  struct FricRfrData {
    bool is_power_on = false;         // 摩擦轮电源是否开启
    bool is_new_bullet_shot = false;  // 是否有一颗新的弹丸射出
    float bullet_spd;                 // 发射机构检测到的弹丸射速
    float last_bullet_spd;            // 上一次发射机构检测到的弹丸射速
  } rfr_data_;                        // 裁判系统中与发射机构相关的数据

  /* 无通信功能的组件指针 */
  Pid *pid_ptr_[3] = {nullptr};  // PID 指针
  /* 接收、发送数据的组件指针 */
  Motor *motor_ptr_[2] = {nullptr};  // 电机指针 接收、发送数据

  /* Debug变量 */
  PwrState debug_pwr_state_ = PwrState::kDead;
};  // class Fric
}  // namespace fric_impl
using fric_impl::Fric;
}  // namespace module
}  // namespace hello_world
#endif /** HW_COMPONENTS_MODULES_FRIC_HPP_ */