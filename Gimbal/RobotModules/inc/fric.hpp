/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_MODULE_FRIC_HPP_
#define ROBOT_MODULE_FRIC_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "module_fsm.hpp"
#include "module_state.hpp"
#include "motor.hpp"
#include "pid.hpp"

/* Exported macro ------------------------------------------------------------*/

namespace robot
{
/* Exported constants --------------------------------------------------------*/

enum class FricStuckStatus : uint8_t
{
  None = 0u,  ///< 没有卡住
  Farward,    ///< 向前卡住
  Backward,   ///< 向后卡住
};

/* Exported types ------------------------------------------------------------*/

struct FricRfrInputData
{
  bool is_power_on = false;    ///< 摩擦轮电源是否开启
  float heat_cooling_ps = 10;  ///< 摩擦轮热量的每秒冷却值
  float bullet_speed = 15.5f;    ///< 发射机构检测到的弹丸速度 m/s
};

struct FricRfrData
{
  bool is_power_on = false;         ///< 摩擦轮电源是否开启
  float bullet_speed = 15.5f;         ///< 发射机构检测到的弹丸射速
  float last_bullet_speed = 15.5f;    ///< 上一次发射机构检测到的弹丸射速
};

struct FricConfig
{
  float min_bullet_speed = 14.0f;             ///< 最小合理弹丸速度 m/s
  float max_bullet_speed = 16.5f;             ///< 最大合理弹丸速度 m/s
  float tgt_max_bullet_speed = 15.9f;         ///< 最大目标弹丸速度 m/s
  float tgt_min_bullet_speed = 15.3f;         ///< 最小目标弹丸速度 m/s
  float tgt_fric_spd_ref = 597.0f;            ///< 摩擦轮期望速度预设值 rad/s
  float tgt_fric_spd_ref_backward = -100.0f;  ///< 摩擦轮反转目标速度 rad/s

  float fric_stuck_curr_thre = 14.0f;  ///< 用于判断摩擦轮堵转的电流阈值
  float fric_spd_delta_thre = 10.0f;   ///< 用于判断摩擦轮速度保持恒定的阈值 (rad)
  float fric_spd_err_thre = 5.0f;      ///< 用于判断摩擦轮速度跟上期望转速的阈值 (rad)
};

class Fric : public robot::Fsm
{
 public:
  typedef hello_world::pid::MultiNodesPid Pid;
  typedef hello_world::motor::Motor Motor;

  typedef ShooterWorkingMode WorkingMode;

  typedef FricStuckStatus StuckStatus;
  typedef FricConfig Config;

  /** 电机ID */
  enum MotorIdx : uint8_t
  {
    kMotorIdxFricLeft,   ///< 摩擦轮左侧电机
    kMotorIdxFricRight,  ///< 摩擦轮右侧电机
    kMotorNum,           ///< 电机数量
  };

  /** PID ID */
  enum PidIdx : uint8_t
  {
    kPidIdxFricLeft,   ///< 摩擦轮左侧 PID
    kPidIdxFricRight,  ///< 摩擦轮右侧 PID
    kPidNum,           ///< PID 数量
  };

  Fric() = default;
  Fric(const Config &config) { cfg_ = config; };
  ~Fric(){};

  Config &getConfig() { return cfg_; }
  const Config &getConfig() const { return cfg_; }

  void update() override;

  void run() override;

  void reset() override;

  void standby() override;
  void setWorkingMode(WorkingMode mode) { working_mode_ = mode; }
  WorkingMode getWorkingMode() const { return working_mode_; }

  void updateRfrData(const FricRfrInputData &inp_data);

  float getBulletSpeed() const { return rfr_data_.bullet_speed; }
  bool getFricStatus() const { return fric_status_.is_inited; }

  // 注册组件指针
  void registerMotor(Motor *ptr, int idx);
  void registerPid(Pid *ptr, int idx);

 private:
  // 数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateMotorData();
  void updateMotorFricStuckStatus();
  void updateMotorFricHoldStatus();

  void updatePwrState();

  // 执行任务
  void runOnDead();
  void runOnResurrection();
  void runOnWorking();

  void calcFricSpdRefOnResurrection();
  void calcFricSpdRefOnWorking();
  void calcMotorFricInput();

  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();
  void resetPids();

  // 设置通信组件数据函数
  void setCommData(bool is_working);

  Config cfg_;

  // 由robot 设置的数据
  WorkingMode working_mode_ = WorkingMode::Normal;  ///< 工作模式

  // 内部管理数据
  bool is_power_on_ = false;          ///< 发射机构电源是否开启
  bool is_any_motor_pwr_on_ = false;  ///< 是否有任意电机在线

  uint32_t resurrection_duration_ = 0;  ///< 复活阶段持续时间

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
  FricRfrData rfr_data_;  ///< 裁判系统中与发射机构相关的数据

  // 各组件指针
  // 无通信功能的组件指针
  Pid *pid_ptr_[kPidNum] = {nullptr};  ///< PID 指针
  // 只接收数据的组件指针
  // 接收、发送数据的组件指针
  Motor *motor_ptr_[kMotorNum] = {nullptr};  ///< 电机指针 接收、发送数据
};
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
}  // namespace robot
#endif  /** ROBOT_MODULE_FRIC_HPP_ */