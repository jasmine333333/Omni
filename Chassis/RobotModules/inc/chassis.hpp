/** 
 *******************************************************************************
 * @file      :chassis.hpp
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
#ifndef ROBOT_MODULES_CHASSIS_HPP_
#define ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>

#include "allocator.hpp"
#include "chassis_iksolver.hpp"
#include "gimbal_chassis_comm.hpp"
#include "module_fsm_private.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "power_limiter.hpp"
#include "super_cap.hpp"
#include "usr_imu.hpp"
/* Exported macro ------------------------------------------------------------*/
// namespace hw_rfr = hello_world::referee;

namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

union ChassisCmd {
  struct {
    float v_x;
    float v_y;
    float w;
  };
  float data[3];
  void reset()
  {
    v_x = 0;
    v_y = 0;
    w = 0;
  }
  ChassisCmd operator+(const ChassisCmd &other) const { return {v_x + other.v_x, v_y + other.v_y, w + other.w}; }

  ChassisCmd operator-(const ChassisCmd &other) const { return {v_x - other.v_x, v_y - other.v_y, w - other.w}; }

  ChassisCmd operator*(float scalar) const { return {v_x * scalar, v_y * scalar, w * scalar}; }

  ChassisCmd operator+=(const ChassisCmd &other)
  {
    v_x += other.v_x;
    v_y += other.v_y;
    w += other.w;
    return *this;
  }

  ChassisCmd operator-=(const ChassisCmd &other)
  {
    v_x -= other.v_x;
    v_y -= other.v_y;
    w -= other.w;
    return *this;
  }

  ChassisCmd operator*=(float scalar)
  {
    v_x *= scalar;
    v_y *= scalar;
    w *= scalar;
    return *this;
  }

  friend ChassisCmd operator*(float scalar, const ChassisCmd &cmd);
};
struct ChassisRfrData {
  bool is_rfr_on = false;     ///< 裁判系统是否在线
  bool is_pwr_on = false;     ///< 机器人底盘电源是否开启【裁判系统告知，离线时默认开启】
  float pwr = 0;              ///< 机器人底盘功率【裁判系统告知，离线时默认0】
  uint16_t pwr_limit = 60;    ///< 机器人底盘功率限制【裁判系统告知，离线时采用默认值】
  uint16_t pwr_buffer = 60;   ///< 机器人底盘缓冲能量【裁判系统告知，离线时采用默认值】
  uint16_t voltage = 24;      ///< 底盘电压【裁判系统告知，离线时采用默认值】
  uint16_t current_hp = 100;  ///< 底盘血量【裁判系统告知，离线时采用默认值】
};

struct ChassisConfig {
  float normal_trans_vel;  ///< 正常平移速度
  float normal_rot_spd;    ///< 正常旋转速度
  float max_trans_vel;     ///< 最大平移速度
  float max_rot_spd;       ///< 最大旋转速度
  float cmd_smooth_factor;  ///< 运动指令平滑系数, 值域[0,1], 启用超电时默认为1
};

class Chassis : public Fsm
{
 public:
  typedef hello_world::motor::Motor Motor;
  typedef hello_world::pid::MultiNodesPid MultiNodesPid;
  typedef hello_world::chassis_ik_solver::ChassisIkSolver ChassisIkSolver;
  typedef hello_world::cap::SuperCap Cap;
  typedef hello_world::power_limiter::PowerLimiter PwrLimiter;

  typedef robot::GimbalChassisComm GimbalChassisComm;
  typedef ChassisWorkingMode WorkingMode;
  typedef robot::Imu Imu;


  typedef ChassisCmd Cmd;
  typedef ChassisRfrData RfrData;
  typedef ChassisConfig Config;

  static std::string WorkingModeToStr(WorkingMode mode)
  {
    if (mode == Chassis::WorkingMode::Depart) return "Depart";
    if (mode == Chassis::WorkingMode::Follow) return "Follow";
    if (mode == Chassis::WorkingMode::Gyro) return "Gyro";
    return "ErrCWM";
  };

  enum class GyroDir : int8_t {
    Clockwise = -1,     ///< 顺时针
    Unspecified = 0,    ///< 未指定
    AntiClockwise = 1,  ///< 逆时针
  };

  enum WheelMotorIdx : uint8_t {
    kWheelMotorIdxLeftFront,   ///< 左前轮电机下标
    kWheelMotorIdxLeftRear,    ///< 左后轮电机下标
    kWheelMotorIdxRightRear,   ///< 右后轮电机下标
    kWheelMotorIdxRightFront,  ///< 右前轮电机下标
    kWheelMotorNum,            ///< 电机数量
  };

  enum WheelPidIdx : uint8_t {
    kWheelPidIdxLeftFront,   ///< 左前轮 PID
    kWheelPidIdxLeftRear,    ///< 左后轮 PID
    kWheelPidIdxRightRear,   ///< 右后轮 PID
    kWheelPidIdxRightFront,  ///< 右前轮 PID
    kWheelPidNum,            ///< PID 数量
  };

  Chassis(const Config &cfg) { cfg_ = cfg; };
  ~Chassis() {};

  virtual void update() override;
  virtual void run() override;
  virtual void reset() override;
  virtual void standby() override;

  WorkingMode getWorkingMode() const { return working_mode_; }
  WorkingMode getLastWorkingMode() const { return last_working_mode_; }
  void setWorkingMode(WorkingMode mode);
  void setNormCmd(const Cmd &cmd) { norm_cmd_ = cmd; }
  void setRfrData(const RfrData &data) { rfr_data_ = data; }
  float getThetaI2r(bool actual_head_dir = true) const;
  void revHead()
  {
    if (work_tick_ - last_rev_head_tick_ > 200) {
      rev_head_flag_ = !rev_head_flag_;
      last_rev_head_tick_ = work_tick_;
    }
  }
  void setGyroDir(GyroDir dir);
  void setUseCapFlag(bool flag) { use_cap_flag_ = flag; }
  bool getUseCapFlag() const { return use_cap_flag_; }

  void registerIkSolver(ChassisIkSolver *ptr);
  void registerWheelMotor(Motor *ptr, int idx);
  void registerYawMotor(Motor *ptr);
  void registerWheelPid(MultiNodesPid *ptr, int idx);
  void registerFollowOmegaPid(MultiNodesPid *ptr);
  void registerCap(Cap *ptr);
  void registerGimbalChassisComm(GimbalChassisComm *ptr);
  void registerPwrLimiter(PwrLimiter *ptr);
  void registerImu(Imu *ptr);

 private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateGimbalBoard();
  void updateMotor();
  void updateCap();
  void updateIsPowerOn();
  void updatePwrState();

  void runOnDead();
  void runOnResurrection();
  void runOnWorking();

  // 工作状态下，获取控制指令的函数
  void revNormCmd();
  void calcWheelSpeedRef();
  void calcwheelfeedbackRef();
  void calcWheelLimitedSpeedRef();
  void calcPwrLimitedCurrentRef();
  void calcWheelCurrentRef();
  void calcWheelCurrentLimited();
  void calcWheelRawInput();

  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();
  void resetDataOnStandby();
  void resetPids();

  // 设置通讯组件数据函数
  void setCommData(bool is_working)
  {
    setCommDataWheels(is_working);
    setCommDataCap(is_working);
  };
  void setCommDataWheels(bool is_working);
  void setCommDataCap(bool is_working);

  // 工具函数
  void setCmdSmoothly(const Cmd &cmd, float beta = 0.9)
  {
    beta = hello_world::Bound(beta, 0.0, 1.0);
    cmd_ = cmd * beta + (1 - beta) * last_cmd_;
    last_cmd_ = cmd_;
  };

  // 配置参数
  Config cfg_;


  // 由 robot 设置的数据
  bool use_cap_flag_ = false;              ///< 是否使用超级电容
  bool is_gyro2follow_handled_ = false;    ///< 小陀螺切跟随是否已经处理
  GyroDir gyro_dir_ = GyroDir::Unspecified;  ///< 小陀螺方向，正为绕 Z 轴逆时针，负为顺时针，
  GyroDir last_gyro_dir_ = GyroDir::Unspecified;  ///< 上一次小陀螺方向
  Cmd norm_cmd_ = {0};                     ///< 原始控制指令，基于图传坐标系
  ChassisRfrData rfr_data_;                ///< 底盘 RFR 数据

  WorkingMode working_mode_ = WorkingMode::Depart;       ///< 工作模式
  WorkingMode last_working_mode_ = WorkingMode::Depart;  ///< 上一次工作模式

  // 在 update 函数中更新的数据
  bool is_power_on_ = false;        ///< 底盘电源是否开启
  uint32_t last_pwr_off_tick_ = 0;  ///< 上一次底盘电源处于关闭状态的时间戳，单位为 ms，实际上是作为上电瞬间的记录

  // 在 runOnWorking 函数中更新的数据
  Cmd cmd_ = {0}, last_cmd_ = {0};          ///< 控制指令，基于图传坐标系
  float wheel_speed_ref_[4] = {0};          ///< 轮电机的速度参考值 单位 rad/s
  float wheel_speed_ref_limited_[4] = {0};  ///< 轮电机的速度参考值(限幅后) 单位 rad/s
  float wheel_current_ref_[4] = {0};        ///< 轮电机的电流参考值 单位 A [-20, 20]
  bool rev_head_flag_ = false;              ///< 转向后退标志
  float last_rev_head_angle_ = 0.0f;         ///< 上一次转向后退的标志
  uint32_t last_rev_head_tick_ = 0;         ///< 上一次转向后退的时间戳
  bool rev_chassis_flag_ = false;           ///< 底盘后退标志
  uint32_t last_rev_chassis_tick_ = 0;      ///< 上一次转向后退的时间戳

  // gimbal board fdb data  在 update 函数中更新
  bool is_gimbal_imu_ready_ = false;  ///< 云台主控板的IMU是否准备完毕

  // motor fdb data 在 update 函数中更新
  bool is_all_wheel_online_ = false;  ///< 所有轮电机是否都处于就绪状态
  bool is_any_wheel_online_ = false;  ///< 任意电机是否处于就绪状态
  float wheel_speed_fdb_[4] = {0};    ///< 轮速反馈数据
  float wheel_current_fdb_[4] = {0};  ///< 轮电流反馈数据
  float theta_i2r_ = 0.0f;            ///< 图传坐标系绕 Z 轴到底盘坐标系的旋转角度，右手定则判定正反向，单位 rad

  // cap fdb data 在 update 函数中更新
  bool is_high_spd_enabled_ = false;   ///< 是否开启了高速模式 （开启意味着从电容取电）
  float cap_remaining_energy_ = 0.0f;  ///< 剩余电容能量百分比，单位 %

  // 各组件指针
  // 无通信功能的组件指针
  ChassisIkSolver *ik_solver_ptr_ = nullptr;                ///< 逆解算器指针
  MultiNodesPid *wheel_pid_ptr_[kWheelPidNum] = {nullptr};  ///< PID 指针
  MultiNodesPid *follow_omega_pid_ptr_ = nullptr;           ///< 跟随模式下角速度 PID 指针
  PwrLimiter *pwr_limiter_ptr_ = nullptr;
  // 只接收数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr;  ///< 云台底盘通信器指针 只接收数据
  Motor *yaw_motor_ptr_ = nullptr;            ///< 云台电机指针 接收、发送数据
  Imu *imu_ptr_ = nullptr;                    ///< IMU指针 接收数据
  // 接收、发送数据的组件指针
  Cap *cap_ptr_ = nullptr;                              ///< 超电指针 接收、发送数据
  Motor *wheel_motor_ptr_[kWheelMotorNum] = {nullptr};  ///< 电机指针 接收、发送数据 【YAW 只接收数据】
};

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

inline ChassisCmd operator*(float scalar, const ChassisCmd &cmd) { return cmd * scalar; };

}  // namespace robot
#endif /* ROBOT_MODULES_CHASSIS_HPP_ */
