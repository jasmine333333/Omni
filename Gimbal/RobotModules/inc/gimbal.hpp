/** 
 *******************************************************************************
 * @file      :gimbal.hpp
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
#ifndef ROBOT_MODULES_GIMBAL_HPP_
#define ROBOT_MODULES_GIMBAL_HPP_
/* Includes ------------------------------------------------------------------*/
#include "filter.hpp"
#include "module_fsm_private.hpp"
#include "module_state.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "imu.hpp"
#include "feed.hpp"
#include "laser.hpp"

/* Exported macro ------------------------------------------------------------*/

namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
union GimbalCmd {
  struct {
    float pitch;
    float yaw;
  };
  float data[2];

  GimbalCmd operator+(const GimbalCmd &other) const { return {pitch + other.pitch, yaw + other.yaw}; }

  GimbalCmd operator-(const GimbalCmd &other) const { return {pitch - other.pitch, yaw - other.yaw}; }

  GimbalCmd operator*(float scalar) const { return {pitch * scalar, yaw * scalar}; }

  GimbalCmd operator+=(const GimbalCmd &other)
  {
    pitch += other.pitch;
    yaw += other.yaw;
    return *this;
  }

  GimbalCmd operator-=(const GimbalCmd &other)
  {
    pitch -= other.pitch;
    yaw -= other.yaw;
    return *this;
  }

  GimbalCmd operator*=(float scalar)
  {
    pitch *= scalar;
    yaw *= scalar;
    return *this;
  }

  friend GimbalCmd operator*(float scalar, const GimbalCmd &cmd);
};
class Gimbal : public Fsm
{
 public:
  typedef hello_world::filter::Td Td;
  typedef hello_world::motor::DM_J4310 Motor;
  typedef hello_world::PeriodAngle2ContAngleRad p2c;
  typedef hello_world::pid::MultiNodesPid Pid;

  typedef hello_world::imu::Imu Imu;
  typedef hello_world::laser::Laser Laser;
  typedef GimbalWorkingMode WorkingMode;
  typedef GimbalCmd Cmd;

  struct Config {
    float sensitivity_yaw;      ///< yaw角度灵敏度，单位 rad/ms
    float sensitivity_pitch;    ///< pitch角度灵敏度，单位 rad/ms
    float max_pitch_ang;        ///< 最大俯仰角度，单位 rad
    float min_pitch_ang;        ///< 最小俯仰角度，单位 rad
    /* Pitch重力前馈 */
    float max_pitch_torq;       ///< 云台最大的重力矩，单位 N·m
    float pitch_center_offset;  ///< 云台水平时，重心和pitch轴的连线与水平轴的夹角，单位 rad
    /* 电机阻力前馈 */
    float resist_ffd_torq[2];  ///< 云台电机阻力前馈力矩，单位 N·m
    float allowed_ang_err[2];  ///< 云台角度的允许误差（用于分段计算阻力前馈力矩）
  };

  struct VisionData {
    Cmd cmd = {0.0f, 0.0f};
    bool is_target_detected = true;
  };

  enum JointIdx : uint8_t {
    kJointPitch = 0u, 
    kJointYaw = 1U,
    kJointNum = 2U,
  };

  enum class CtrlAngBased : uint8_t {
    Imu,    ///< 基于IMU的角度控制
    Motor,  ///< 基于关节角度的角度控制
  };

  Gimbal(Config config) { cfg_ = config; };
  ~Gimbal(){};

  void update() override;

  void run() override;

  void reset() override;

  void standby() override;

  void setRevHeadFlag(bool flag)
  {
    rev_command_flag_ = flag;
    if (rev_command_flag_ != last_rev_command_flag_) {
      rev_head_flag_ = (work_tick_ - last_rev_head_tick_ > 200);
      last_rev_head_tick_ = work_tick_;
    }
    last_rev_command_flag_ = rev_command_flag_;
  }
  bool getRevHeadFlag() const { return rev_head_flag_; }

  void setNavigationFlag(bool flag) {navigation_flag_ = flag;}
  bool getNavigationFlag() const {return navigation_flag_;}

  void setVisionTargetDetected(bool flag) { vis_data_.is_target_detected = flag; }
  void updateIsRfrPwrOn(bool flag) { is_rfr_pwr_on_ = flag; }

  void setNormCmdDelta(const Cmd &cmd) { norm_cmd_delta_ = cmd; }
  void setNormCmdDelta(float yaw, float pitch)
  {
    norm_cmd_delta_.yaw = yaw;
    norm_cmd_delta_.pitch = pitch;
  }
  const Cmd &getNormCmdDelta() const { return norm_cmd_delta_; }

  void setVisionCmd(const Cmd &cmd) { vis_data_.cmd = cmd; }
  void setVisionCmd(float yaw, float pitch)
  {
    vis_data_.cmd.yaw = yaw;
    vis_data_.cmd.pitch = pitch;
  }

  float getJointYawAngFdb() const { return joint_ang_fdb_[kJointYaw]; }
  float getJointPitchAngFdb() const { return joint_ang_fdb_[kJointPitch]; }
  float getJointRollAngFdb() const
  {
    HW_ASSERT(imu_ptr_ != nullptr, "IMU pointer is nullptr", imu_ptr_);
    return imu_ptr_->roll();
  }

  void setCtrlMode(CtrlMode mode) { ctrl_mode_ = mode; }
  CtrlMode getCtrlMode() const { return ctrl_mode_; }

  void setWorkingMode(WorkingMode mode) { working_mode_ = mode; }
  WorkingMode getWorkingMode() const { return working_mode_; }
  // 注册组件指针
  void registerMotor(Motor *ptr, JointIdx idx);
  void registerPid(Pid *ptr, JointIdx idx);
  void registerTd(Td *ptr, size_t idx);
  void registerImu(Imu *ptr);

 private:
  //  数据更新
  void updateData();
  void updateMotorData();
  void updateImuData();
  void updateIsPwrOn();
  void updatePwrState();

  // 任务执行
  void runOnDead();
  void runOnResurrection();
  void runOnWorking();

  void calcCtrlAngBased();
  void adjustJointFdb();
  void calcCruiseMode(Cmd &tmp_ang_ref);
  float normalize_angle(float angle);
  float get_shortest_angle_diff(float from, float to);
  void adjustLastJointAngRef();
  void calcJointAngRef();
  void calcJointTorRef();
  float calcJointFfdResistance(JointIdx idx);

  // 数据重置
  void resetDataOnDead();
  void resetDataOnResurrection();
  void resetPids();

  // 设置通讯数据
  void setCommData(bool working_flag) { setCommDataMotors(working_flag); };
  void setCommDataMotors(bool working_flag);

  // 由 robot 设置的数据
  bool rev_command_flag_ = false;  ///< 云台是否接收到翻转头部的指令
  bool last_rev_command_flag_ = false;  ///< 上一翻转头部指令
  bool rev_head_flag_ = false;  ///< 翻转头部朝向标志位
  bool last_rev_head_flag_ = false;  ///< 上一翻转头部朝向标志位
  bool navigation_flag_ = false;  ///< 云台巡航标志位
  bool is_rfr_pwr_on_ = false;  ///< 裁判系统电源管理 gimbal 是否输出

  Cmd norm_cmd_delta_ = {0.0, 0.0};  ///< 控制指令的增量
  VisionData vis_data_;

  CtrlMode ctrl_mode_ = CtrlMode::Manual;  ///< 控制模式

  WorkingMode working_mode_ = WorkingMode::Normal;  ///< 工作模式

  // 由 Gimbal 内部维护的数据
  Config cfg_;  ///< 配置参数

  bool is_rotating_ = false;  // 新增正在转头标志位

  bool is_pwr_on_ = false;  ///< 电源是否开启

  uint32_t last_rev_head_tick_ = 0;  ///< 上一次翻转头部朝向的时间戳

  // 控制电机的 PID 所需数据
  CtrlAngBased ctrl_ang_based_[kJointNum] = {CtrlAngBased::Imu, CtrlAngBased::Imu};       ///< 角度控制方式
  CtrlAngBased last_ctrl_ang_based_[kJointNum] = {CtrlAngBased::Imu, CtrlAngBased::Imu};  ///< 上一控制周期的角度控制方式

  float last_joint_ang_ref_[kJointNum] = {0.0f};  ///< 上一控制周期的关节角度期望值
  float joint_ang_ref_[kJointNum] = {0.0f};       ///< 关节角度期望值
  float joint_ang_fdb_[kJointNum] = {0.0f};       ///< 关节角度反馈值
  float joint_spd_fdb_[kJointNum] = {0.0f};       ///< 关节角速度反馈值
  float joint_tor_ref_[kJointNum] = {0.0f};       ///< 关节扭矩期望值
  float joint_tor_ffd_[kJointNum] = {0.0f};       ///< 关节扭矩前馈值
  float pitch_spd_ref_ = 0.0;

  // 从电机中拿到的数据
  bool is_any_motor_pwron_ = false;  ///< 是否有电机上电
  bool is_all_motor_pwron_ = false;  ///< 是否所有电机都上电

  float motor_ang_fdb_[kJointNum] = {0.0f};  ///< 电机角度反馈值
  float motor_spd_fdb_[kJointNum] = {0.0f};  ///< 电机角速度反馈值

  // 从 IMU 拿到的数据
  float imu_ang_fdb_[kJointNum] = {0.0f};  ///< IMU 角度反馈值
  float imu_spd_fdb_[kJointNum] = {0.0f};  ///< IMU 角速度反馈值

  // 各组件指针
  // 无通信功能的组件指针
  Pid *pid_ptr_[kJointNum] = {nullptr};          ///< PID 指针
  Td *motor_spd_td_ptr_[kJointNum] = {nullptr};  ///< 电机速度滤波器指针

  // 只接收数据的组件指针
  Imu *imu_ptr_ = nullptr;  ///< IMU 指针 只接收数据
  // Laser *laser_ptr_ = nullptr;                   ///< 激光指针

  // 接收、发送数据的组件指针
  Motor *motor_ptr_[kJointNum] = {nullptr};  ///< 电机指针 接收、发送数据
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
inline GimbalCmd operator*(float scalar, const GimbalCmd &cmd) { return {cmd.pitch * scalar, cmd.yaw * scalar}; }
}  // namespace robot
#endif /* ROBOT_MODULES_GIMBAL_HPP_ */
