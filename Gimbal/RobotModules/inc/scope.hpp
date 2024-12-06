/** 
 *******************************************************************************
 * @file      : scope.hpp
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
#ifndef ROBOT_MODULES_SCOPE_HPP_
#define ROBOT_MODULES_SCOPE_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "module_fsm.hpp"
#include "module_state.hpp"
#include "servo.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Scope : public robot::Fsm
{
 public:
  typedef ScopeWorkingMode WorkingMode;

  typedef hello_world::servo::Servo Servo;

  enum class ServoIdx : uint8_t {
    Pitch,
    Scope,
    Num,
  };

  struct Config {
    float pitch_sensitivity = 1.0f;  ///< pitch 灵敏度
    float pitch_normal_ang = 90.0f;
    float min_pitch_ang = 60.0f;         ///< pitch 最小角度，单位：Deg
    float max_pitch_ang = 90.0f;         ///< pitch 最大角度，单位：Deg
    float using_scope_ang = 0.0f;        ///< 使用倍镜时舵机的角度
    float not_using_scope_ang = 130.0f;  ///< 不使用倍镜时舵机的角度
  };

  Scope(const Config &config) { cfg_ = config; };
  ~Scope(){};

  void update() override;

  void run() override;

  void reset() override;

  void standby() override;

  WorkingMode getWorkingMode() const { return working_mode_; }
  void setWorkingMode(WorkingMode mode)
  {
    if (working_mode_ != mode) {
      last_working_mode_ = working_mode_;
      working_mode_ = mode;
    }
  }

  bool getCtrlAngleFlag() const { return ctrl_angle_flag_; }
  void setCtrlAngleFlag(bool flag) { ctrl_angle_flag_ = flag && (working_mode_ == WorkingMode::Farshoot); }

  bool getSwitchScopeFlag() const { return switch_scope_flag_; }
  void setSwitchScopeFlag(bool flag) { switch_scope_flag_ = flag && (working_mode_ == WorkingMode::Farshoot); }
  void enableScope() { setSwitchScopeFlag(true); }
  void disableScope() { setSwitchScopeFlag(false); }

  void setNormPitchDelta(float delta) { norm_pitch_delta_ = (working_mode_ == WorkingMode::Farshoot) ? delta : 0.0f; }

  // 注册组件指针
  void registerServo(Servo *ptr, ServoIdx idx);

 private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updatePwrState();

  // 执行任务
  void runOnDead();
  void runOnResurrection();
  void runOnWorking();

  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();

  //  由 robot 设置的数据
  bool ctrl_angle_flag_ = false;    ///< 控制角度标志位
  bool switch_scope_flag_ = false;  ///< 开关倍镜标志位

  WorkingMode working_mode_ = WorkingMode::Normal;       ///< 当前工作模式
  WorkingMode last_working_mode_ = WorkingMode::Normal;  ///< 上一次工作模式

  float norm_pitch_delta_ = 0.0f;  ///< 相对角度增量

  // 由内部管理的数据
  Config cfg_;  ///< 配置参数

  bool use_scope_flag_ = false;  ///< 是否使用倍镜

  uint32_t last_switch_scope_tick_ = 0;  ///< 上一次开关倍镜的时间戳

  float pitch_ang_ref_ = 0.0f;       ///< pitch 角度期望值，单位：Deg
  float pitch_ang_recorded_ = 0.0f;  ///< 记录下的 pitch 角度，单位：Deg

  float scope_ang_ref_ = 0.0f;

  // 各组件指针
  // 无通信功能的组件指针
  // 只接收数据的组件指针
  // 只发送数据的组件指针
  Servo *servo_ptr_[(uint8_t)ServoIdx::Num] = {nullptr};  ///< 舵机指针 接收、发送数据
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* ROBOT_MODULES_SCOPE_HPP_ */
