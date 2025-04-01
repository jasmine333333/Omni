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
#include "allocator.hpp"
#include "module_state.hpp"
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
class Gimbal : public hello_world::MemMgr
{
 public:
  typedef GimbalWorkingMode WorkingMode;
  typedef GimbalCmd Cmd;
  Gimbal(){};
  ~Gimbal(){};

  void setRevHeadFlag(bool flag) { rev_head_flag_ = flag; }
  bool getRevHeadFlag() const { return rev_head_flag_; }
  
  bool setnavigateFlag(bool flag) {navigate_flag_ = flag;}
  bool getnavigateFlag() const { return navigate_flag_; }

  void setRevChassisFlag(bool flag) { rev_chassis_flag_ = flag; }
  bool getRevChassisFlag() const { return rev_chassis_flag_; }


  void setNormCmdDelta(const Cmd &cmd) { norm_cmd_delta_ = cmd; }
  const Cmd &getNormCmdDelta() const { return norm_cmd_delta_; }

  void setCtrlMode(CtrlMode mode) { ctrl_mode_ = mode; }
  CtrlMode getCtrlMode() const { return ctrl_mode_; }

  void setWorkingMode(WorkingMode mode) { working_mode_ = mode; }
  WorkingMode getWorkingMode() const { return working_mode_; }

  static std::string WorkStateToStr(PwrState state)
  {
    if (state == PwrState::Dead) return "Dead";
    if (state == PwrState::Resurrection) return "Resurrection";
    if (state == PwrState::Working) return "Working";
    return "ErrWS";
  };
  static std::string WorkingModeToStr(WorkingMode mode)
  {
    if (mode == WorkingMode::Normal) return "Normal";
    return "ErrGWM";
  };
  static std::string CtrlModeSrcToStr(CtrlMode mode, ManualCtrlSrc src)
  {
    if (mode == CtrlMode::Manual) return ManualCtrlSrcToStr(src);
    if (mode == CtrlMode::Auto) return CtrlModeToStr(mode);
    return "ErrCM";
  };

 private:
  // 由 robot 设置的数据

  Cmd norm_cmd_delta_ = {0.0, 0.0};                 ///< 控制指令的增量
  CtrlMode ctrl_mode_ = CtrlMode::Manual;           ///< 控制模式
  WorkingMode working_mode_ = WorkingMode::Normal;  ///< 工作模式

  bool rev_head_flag_ = false;       ///< 翻转头部朝向标志位
  uint32_t last_rev_head_tick_ = 0;  ///< 上一次翻转头部朝向的时间戳
  bool navigate_flag_ = false;       ///< 是否导航模式

  bool rev_chassis_flag_ = false;  ///< 翻转底盘朝向标志位
  uint32_t last_rev_chassis_tick_ = 0;  ///< 上一次翻转底盘朝向的时间戳
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
inline GimbalCmd operator*(float scalar, const GimbalCmd &cmd) { return {cmd.pitch * scalar, cmd.yaw * scalar}; }
}  // namespace robot
#endif /* ROBOT_MODULES_GIMBAL_HPP_ */
