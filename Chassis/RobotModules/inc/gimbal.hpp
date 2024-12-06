/** 
 *******************************************************************************
 * @file      : gimbal.hpp
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

  void setNormCmdDelta(const Cmd &cmd) { norm_cmd_delta_ = cmd; }
  const Cmd &getNormCmdDelta() const { return norm_cmd_delta_; }

  void setCtrlMode(CtrlMode mode) { ctrl_mode_ = mode; }
  CtrlMode getCtrlMode() const { return ctrl_mode_; }

  void setWorkingMode(WorkingMode mode) { working_mode_ = mode; }
  WorkingMode getWorkingMode() const { return working_mode_; }

 private:
  // 由 robot 设置的数据

  Cmd norm_cmd_delta_ = {0.0, 0.0};                 ///< 控制指令的增量
  CtrlMode ctrl_mode_ = CtrlMode::Manual;           ///< 控制模式
  WorkingMode working_mode_ = WorkingMode::Normal;  ///< 工作模式

  bool rev_head_flag_ = false;       ///< 翻转头部朝向标志位
  uint32_t last_rev_head_tick_ = 0;  ///< 上一次翻转头部朝向的时间戳
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
inline GimbalCmd operator*(float scalar, const GimbalCmd &cmd) { return {cmd.pitch * scalar, cmd.yaw * scalar}; }
}  // namespace robot
#endif /* ROBOT_MODULES_GIMBAL_HPP_ */
