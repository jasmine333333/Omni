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
#include "module_state.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Scope
{
 public:
  typedef ScopeWorkingMode WorkingMode;

  Scope(){};
  ~Scope(){};

  CtrlMode getCtrlMode() const { return ctrl_mode_; }
  void setCtrlMode(CtrlMode mode) { ctrl_mode_ = mode; }

  WorkingMode getWorkingMode() const { return working_mode_; }
  void setWorkingMode(WorkingMode mode) { working_mode_ = mode; }

  bool getCtrlAngleFlag() const { return ctrl_angle_flag_; }
  void setCtrlAngleFlag(bool flag) { ctrl_angle_flag_ = flag; }

  bool getSwitchScopeFlag() const { return switch_scope_flag_; }
  void setSwitchScopeFlag(bool flag) { switch_scope_flag_ = flag; }
  void enableScope() { setSwitchScopeFlag(true); }
  void disableScope() { setSwitchScopeFlag(false); }

 private:
  //  由 robot 设置的数据
  bool ctrl_angle_flag_ = false;    ///< 控制角度标志位
  bool switch_scope_flag_ = false;  ///< 开关倍镜标志位

  CtrlMode ctrl_mode_ = CtrlMode::Manual;           ///< 控制模式
  WorkingMode working_mode_ = WorkingMode::Normal;  ///< 当前工作模式
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* ROBOT_MODULES_SCOPE_HPP_ */
