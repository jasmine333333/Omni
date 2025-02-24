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
#ifndef ROBOT_MODULES_SHOOTER_HPP_
#define ROBOT_MODULES_SHOOTER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "module_state.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Shooter : public hello_world::MemMgr
{
 public:
  typedef ShooterWorkingMode WorkingMode;

  Shooter(){};
  ~Shooter(){};

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
  WorkingMode working_mode_ = WorkingMode::Normal;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* ROBOT_MODULE_SHOOTER_HPP_ */
