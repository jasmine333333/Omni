/** 
 *******************************************************************************
 * @file      :module_fsm_private.hpp
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
#ifndef ROBOT_MODULE_FSM_HPP_
#define ROBOT_MODULE_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "module_state.hpp"
#include "tick.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot
{
/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Fsm : public hello_world::MemMgr
{
 public:
  /** 
    * @brief 任务执行函数
    * 
    * 负责根据当前状态和控制模式，执行相应的动作
    */
  virtual void run() = 0;
  /** 
   * @brief 进入待机状态
   * 
   * 进入待机状态，停止所有动作
   */
  virtual void standby() = 0;
  /** 
   * @brief 更新状态机
   * 
   * 更新内部状态数据和状态机状态
   */
  virtual void update() = 0;
  /** 
   * @brief 复位状态机  
   * 
   * 复位状态机，恢复初始状态
   */
  virtual void reset() = 0;

  // 接口函数
  virtual PwrState getPwrState() const { return pwr_state_; };
  virtual PwrState getLastPwrState() const { return last_pwr_state_; };

  static std::string WorkStateToStr(PwrState state)
  {
    if (state == PwrState::Dead) return "Dead";
    if (state == PwrState::Resurrection) return "Resurrection";
    if (state == PwrState::Working) return "Working";
    return "ErrWS";
  };

  static std::string CtrlModeToStr(CtrlMode mode)
  {
    if (mode == CtrlMode::Manual) return "Manual";
    if (mode == CtrlMode::Auto) return "Auto";
    return "ErrCM";
  };

  static std::string ManualCtrlSrcToStr(ManualCtrlSrc src)
  {
    if (src == ManualCtrlSrc::Rc) return "Rc";
    if (src == ManualCtrlSrc::Kb) return "Kb";
    return "ErrMCS";
  };

  static std::string AutoCamSrcToStr(AutoCamSrc src)
  {
    if (src == kAutoCamNormal) return "Normal";
    if (src == kAutoCamLongFocal) return "LongFocal";
    return "ErrACS";
  };

  static std::string CtrlModeSrcToStr(CtrlMode mode, ManualCtrlSrc src)
  {
    if (mode == CtrlMode::Manual) return ManualCtrlSrcToStr(src);
    if (mode == CtrlMode::Auto) return CtrlModeToStr(mode);
    return "ErrCM";
  };

 protected:
  // 工具函数
  uint32_t getCurrentTickMs() const { return hello_world::tick::GetTickMs(); };
  uint32_t updateWorkTick()
  {
    last_work_tick_ = work_tick_;
    work_tick_ = getCurrentTickMs();
    interval_ticks_ = work_tick_ - last_work_tick_;
    return work_tick_;
  }

  void setPwrState(PwrState state)
  {
    if (state != pwr_state_) {
      last_pwr_state_ = pwr_state_;
      pwr_state_ = state;
    }
  };

  uint32_t work_tick_ = 0;       ///< 工作时钟，单位：ms
  uint32_t last_work_tick_ = 0;  ///< 上一工作时钟，单位：ms
  uint32_t interval_ticks_ = 0;  ///< 状态间隔时钟，单位：ms

  PwrState pwr_state_ = PwrState::Dead;       ///< 电源状态
  PwrState last_pwr_state_ = PwrState::Dead;  ///< 上一电源状态
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace robot
#endif /* ROBOT_MODULE_FSM_HPP_ */
