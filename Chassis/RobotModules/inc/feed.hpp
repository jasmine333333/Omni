/** 
 *******************************************************************************
 * @file      : feed.hpp
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
#ifndef ROBOT_MODULES_FEED_HPP_
#define ROBOT_MODULES_FEED_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "module_state.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Feed
{
 public:
  typedef ShooterWorkingMode WorkingMode;

  Feed() {};
  ~Feed() {};

  void setCtrlMode(CtrlMode mode) { ctrl_mode_ = mode; }
  CtrlMode getCtrlMode() const { return ctrl_mode_; }

  void setWorkingMode(WorkingMode mode) { working_mode_ = mode; }
  WorkingMode getWorkingMode() const { return working_mode_; }

  void setShootFlag(bool flag) { shoot_flag_ = flag; }
  void shoot() { setShootFlag(true); }
  void clearShootFlag() { setShootFlag(false); }
  bool getShootFlag() const { return shoot_flag_; }
 private:
  // 由 robot 设置的数据
  bool shoot_flag_ = false;

  CtrlMode ctrl_mode_ = CtrlMode::Manual;
  WorkingMode working_mode_ = WorkingMode::Normal;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* ROBOT_MODULES_FEED_HPP_ */
