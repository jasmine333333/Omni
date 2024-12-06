/** 
 *******************************************************************************
 * @file      : module_state.hpp
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
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
#include <string>

/* Exported macro ------------------------------------------------------------*/

namespace robot
{
/* Exported constants --------------------------------------------------------*/
// 通用状态枚举

/**
 * @brief 电源状态
 * 
 * 定义了与机器人模块相关的电源状态，包括死亡状态（断电）、复活状态（上电后的初始状态）和工作状态。
 */
enum class PwrState : uint8_t {
  Dead = 0u,     ///< 死亡状态
  Resurrection,  ///< 复活状态
  Working,       ///< 工作状态
};

/** 控制模式 */
enum class CtrlMode : uint8_t {
  Manual,  ///< 手动控制模式
  Auto,    ///< 自动控制模式
};
/** 手动控制源 */
enum class ManualCtrlSrc : uint8_t {
  Rc,  ///< 遥控器操控
  Kb,  ///< 键盘操控
};

/** 底盘工作模式 */
enum class ChassisWorkingMode : uint8_t {
  Depart,    ///< 分离模式
  Follow,    ///< 随动模式
  Gyro,      ///< 小陀螺模式
  Farshoot,  ///< 吊射模式
};

/** 云台工作模式 */
enum class GimbalWorkingMode {
  Normal,    ///< 正常模式
  Farshoot,  ///< 吊射模式
};
/** 发射机构工作模式 */
enum class ShooterWorkingMode : uint8_t {
  Normal,        ///< 正常模式
  Crazy,         ///< 嗜血模式
  FricBackward,  ///< 摩擦轮倒转模式
  Stop,          ///< 摩擦轮停转模式
};
/** 倍镜云台工作模式 */
enum class ScopeWorkingMode : uint8_t {
  Normal,    ///< 正常模式
  Farshoot,  ///< 吊射模式
};

/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
inline std::string PwrStateToStr(PwrState state)
{
  if (state == PwrState::Dead) return "Dead";
  if (state == PwrState::Resurrection) return "Resurrection";
  if (state == PwrState::Working) return "Working";
  return "ErrWS";
};

inline std::string CtrlModeToStr(CtrlMode mode)
{
  if (mode == CtrlMode::Manual) return "Manual";
  if (mode == CtrlMode::Auto) return "Auto";
  return "ErrCM";
};

inline std::string ManualCtrlSrcToStr(ManualCtrlSrc src)
{
  if (src == ManualCtrlSrc::Rc) return "Rc";
  if (src == ManualCtrlSrc::Kb) return "Kb";
  return "ErrMCS";
};

inline std::string CtrlModeSrcToStr(CtrlMode mode, ManualCtrlSrc src)
{
  if (mode == CtrlMode::Manual) return ManualCtrlSrcToStr(src);
  if (mode == CtrlMode::Auto) return CtrlModeToStr(mode);
  return "ErrCM";
};

inline std::string ChassisWorkingModeToStr(ChassisWorkingMode mode)
{
  if (mode == ChassisWorkingMode::Depart) return "Depart";
  if (mode == ChassisWorkingMode::Follow) return "Follow";
  if (mode == ChassisWorkingMode::Gyro) return "Gyro";
  if (mode == ChassisWorkingMode::Farshoot) return "Farshoot";
  return "ErrCWM";
};

inline std::string GimbalWorkingModeToStr(GimbalWorkingMode mode)
{
  if (mode == GimbalWorkingMode::Normal) return "Normal";
  if (mode == GimbalWorkingMode::Farshoot) return "Farshoot";
  return "ErrGWM";
};

inline std::string ShooterWorkingModeToStr(ShooterWorkingMode mode)
{
  if (mode == ShooterWorkingMode::Normal) return "Normal";
  if (mode == ShooterWorkingMode::Crazy) return "Crazy";
  if (mode == ShooterWorkingMode::FricBackward) return "FricBackward";
  return "ErrSWM";
};

inline std::string ScopeWorkingModeToStr(ScopeWorkingMode mode)
{
  if (mode == ScopeWorkingMode::Normal) return "Normal";
  if (mode == ScopeWorkingMode::Farshoot) return "Farshoot";
  return "ErrSGWM";
}
}  // namespace robot
#endif /* MODULE_STATE_HPP_ */
