/** 
 *******************************************************************************
 * @file      :ui_drawer.hpp
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
#ifndef UI_DRAWER_HPP_
#define UI_DRAWER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "fsm.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"
#include "rfr_official_pkgs.hpp"
#include "rfr_encoder.hpp"
#include "module_fsm_private.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class UiDrawer
{
 public:
  typedef hello_world::referee::GraphicOperation GraphicOperation;
  typedef hello_world::referee::ids::RobotId RobotId;
  typedef hello_world::referee::RfrEncoder RfrEncoder;
  typedef hello_world::referee::String String;
  typedef robot::Chassis::WorkingMode ChassisWorkingMode;
  typedef robot::CtrlMode FsmCtrlMode;
  typedef robot::ManualCtrlSrc FsmManualCtrlSrc;
  typedef robot::PwrState FsmWorkState;
  typedef robot::Gimbal::WorkingMode GimbalWorkingMode;
  typedef robot::Shooter::WorkingMode ShooterWorkingMode;
  
  
  enum StaticUiIdx {
    kSuiDelAll = 0,
    kSuiPassLinePkgGroup1,
    kSuiPassLinePkgGroup2,
    kSuiChassisTitle,
    kSuiGimbalTitle,
    kSuiShooterTitle,
    kSuiScopeTitle,
    kSuiFeedAngTitle,
    kSuiFricSpdTitle,
    kSuiScopeAngTitle,
    kSuiPitchAngTitle,
    kSuiYawAngTitle,
    kSuiPkgNum,
  };

  enum DynamicUiIdx {
    kDuiChassisContent,
    kDuiGimbalContent,
    kDuiShooterContent,
    kDuiScopeContent,
    kDuiPkgGroup1,  ///< 云台 pitch yaw 角度反馈，拨盘角度反馈，摩擦轮转速反馈，超电，底盘朝向(2)
    kDuiPkgGroup2,  ///< 云台 pitch yaw 角度期望，拨盘角度期望，摩擦轮转速期望，小云台预设值，小云台当前俯仰角度
    kDuiPkgGroup3,
    kDuiPkgGroup4,
    kDuiPkgNum,
  };

  static constexpr size_t kNumAllPkgs = (size_t)kDuiPkgNum + (size_t)kSuiPkgNum;

  UiDrawer(){};
  ~UiDrawer(){};

  void refresh()
  {
    ui_idx_ = 0;
    n_added_ = 0;
  };
  bool encode(uint8_t* data_ptr, size_t& data_len);

#pragma region 接口函数
  void setSenderId(RobotId id) { sender_id_ = id; }

  void setChassisWorkState(FsmWorkState state)
  {
    if (state != chassis_work_state_) {
      last_chassis_work_state_ = chassis_work_state_;
      chassis_work_state_ = state;
    }
  }
  void setChassisCtrlMode(FsmCtrlMode mode)
  {
    if (mode != chassis_ctrl_mode_) {
      last_chassis_ctrl_mode_ = chassis_ctrl_mode_;
      chassis_ctrl_mode_ = mode;
    }
  }
  void setChassisManualCtrlSrc(FsmManualCtrlSrc src)
  {
    if (src != chassis_manual_ctrl_src_) {
      last_chassis_manual_ctrl_src_ = chassis_manual_ctrl_src_;
      chassis_manual_ctrl_src_ = src;
    }
  }
  void setChassisWorkingMode(ChassisWorkingMode mode)
  {
    if (mode != chassis_working_mode_) {
      last_chassis_working_mode_ = chassis_working_mode_;
      chassis_working_mode_ = mode;
    }
  }
  void setChassisHeadDir(float theta_i2r) { theta_i2r_ = theta_i2r; }

  void setGimbalWorkState(FsmWorkState state)
  {
    if (state != gimbal_work_state_) {
      last_gimbal_work_state_ = gimbal_work_state_;
      gimbal_work_state_ = state;
    }
  }
  void setGimbalCtrlMode(FsmCtrlMode mode)
  {
    if (mode != gimbal_ctrl_mode_) {
      last_gimbal_ctrl_mode_ = gimbal_ctrl_mode_;
      gimbal_ctrl_mode_ = mode;
    }
  }
  void setGimbalManualCtrlSrc(FsmManualCtrlSrc src)
  {
    if (src != gimbal_manual_ctrl_src_) {
      last_gimbal_manual_ctrl_src_ = gimbal_manual_ctrl_src_;
      gimbal_manual_ctrl_src_ = src;
    }
  }
  void setGimbalWorkingMode(GimbalWorkingMode mode)
  {
    if (mode != gimbal_working_mode_) {
      last_gimbal_working_mode_ = gimbal_working_mode_;
      gimbal_working_mode_ = mode;
    }
  }
  void setGimbalJointAngPitchFdb(float pitch) { gimbal_joint_ang_pitch_fdb_ = pitch; }
  void setGimbalJointAngPitchRef(float pitch) { gimbal_joint_ang_pitch_ref_ = pitch; }
  void setGimbalJointAngYawFdb(float yaw) { gimbal_joint_ang_yaw_fdb_ = yaw; }
  void setGimbalJointAngYawRef(float yaw) { gimbal_joint_ang_yaw_ref_ = yaw; }

  void setShooterWorkState(FsmWorkState state)
  {
    if (state != shooter_work_state_) {
      last_shooter_work_state_ = shooter_work_state_;
      shooter_work_state_ = state;
    }
  }
  void setShooterCtrlMode(FsmCtrlMode mode)
  {
    if (mode != shooter_ctrl_mode_) {
      last_shooter_ctrl_mode_ = shooter_ctrl_mode_;
      shooter_ctrl_mode_ = mode;
    }
  }
  void setShooterManualCtrlSrc(FsmManualCtrlSrc src)
  {
    if (src != shooter_manual_ctrl_src_) {
      last_shooter_manual_ctrl_src_ = shooter_manual_ctrl_src_;
      shooter_manual_ctrl_src_ = src;
    }
  }
  void setShooterWorkingMode(ShooterWorkingMode mode)
  {
    if (mode != shooter_working_mode_) {
      last_shooter_working_mode_ = shooter_working_mode_;
      shooter_working_mode_ = mode;
    }
  }
  void setHeat(float heat) { heat_ = heat; }
  void setHeatLimit(float limit) { heat_limit_ = limit; }
  void setFeedAngFdb(float ang) { feed_ang_fdb_ = ang; }
  void setFeedAngRef(float ang) { feed_ang_ref_ = ang; }
  void setFeedStuckFlag(bool flag)
  {
    if (flag != feed_stuck_flag_) {
      last_feed_stuck_flag_ = feed_stuck_flag_;
      feed_stuck_flag_ = flag;
    }
  }
  void setFricSpdFdb(float spd) { fric_spd_fdb_ = spd; }
  void setFricSpdRef(float spd) { fric_spd_ref_ = spd; }
  void setFricStuckFlag(bool flag)
  {
    if (flag != fric_stuck_flag_) {
      last_fric_stuck_flag_ = fric_stuck_flag_;
      fric_stuck_flag_ = flag;
    }
  }


  void setCapPwrPercent(float percent) { cap_pwr_percent_ = percent; }

  void setVisTgtX(uint16_t x, bool valid) { vis_tgt_x_ = valid ? x : -1; }
  void setVisTgtY(uint16_t y, bool valid) { vis_tgt_y_ = valid ? y : -1; }

  void setisvisionvalid(bool isvisionvalid) { is_vision_valid_ = isvisionvalid; }
#pragma endregion
 private:
  template <typename T>
  bool encodePkg(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt, T& pkg)
  {
    pkg.setSenderId(static_cast<uint16_t>(sender_id_));
    return encoder_.encodeFrame(&pkg, data_ptr, &data_len);
  };

  bool encodeString(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt, String& g, std::string& str)
  {
    g.setOperation(opt);
    hello_world::referee::InterGraphicStringPackage pkg;
    pkg.setStrintg(g, str);
    return encodePkg(data_ptr, data_len, opt, pkg);
  };

  bool encodeStaticUi(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt, StaticUiIdx idx);
  bool encodeDynamicUi(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt, DynamicUiIdx idx);

  bool encodeDelAll(uint8_t* data_ptr, size_t& data_len);
  bool encodeChassisWorkStateTitle(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeChassisWorkStateContent(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeGimbalWorkStateTitle(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeGimbalWorkStateContent(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeShooterWorkStateTitle(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeShooterWorkStateContent(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);

  bool encodeFeedTitle(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeFricTitle(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeGimbalPitchTitle(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeGimbalYawTitle(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);

  bool encodeStaticPkgGroup1(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeStaticPkgGroup2(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);

  bool encodeDynaUiPkgGroup1(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeDynaUiPkgGroup2(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeDynaUiPkgGroup3(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeDynaUiPkgGroup4(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);

  void genChassisStatus(hello_world::referee::Arc& g_head, hello_world::referee::Arc& g_other);
  void genChassisPassLineLeft(hello_world::referee::StraightLine& g);
  void genChassisPassLineRight(hello_world::referee::StraightLine& g);

  void genGimbalJointAngPitchFdb(hello_world::referee::FloatingNumber& g);
  void genGimbalJointAngPitchRef(hello_world::referee::FloatingNumber& g);
  void genGimbalJointAngYawFdb(hello_world::referee::FloatingNumber& g);
  void genGimbalJointAngYawRef(hello_world::referee::FloatingNumber& g);
  void genPassSafe(hello_world::referee::Circle& g, bool is_safe);

  void genShooterHeat(hello_world::referee::Arc& g);
  void genShooterFeedAngFdb(hello_world::referee::FloatingNumber& g);
  void genShooterFeedAngFdb(hello_world::referee::Integer& g);
  void genShooterFeedAngRef(hello_world::referee::FloatingNumber& g);
  void genShooterFeedAngRef(hello_world::referee::Integer& g);
  void genShooterFricSpdFdb(hello_world::referee::FloatingNumber& g);
  void genShooterFricSpdFdb(hello_world::referee::Integer& g);
  void genShooterFricSpdRef(hello_world::referee::FloatingNumber& g);
  void genShooterFricSpdRef(hello_world::referee::Integer& g);



  void genCapPwrPercent(hello_world::referee::Rectangle& g_rect, hello_world::referee::FloatingNumber& g_num);

  void genVisTgt(hello_world::referee::Circle& g);
  void genVisionbox1(hello_world::referee::StraightLine& g_rect);
  void genVisionbox2(hello_world::referee::StraightLine& g_rect);
  void genVisionbox3(hello_world::referee::StraightLine& g_rect);
  void genVisionbox4(hello_world::referee::StraightLine& g_rect);

  // encode
  size_t ui_idx_ = 0;
  size_t n_added_ = 0;
  RobotId sender_id_ = RobotId::kBlueStandard3;
  RfrEncoder encoder_;

  // var for chassis
  FsmWorkState chassis_work_state_ = FsmWorkState::Dead;
  FsmWorkState last_chassis_work_state_ = FsmWorkState::Dead;
  FsmCtrlMode chassis_ctrl_mode_ = FsmCtrlMode::Manual;
  FsmCtrlMode last_chassis_ctrl_mode_ = FsmCtrlMode::Manual;
  FsmManualCtrlSrc chassis_manual_ctrl_src_ = FsmManualCtrlSrc::Rc;
  FsmManualCtrlSrc last_chassis_manual_ctrl_src_ = FsmManualCtrlSrc::Rc;
  ChassisWorkingMode chassis_working_mode_ = ChassisWorkingMode::Depart;
  ChassisWorkingMode last_chassis_working_mode_ = ChassisWorkingMode::Depart;
  float theta_i2r_ = 0.0f;

  // var for gimbal
  FsmWorkState gimbal_work_state_ = FsmWorkState::Dead;
  FsmWorkState last_gimbal_work_state_ = FsmWorkState::Dead;
  FsmCtrlMode gimbal_ctrl_mode_ = FsmCtrlMode::Manual;
  FsmCtrlMode last_gimbal_ctrl_mode_ = FsmCtrlMode::Manual;
  FsmManualCtrlSrc gimbal_manual_ctrl_src_ = FsmManualCtrlSrc::Rc;
  FsmManualCtrlSrc last_gimbal_manual_ctrl_src_ = FsmManualCtrlSrc::Rc;
  GimbalWorkingMode gimbal_working_mode_ = GimbalWorkingMode::Normal;
  GimbalWorkingMode last_gimbal_working_mode_ = GimbalWorkingMode::Normal;
  float gimbal_joint_ang_pitch_fdb_ = 0.0f, gimbal_joint_ang_pitch_ref_ = 0.0f;
  float gimbal_joint_ang_yaw_fdb_ = 0.0f, gimbal_joint_ang_yaw_ref_ = 0.0f;

  // var for shooter
  FsmWorkState shooter_work_state_ = FsmWorkState::Dead;
  FsmWorkState last_shooter_work_state_ = FsmWorkState::Dead;
  FsmCtrlMode shooter_ctrl_mode_ = FsmCtrlMode::Manual;
  FsmCtrlMode last_shooter_ctrl_mode_ = FsmCtrlMode::Manual;
  FsmManualCtrlSrc shooter_manual_ctrl_src_ = FsmManualCtrlSrc::Rc;
  FsmManualCtrlSrc last_shooter_manual_ctrl_src_ = FsmManualCtrlSrc::Rc;
  ShooterWorkingMode shooter_working_mode_ = ShooterWorkingMode::Normal;
  ShooterWorkingMode last_shooter_working_mode_ = ShooterWorkingMode::Normal;
  bool feed_stuck_flag_ = false, last_feed_stuck_flag_ = false;
  bool fric_stuck_flag_ = false, last_fric_stuck_flag_ = false;
  float heat_ = 0;
  float heat_limit_ = 100;
  float feed_ang_fdb_ = 0.0f, feed_ang_ref_ = 0.0f;
  float fric_spd_fdb_ = 0.0f, fric_spd_ref_ = 0.0f;


  // var for super capacitor
  float cap_pwr_percent_ = 0;

  // var for vision
  int16_t vis_tgt_x_ = 0;  ///< 视觉瞄准目标的 x 坐标，单位为像素，负数为无效值
  int16_t vis_tgt_y_ = 0;  ///< 视觉瞄准目标的 y 坐标，单位为像素，负数为无效值
  bool is_vision_valid_ = false;//视觉是否瞄到
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hero

#endif /* UI_DRAWER_HPP_ */
