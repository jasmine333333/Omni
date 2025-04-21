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
    kSuiPkgNum,
  };

  enum DynamicUiIdx {
    kDuiChassisContent,
    kDuiGimbalContent,
    kDuiPkgGroup1,  ///< 云台 pitch yaw 角度反馈，拨盘角度反馈，摩擦轮转速反馈，超电，底盘朝向(2)
    kDuiPkgGroup2,  ///< 云台 pitch yaw 角度期望，拨盘角度期望，摩擦轮转速期望，小云台预设值，小云台当前俯仰角度
    kDuiPkgGroup3,
    kDuiPkgGroup4,
    kDuiPkgGroup5,
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

  void setHeat(float heat) { heat_ = heat; }
  void setHeatLimit(float limit) { heat_limit_ = limit; }
  void setFeedStuckFlag(bool flag)
  {
    if (flag != feed_stuck_flag_) {
      last_feed_stuck_flag_ = feed_stuck_flag_;
      feed_stuck_flag_ = flag;
    }
  }
  void setFricStuckFlag(bool flag)
  {
    if (flag != fric_stuck_flag_) {
      last_fric_stuck_flag_ = fric_stuck_flag_;
      fric_stuck_flag_ = flag;
    }
  }

  void setBulletNum(uint16_t num) { bullet_num_ = num; }

  void setBaseAttack(bool flag) { is_base_attack_ = flag; }

  void setCapPwrPercent(float percent) { cap_pwr_percent_ = percent; }

  void setNavigateFlag(bool state) { is_navigating_ = state; }

  void setHurtModuleid(uint8_t id){hurt_module_id_ = id; }
  void setisArmorHit(bool is_armor_hit){is_armor_hit_ = is_armor_hit; }  
  
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

  bool encodeStaticPkgGroup2(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);

  bool encodeDynaUiPkgGroup1(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeDynaUiPkgGroup2(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeDynaUiPkgGroup3(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeDynaUiPkgGroup4(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);
  bool encodeDynaUiPkgGroup5(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt);

  void genChassisStatus(hello_world::referee::Arc& g_head, hello_world::referee::Arc& g_other);
  void genChassisPassLineLeft(hello_world::referee::StraightLine& g);
  void genChassisPassLineRight(hello_world::referee::StraightLine& g);
  void genArmorHit(hello_world::referee::Arc &g_hit);

  void genPassSafe(hello_world::referee::Circle& g, bool is_safe);

  void genShooterHeat(hello_world::referee::Arc& g);

  void genBulletNum(hello_world::referee::FloatingNumber& g);
  void genCapPwrPercent(hello_world::referee::Rectangle& g_rect, hello_world::referee::FloatingNumber& g_num);

  void genVisTgt(hello_world::referee::Circle& g);
  void genVisionbox(hello_world::referee::Rectangle& g_rect);

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
  bool feed_stuck_flag_ = false, last_feed_stuck_flag_ = false;
  bool fric_stuck_flag_ = false, last_fric_stuck_flag_ = false;
  float heat_ = 0;
  float heat_limit_ = 100;
  float bullet_num_ = 0;
  bool is_base_attack_ = false;  ///< 基地受攻击标志位
  bool last_base_attack_ = false;  ///< 上一次基地受攻击标志位
  //var for navigation
  bool is_navigating_ = false; //巡航模式
  // var for super capacitor
  float cap_pwr_percent_ = 0;
  //受打击装甲板
  uint8_t hurt_module_id_ = 0;  ///< 受打击的装甲板ID，0表示未受打击
  uint8_t last_hurt_module_id_ = 0;  ///< 上一次受打击的装甲板ID，用于判断是否变化
  float last_armor_angle_hit_ = 0.0f;
  bool is_armor_hit_ = 0;  ///< 是否有装甲板被打击

  // var for vision
  int16_t vis_tgt_x_ = 0;  ///< 视觉瞄准目标的 x 坐标，单位为像素，负数为无效值
  int16_t vis_tgt_y_ = 0;  ///< 视觉瞄准目标的 y 坐标，单位为像素，负数为无效值
  bool is_vision_valid_ = false;//视觉是否瞄到
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hero

#endif /* UI_DRAWER_HPP_ */
