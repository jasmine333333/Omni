/** 
 *******************************************************************************
 * @file      :ui_drawer.cpp
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
/* Includes ------------------------------------------------------------------*/
#include <string>

#include "rfr_pkg/rfr_pkg_0x0301_inter_graphics.hpp"
#include "ui_drawer.hpp"

// typedef hello_world::referee hello_world::referee ;
/* Private macro -------------------------------------------------------------*/

namespace robot
{
/* Private constants ---------------------------------------------------------*/

#pragma region names of graphics

const hello_world::referee::GraphicLayer kStaticUiLayer = hello_world::referee::GraphicLayer::k0;
const hello_world::referee::GraphicLayer kDynamicUiLayer = hello_world::referee::GraphicLayer::k1;

const uint16_t kUiModuleStateAreaX1 = 100;
const uint16_t kUiModuleStateAreaX2 = 160;
const uint16_t kUiModuleStateAreaX3 = 200;
const uint16_t kUiModuleStateAreaX4 = 1300;
const uint16_t kUiModuleStateAreaX5 = 1340;
const uint16_t kUiModuleStateAreaY1 = 860;
const uint16_t kUiModuleStateAreaY2 = 700;
const int16_t kUiModuleStateAreaXDelta = 60;
const int16_t kUiModuleStateAreaXDeltaFloating = 100;
const int16_t kUiModuleStateAreaYDelta = -35;
const hello_world::referee::String::Color kUiModuleStateColor = hello_world::referee::String::Color::kOrange;
const hello_world::referee::Pixel kUiModuleStateFontSize = 10;
const hello_world::referee::Pixel kUiModuleStateLineWidth = 3;

// 行车线 中下

// 各模块状态 左上角
// chassis
const uint8_t kUiNameChassisWorkStateTitle[3] = {0x00, 0x00, 0x01};    ///< 底盘工作状态标题
const uint8_t kUiNameChassisWorkStateContent[3] = {0x00, 0x00, 0x02};  ///< 底盘工作状态内容

const uint16_t kUiChassisDirCircleX = 960;//灯条中心位置
const uint16_t kUiChassisDirCircleY = 540;
const uint8_t kUiNameChassisDirHead[3] = {0x00, 0x00, 0x03};  ///< 底盘朝向示意（底盘头部）
const uint8_t kUiNameChassisDirTail[3] = {0x00, 0x00, 0x04};  ///< 底盘朝向示意（底盘尾部）

const uint16_t kPixelCenterXCapBox = 1920 / 2;  //超电位置
const uint16_t kPixelCenterYCapBox = 120;
const uint16_t kPixelCapBoxWidth = 400; //超电能量余量外框
const uint16_t kPixelCapBoxHeight = 15;
const uint8_t kUiNameChassisCapBox[3] = {0x00, 0x00, 0x05};         ///< 超级电容能量余量外框
const uint8_t kUiNameChassisCapPercent[3] = {0x00, 0x00, 0x06};     ///< 超级电容能量余量百分比示意
const uint8_t kUiNameChassisCapPercentNum[3] = {0x00, 0x00, 0x07};  ///< 超级电容能量余量百分比数字

// 行车线
const uint8_t kuiNameChassisPassLineLeft[3] = {0x00, 0x00, 0x08};   ///< 底盘通行线左侧
const uint8_t kuiNameChassisPassLineRight[3] = {0x00, 0x00, 0x09};  ///< 底盘通行线右侧
const uint8_t kUiNamePassSafe[3] = {0x00, 0x00, 0x0A}; ///< 底盘通行线中间
// gimbal
const uint16_t kPixelCenterXVisionBox = 1704.5 / 2;  //todo 云台视觉状态位置
const uint16_t kPixelCenterYVisionBox =1198.3/2;
const uint16_t kPixelVisionBoxWidth = 592.5; //状态外框 云台视觉
const uint16_t kPixelVisionBoxHeight = 315.3;

const uint8_t kUiNameGimbalWorkStateTitle[3] = {0x00, 0x00, 0x40};    ///< 云台工作状态标题
const uint8_t kUiNameGimbalWorkStateContent[3] = {0x00, 0x00, 0x41};  ///< 云台工作状态内容

const uint8_t kUiNameGimbalPitchTitle[3] = {0x00, 0x00, 0x42};  ///< 云台俯仰角度标题
const uint8_t kUiNameGimbalPitchFdb[3] = {0x00, 0x00, 0x43};    ///< 云台俯仰角度反馈
const uint8_t kUiNameGimbalPitchRef[3] = {0x00, 0x00, 0x44};    ///< 云台俯仰角度期望

const uint8_t kUiNameGimbalYawTitle[3] = {0x00, 0x00, 0x45};  ///< 云台偏航角度标题
const uint8_t kUiNameGimbalYawFdb[3] = {0x00, 0x00, 0x46};    ///< 云台偏航角度反馈
const uint8_t kUiNameGimbalYawRef[3] = {0x00, 0x00, 0x47};    ///< 云台偏航角度期望

// shooter
const uint8_t kUiNameShooterWorkStateTitle[3] = {0x00, 0x00, 0x80};    ///< 发射机构工作状态标题
const uint8_t kUiNameShooterWorkStateContent[3] = {0x00, 0x00, 0x81};  ///< 发射机构工作状态内容

const uint8_t kUiNameFeedAngTitle[3] = {0x00, 0x00, 0x82};  ///< 拨盘角度标题
const uint8_t kUiNameFeedAngFdb[3] = {0x00, 0x00, 0x83};    ///< 拨盘角度反馈
const uint8_t kUiNameFeedAngRef[3] = {0x00, 0x00, 0x84};    ///< 拨盘角度期望
const uint8_t kUiNameFeedStuck[3] = {0x00, 0x00, 0x85};     ///< 拨盘堵转提示

const uint8_t kUiNameFricSpdTitle[3] = {0x00, 0x00, 0x86};    ///< 摩擦轮转速标题
const uint8_t kUiNameFricSpdContent[3] = {0x00, 0x00, 0x87};  ///< 摩擦轮转速反馈
const uint8_t kUiNameFricSpdRef[3] = {0x00, 0x00, 0x88};      ///< 摩擦轮转速期望
const uint8_t kUiNameFricStuck[3] = {0x00, 0x00, 0x89};       ///< 摩擦轮堵转提示

const uint8_t kUiNameShooterHeat[3] = {0x00, 0x00, 0x8A};  ///< 发射机构热量

// 瞄准线 正中
// const uint16_t kUiAimLineX = 987;
// const uint16_t kUiAimLineH5mY = 700;
// const uint16_t kUiAimErrH5m = 20;
// const uint16_t kUiAimLineH8mY = 660;
// const uint16_t kUiAimErrH8m = 15;
// const uint16_t kUiAimLineH10mY = 620;
// const uint16_t kUiAimErrH10m = 10;
// const uint16_t kUiAimLineH15mY = 580;
// const uint16_t kUiAimErrH15m = 5;

// const uint8_t kUiNameAimLineV[3] = {0x00, 0x00, 0x8B};     ///< 瞄准线垂直
// const uint8_t kUiNameAimLineH5m[3] = {0x00, 0x00, 0x8C};   ///< 瞄准线水平 5m
// const uint8_t kUiNameAimLineH8m[3] = {0x00, 0x00, 0x8D};   ///< 瞄准线水平 8m
// const uint8_t kuiNameAimLineH10m[3] = {0x00, 0x00, 0x8E};  ///< 瞄准线水平 10m
// const uint8_t kuiNameAimLineH15m[3] = {0x00, 0x00, 0x8F};  ///< 瞄准线水平 15m
// mini gimbal
const uint8_t kUiNameScopeWorkStateTitle[3] = {0x00, 0x00, 0xC0};    ///< 小云台工作状态标题
const uint8_t kUiNameScopeWorkStateContent[3] = {0x00, 0x00, 0xC1};  ///< 小云台工作状态内容

const uint8_t kUiNameScopeAngTitle[3] = {0x00, 0x00, 0xC2};   ///< 小云台俯仰角度标题
const uint8_t kUiNameScopeAngPreSet[3] = {0x00, 0x00, 0xC3};  ///< 当前小云台俯仰角度预设值
const uint8_t kUiNameScopeAngNow[3] = {0x00, 0x00, 0xC4};     ///< 当前小云台俯仰额外俯仰角

// vision
const uint8_t kUiNameVisionBox[3] = {0x00, 0x00, 0xE0};  ///< 视觉相机视场框
const uint8_t kUiNameVisionTgt[3] = {0x00, 0x00, 0xE1};  ///< 视觉相机目标框

// 安全过洞参数
const float ksafepitchmin = 0.0;//todo
const float ksafepitchmax = 0.1;//todo
#pragma endregion names of graphics

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

bool UiDrawer::encode(uint8_t* data_ptr, size_t& data_len)
{
  bool res = false;
  bool is_all_added = (n_added_ == kNumAllPkgs);
  hello_world::referee::GraphicOperation opt = hello_world::referee::GraphicOperation::kAdd;
  if (is_all_added) {
    opt = hello_world::referee::GraphicOperation::kModify;
  }
  if (ui_idx_ >= kNumAllPkgs) {
    if (is_all_added) {
      ui_idx_ = kSuiPkgNum;
    } else {
      ui_idx_ = 0;
    }
  }

  if (ui_idx_ < kSuiPkgNum) {
    res = encodeStaticUi(data_ptr, data_len, opt, StaticUiIdx(ui_idx_));
  } else if (ui_idx_ < kNumAllPkgs) {
    res = encodeDynamicUi(data_ptr, data_len, opt, DynamicUiIdx(ui_idx_ - kSuiPkgNum));
  };

  if (res) {
    ui_idx_++;
  }

  if (res && (is_all_added == false)) {
    n_added_++;
  }
  return res;
};

bool UiDrawer::encodeStaticUi(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt, StaticUiIdx idx)
{
  switch (idx) {
    case kSuiDelAll:
      return encodeDelAll(data_ptr, data_len);
      break;
    // case kSuiPassLinePkgGroup1:
    //   return encodeStaticPkgGroup1(data_ptr, data_len, opt);
    //   break;
    case kSuiPassLinePkgGroup2:
      return encodeStaticPkgGroup2(data_ptr, data_len, opt);
      break;
    case kSuiChassisTitle:
      return encodeChassisWorkStateTitle(data_ptr, data_len, opt);
      break;
    case kSuiGimbalTitle:
      return encodeGimbalWorkStateTitle(data_ptr, data_len, opt);
      break;
    // case kSuiShooterTitle:
    //   return encodeShooterWorkStateTitle(data_ptr, data_len, opt);
    //   break;
    // case kSuiFeedAngTitle:
    //   return encodeFeedTitle(data_ptr, data_len, opt);
    //   break;
    // case kSuiFricSpdTitle:
    //   return encodeFricTitle(data_ptr, data_len, opt);
    //   break;
    // case kSuiPitchAngTitle:
    //   return encodeGimbalPitchTitle(data_ptr, data_len, opt);
    //   break;
    // case kSuiYawAngTitle:
    //   return encodeGimbalYawTitle(data_ptr, data_len, opt);
    //   break;
    default:
      break;
  }
  return true;
};
bool UiDrawer::encodeDynamicUi(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt, DynamicUiIdx idx)
{
  bool res = true;
  switch (idx) {
    case kDuiChassisContent:
      if (opt == hello_world::referee::GraphicOperation::kModify) {
        if (last_chassis_work_state_ == chassis_work_state_ && last_chassis_working_mode_ == chassis_working_mode_ &&
            last_chassis_ctrl_mode_ == chassis_ctrl_mode_ && last_chassis_manual_ctrl_src_ == chassis_manual_ctrl_src_) {
          return true;
        }
      } else if (opt == hello_world::referee::GraphicOperation::kAdd) {
      } else {
        return true;
      }

      res = encodeChassisWorkStateContent(data_ptr, data_len, opt);
      if (res == true) {
        last_chassis_work_state_ = chassis_work_state_;
        last_chassis_working_mode_ = chassis_working_mode_;
        last_chassis_ctrl_mode_ = chassis_ctrl_mode_;
        last_chassis_manual_ctrl_src_ = chassis_manual_ctrl_src_;
      }

      break;
    case kDuiGimbalContent:
      if (opt == hello_world::referee::GraphicOperation::kModify) {
        if (last_gimbal_work_state_ == gimbal_work_state_ && last_gimbal_working_mode_ == gimbal_working_mode_ && last_gimbal_ctrl_mode_ == gimbal_ctrl_mode_ &&
            last_gimbal_manual_ctrl_src_ == gimbal_manual_ctrl_src_) {
          return true;
        }
      } else if (opt == hello_world::referee::GraphicOperation::kAdd) {
      } else {
        return true;
      }
      // res = encodeGimbalWorkStateContent(data_ptr, data_len, opt);
      if (res == true) {
        last_gimbal_work_state_ = gimbal_work_state_;
        last_gimbal_working_mode_ = gimbal_working_mode_;
        last_gimbal_ctrl_mode_ = gimbal_ctrl_mode_;
        last_gimbal_manual_ctrl_src_ = gimbal_manual_ctrl_src_;
      }

      break;
    case kDuiShooterContent:
      if (opt == hello_world::referee::GraphicOperation::kModify) {
        if (last_shooter_work_state_ == shooter_work_state_ && last_shooter_working_mode_ == shooter_working_mode_ &&
            last_shooter_ctrl_mode_ == shooter_ctrl_mode_ && last_shooter_manual_ctrl_src_ == shooter_manual_ctrl_src_) {
          return true;
        }
      } else if (opt == hello_world::referee::GraphicOperation::kAdd) {
      } else {
        return true;
      }
      // res = encodeShooterWorkStateContent(data_ptr, data_len, opt);
      if (res == true) {
        last_shooter_work_state_ = shooter_work_state_;
        last_shooter_working_mode_ = shooter_working_mode_;
        last_shooter_ctrl_mode_ = shooter_ctrl_mode_;
        last_shooter_manual_ctrl_src_ = shooter_manual_ctrl_src_;
      }

      break;
    case kDuiPkgGroup1:
      res = encodeDynaUiPkgGroup1(data_ptr, data_len, opt);
      break;
    case kDuiPkgGroup2:
      res = encodeDynaUiPkgGroup2(data_ptr, data_len, opt);
      break;
    case kDuiPkgGroup3:
      res = encodeDynaUiPkgGroup3(data_ptr, data_len, opt);
      break;
    default:
      break;
  }

  return res;
};

bool UiDrawer::encodeDelAll(uint8_t* data_ptr, size_t& data_len)
{
  hello_world::referee::InterGraphicDeletePackage pkg;
  pkg.setSenderId(static_cast<uint16_t>(sender_id_));
  pkg.setDeleteOperation(hello_world::referee::DeleteOperation::kAll);
  return encoder_.encodeFrame(&pkg, data_ptr, &data_len);
};

#pragma region
#pragma region UI 组
// bool UiDrawer::encodeStaticPkgGroup1(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt)
// {
//   hello_world::referee::StraightLine g_aim_line_v, g_aim_line_h5m, g_aim_line_h8m, g_aim_line_h10m, g_aim_line_h15m;
//   genAimLineV(g_aim_line_v);
//   g_aim_line_v.setOperation(opt);
//   genAimLineH5m(g_aim_line_h5m);
//   g_aim_line_h5m.setOperation(opt);
//   genAimLineH8m(g_aim_line_h8m);
//   g_aim_line_h8m.setOperation(opt);
//   genAimLineH10m(g_aim_line_h10m);
//   g_aim_line_h10m.setOperation(opt);
//   genAimLineH15m(g_aim_line_h15m);
//   g_aim_line_h15m.setOperation(opt);

//   hello_world::referee::InterGraphic5Package pkg;
//   pkg.setSenderId(static_cast<uint16_t>(sender_id_));
//   pkg.setStraightLineAt(g_aim_line_v, 0);
//   pkg.setStraightLineAt(g_aim_line_h5m, 1);
//   pkg.setStraightLineAt(g_aim_line_h8m, 2);
//   pkg.setStraightLineAt(g_aim_line_h10m, 3);
//   pkg.setStraightLineAt(g_aim_line_h15m, 4);
//   return encodePkg(data_ptr, data_len, opt, pkg);
// };
bool UiDrawer::encodeStaticPkgGroup2(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt)
{
  //为满足过洞需求，将行车线修改为动态
  // hello_world::referee::StraightLine g_pass_line_left, g_pass_line_right;
  // genChassisPassLineLeft(g_pass_line_left);
  // g_pass_line_left.setOperation(opt);
  // genChassisPassLineRight(g_pass_line_right);
  // g_pass_line_right.setOperation(opt);

  //新增视觉框
  hello_world::referee::Rectangle g_vision_box;
  genVisionbox(g_vision_box);
  g_vision_box.setOperation(opt);

  hello_world::referee::InterGraphic1Package pkg;
  pkg.setSenderId(static_cast<uint16_t>(sender_id_));
  // pkg.setStraightLineAt(g_pass_line_left, 0);
  // pkg.setStraightLineAt(g_pass_line_right, 1);
  pkg.setRectangle(g_vision_box);
  return encodePkg(data_ptr, data_len, opt, pkg);
};
bool UiDrawer::encodeDynaUiPkgGroup1(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt)
{
  // hello_world::referee::FloatingNumber g_pitch_fdb, g_yaw_fdb;
  // hello_world::referee::Integer g_feed_fdb, g_fric_fdb;
  // hello_world::referee::FloatingNumber g_mini_pitch_preset;
  hello_world::referee::Arc g_chassis_status_head, g_chassis_status_other;

  // genGimbalJointAngPitchFdb(g_pitch_fdb);
  // g_pitch_fdb.setOperation(opt);

  // genGimbalJointAngYawFdb(g_yaw_fdb);
  // g_yaw_fdb.setOperation(opt);

  // genShooterFeedAngFdb(g_feed_fdb);
  // g_feed_fdb.setOperation(opt);

  // genShooterFricSpdFdb(g_fric_fdb);
  // g_fric_fdb.setOperation(opt);


  genChassisStatus(g_chassis_status_head, g_chassis_status_other);
  g_chassis_status_head.setOperation(opt);
  g_chassis_status_other.setOperation(opt);

  hello_world::referee::InterGraphic2Package pkg;
  pkg.setSenderId(static_cast<uint16_t>(sender_id_));
  // pkg.setFloatingNumberAt(g_pitch_fdb, 0);
  // pkg.setFloatingNumberAt(g_yaw_fdb, 1);
  // pkg.setIntegerAt(g_feed_fdb, 2);
  // pkg.setIntegerAt(g_fric_fdb, 3);
  // pkg.setFloatingNumberAt(g_mini_pitch_preset, 4);
  pkg.setArcAt(g_chassis_status_head, 0);
  pkg.setArcAt(g_chassis_status_other, 1);
  return encodePkg(data_ptr, data_len, opt, pkg);
};
bool UiDrawer::encodeDynaUiPkgGroup2(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt)
{
  hello_world::referee::FloatingNumber g_pitch_ref, g_yaw_ref;
  hello_world::referee::Integer g_feed_ref, g_fric_ref;
  hello_world::referee::FloatingNumber g_mini_Pitch_now;
  hello_world::referee::Rectangle g_cap_pwr_percent_rect;
  hello_world::referee::FloatingNumber g_cap_pwr_percent_num;
  // genGimbalJointAngPitchRef(g_pitch_ref);
  // g_pitch_ref.setOperation(opt);

  // genGimbalJointAngYawRef(g_yaw_ref);
  // g_yaw_ref.setOperation(opt);

  // genShooterFeedAngRef(g_feed_ref);
  // g_feed_ref.setOperation(opt);

  // genShooterFricSpdRef(g_fric_ref);
  // g_fric_ref.setOperation(opt);


  genCapPwrPercent(g_cap_pwr_percent_rect, g_cap_pwr_percent_num);
  g_cap_pwr_percent_rect.setOperation(opt);
  g_cap_pwr_percent_num.setOperation(opt);

  hello_world::referee::StraightLine g_pass_line_left, g_pass_line_right;
  genChassisPassLineLeft(g_pass_line_left);
  g_pass_line_left.setOperation(opt);
  genChassisPassLineRight(g_pass_line_right);
  g_pass_line_right.setOperation(opt);

  //显示过洞角度是否安全
  hello_world::referee::Circle g_pass_hole;
  bool is_safe = false;
  if (gimbal_joint_ang_pitch_fdb_ > ksafepitchmin && gimbal_joint_ang_pitch_fdb_ < ksafepitchmax)
  {
    is_safe = true;
  }
  else
  {
    is_safe = false;
  }
  genPassSafe(g_pass_hole, is_safe);
  g_pass_hole.setOperation(opt);

  hello_world::referee::InterGraphic5Package pkg;
  pkg.setSenderId(static_cast<uint16_t>(sender_id_));
  // pkg.setFloatingNumberAt(g_pitch_ref, 0);
  // pkg.setFloatingNumberAt(g_yaw_ref, 1);
  // pkg.setIntegerAt(g_feed_ref, 2);
  // pkg.setIntegerAt(g_fric_ref, 3);
  // pkg.setFloatingNumberAt(g_mini_Pitch_now, 4);
  pkg.setRectangleAt(g_cap_pwr_percent_rect, 0);
  pkg.setFloatingNumberAt(g_cap_pwr_percent_num, 1);
  pkg.setStraightLineAt(g_pass_line_left, 2);
  pkg.setStraightLineAt(g_pass_line_right, 3);
  pkg.setCircleAt(g_pass_hole, 4);
  return encodePkg(data_ptr, data_len, opt, pkg);
};
bool UiDrawer::encodeDynaUiPkgGroup3(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt)
{
  hello_world::referee::Arc g_heat;
  hello_world::referee::Circle g_vision;
  hello_world::referee::Integer g_balence_number;

  genShooterHeat(g_heat);
  g_heat.setOperation(opt);

  genVisTgt(g_vision);//自瞄瞄上了画圆？
  g_vision.setOperation(opt);


  hello_world::referee::InterGraphic5Package pkg;
  pkg.setSenderId(static_cast<uint16_t>(sender_id_));
  pkg.setArcAt(g_heat, 0);
  pkg.setCircleAt(g_vision, 1);
  pkg.setArcAt(g_heat, 2);
  pkg.setCircleAt(g_vision, 3);
  return encodePkg(data_ptr, data_len, opt, pkg);
};
  #pragma endregion

#pragma region 底盘相关 UI

/** 
 * @brief 编码左上方 UI 字符串 `Chassis:`
 */
bool UiDrawer::encodeChassisWorkStateTitle(uint8_t* data_ptr, size_t& data_len, GraphicOperation opt)
{
  std::string str = "Chassis:";
  hello_world::referee::String g = hello_world::referee::String(kUiNameChassisWorkStateTitle, opt, kStaticUiLayer, kUiModuleStateColor, kUiModuleStateAreaX1, kUiModuleStateAreaY1,
                                    kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);

  return encodeString(data_ptr, data_len, opt, g, str);
};

/** 
 * @brief 根据底盘工作状态编码左上方 UI 字符串(`Chassis:` 之后的内容)
 */
bool UiDrawer::encodeChassisWorkStateContent(uint8_t* data_ptr, size_t& data_len, UiDrawer::GraphicOperation opt)
{
  std::string str = "Unkown";
  // if ()
  // {
  //   /* code */
  // }
  
  if (chassis_work_state_ != robot::PwrState::Working) {
    str = Chassis::WorkStateToStr(chassis_work_state_);

  } else {
    str = Chassis::WorkingModeToStr(chassis_working_mode_) + "-" + Chassis::CtrlModeSrcToStr(chassis_ctrl_mode_, chassis_manual_ctrl_src_);
  }

  hello_world::referee::String g = hello_world::referee::String(kUiNameChassisWorkStateContent, opt, kDynamicUiLayer, kUiModuleStateColor, kUiModuleStateAreaX3, kUiModuleStateAreaY1,
                                    kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);

  return encodeString(data_ptr, data_len, opt, g, str);
};

void UiDrawer::genChassisStatus(hello_world::referee::Arc& g_head, hello_world::referee::Arc& g_other)
{
  float now_head_ang = -theta_i2r_ * 180 / M_PI;
  float start_ang_head = now_head_ang - 40, end_ang_head = now_head_ang + 40;
  start_ang_head = hello_world::NormPeriodData(0, 360, start_ang_head);
  end_ang_head = hello_world::NormPeriodData(0, 360, end_ang_head);

  float start_ang_other = start_ang_head + 180, end_ang_other = end_ang_head + 180;
  start_ang_other = hello_world::NormPeriodData(0, 360, start_ang_other);
  end_ang_other = hello_world::NormPeriodData(0, 360, end_ang_other);

  float radius = 50;//灯条所处圆的半径
  g_head.setColor(hello_world::referee::Arc::Color::kYellow);
  g_head.setAng(start_ang_head, end_ang_head);
  g_head.setCenterPos(kUiChassisDirCircleX, kUiChassisDirCircleY);//灯条中心位置
  g_head.setRadius(radius, radius);
  g_head.setName(kUiNameChassisDirHead);
  g_head.setLineWidth(3);
  g_head.setLayer(kDynamicUiLayer);

  g_other.setColor(hello_world::referee::Arc::Color::kCyan);
  g_other.setAng(start_ang_other, end_ang_other);
  g_other.setCenterPos(kUiChassisDirCircleX, kUiChassisDirCircleY);
  g_other.setRadius(radius, radius);
  g_other.setName(kUiNameChassisDirTail);
  g_other.setLineWidth(3);
  g_head.setLayer(kDynamicUiLayer);
};

void UiDrawer::genCapPwrPercent(hello_world::referee::Rectangle& g_rect, hello_world::referee::FloatingNumber& g_num)
{
  uint16_t start_x = kPixelCenterXCapBox - kPixelCapBoxWidth / 2;
  float percent = hello_world::Bound(cap_pwr_percent_, 0, 1);
  uint16_t end_x = start_x + kPixelCapBoxWidth * percent;

  hello_world::referee::GraphicColor color = hello_world::referee::Rectangle::Color::kGreen;

  if (percent > 0.8) {
  } else if (percent > 0.6) {
    color = hello_world::referee::Rectangle::Color::kYellow;
  } else if (percent > 0.4) {
    color = hello_world::referee::Rectangle::Color::kOrange;
  } else {
    color = hello_world::referee::Rectangle::Color::kPurple;
  }

  g_rect.setName(kUiNameChassisCapPercent);
  g_rect.setStartPos(start_x, kPixelCenterYCapBox - kPixelCapBoxHeight / 2);
  g_rect.setEndPos(end_x, kPixelCenterYCapBox + kPixelCapBoxHeight / 2);
  g_rect.setColor(color);
  g_rect.setLayer(kDynamicUiLayer);
  g_rect.setLineWidth(kPixelCapBoxHeight * 2);

  g_num.setName(kUiNameChassisCapPercentNum);
  g_num.setDisplayValue(percent * 100);
  g_num.setStartPos(start_x - 100, kPixelCenterYCapBox);
  g_num.setColor(color);
  g_num.setLayer(kDynamicUiLayer);
  g_num.setFontSize(20);//调整超电剩余电量的数字大小
  g_num.setLineWidth(kUiModuleStateLineWidth);
};

void UiDrawer::genChassisPassLineLeft(hello_world::referee::StraightLine& g)
{
  uint16_t end_posX = 0;
  uint16_t start_posX = 0;
  uint16_t end_posY = 0;
  g.setName(kuiNameChassisPassLineLeft);
  end_posX = gimbal_joint_ang_pitch_fdb_ *-81.75 + 814.7;
  start_posX = gimbal_joint_ang_pitch_fdb_ * 660.4 +577.15;
  end_posY = gimbal_joint_ang_pitch_fdb_ * -927.28 + 334.39;
  g.setLayer(kDynamicUiLayer);
  g.setStartPos(start_posX, 0);
  g.setEndPos(end_posX, end_posY);
  g.setColor(kUiModuleStateColor);
  g.setLineWidth(3);
};
void UiDrawer::genChassisPassLineRight(hello_world::referee::StraightLine& g)
{
  uint16_t end_posX = 0;
  uint16_t start_posX = 0;
  uint16_t end_posY = 0;
  start_posX = gimbal_joint_ang_pitch_fdb_ * (-801.84) + 1277.1;
  end_posX = gimbal_joint_ang_pitch_fdb_*96.48 + 1077;
  end_posY = gimbal_joint_ang_pitch_fdb_ * (-927.28) + 334.39;
  g.setName(kuiNameChassisPassLineRight);
  g.setLayer(kDynamicUiLayer);
  g.setStartPos(start_posX, 0);
  g.setEndPos(end_posX, end_posY);
  g.setColor(kUiModuleStateColor);
  g.setLineWidth(3);
};
#pragma endregion
#pragma region 云台相关 UI
/** 
 * @brief 编码左上方 UI 字符串 `Gimbal:`  
 */
bool UiDrawer::encodeGimbalWorkStateTitle(uint8_t* data_ptr, size_t& data_len, UiDrawer::GraphicOperation opt)
{
  std::string str = "Gimbal:";

  hello_world::referee::String g = hello_world::referee::String(kUiNameGimbalWorkStateTitle, opt, kStaticUiLayer, kUiModuleStateColor, kUiModuleStateAreaX1,
                                    kUiModuleStateAreaY1 + kUiModuleStateAreaYDelta, kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);
  return encodeString(data_ptr, data_len, opt, g, str);
};
/** 
 * @brief 根据云台工作状态编码左上方 UI 字符串(`Gimbal:` 之后的内容)
 */
bool UiDrawer::encodeGimbalWorkStateContent(uint8_t* data_ptr, size_t& data_len, UiDrawer::GraphicOperation opt)
{
  std::string str = "Unkown";
  if (gimbal_work_state_ != robot::PwrState::Working) {
    str = Gimbal::WorkStateToStr(gimbal_work_state_);
  } else {
    str = Gimbal::WorkingModeToStr(gimbal_working_mode_) + "-" + Gimbal::CtrlModeSrcToStr(gimbal_ctrl_mode_, gimbal_manual_ctrl_src_);
  }
  hello_world::referee::String g = hello_world::referee::String(kUiNameGimbalWorkStateContent, opt, kDynamicUiLayer, kUiModuleStateColor, kUiModuleStateAreaX3,
                                    kUiModuleStateAreaY1 + kUiModuleStateAreaYDelta, kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);

  return encodeString(data_ptr, data_len, opt, g, str);
};

bool UiDrawer::encodeGimbalPitchTitle(uint8_t* data_ptr, size_t& data_len, UiDrawer::GraphicOperation opt)
{
  std::string str = "P:";
  hello_world::referee::String g = hello_world::referee::String(kUiNameGimbalPitchTitle, opt, kStaticUiLayer, kUiModuleStateColor, kUiModuleStateAreaX4, kUiModuleStateAreaY2,
                                    kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);
  return encodeString(data_ptr, data_len, opt, g, str);
};

bool UiDrawer::encodeGimbalYawTitle(uint8_t* data_ptr, size_t& data_len, UiDrawer::GraphicOperation opt)
{
  std::string str = "Y:";
  hello_world::referee::String g = hello_world::referee::String(kUiNameGimbalYawTitle, opt, kStaticUiLayer, kUiModuleStateColor, kUiModuleStateAreaX4,
                                    kUiModuleStateAreaY2 + kUiModuleStateAreaYDelta, kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);
  return encodeString(data_ptr, data_len, opt, g, str);
};

void UiDrawer::genGimbalJointAngPitchFdb(hello_world::referee::FloatingNumber& g)
{
  g.setName(kUiNameGimbalPitchFdb);
  g.setDisplayValue(gimbal_joint_ang_pitch_fdb_ * 180 / PI);
  g.setStartPos(kUiModuleStateAreaX5, kUiModuleStateAreaY2);
  g.setColor(kUiModuleStateColor);
  g.setLayer(kDynamicUiLayer);
  g.setFontSize(kUiModuleStateFontSize);
  g.setLineWidth(kUiModuleStateLineWidth);
};
void UiDrawer::genGimbalJointAngPitchRef(hello_world::referee::FloatingNumber& g)
{
  g.setName(kUiNameGimbalPitchRef);
  g.setDisplayValue(gimbal_joint_ang_pitch_ref_ * 180 / PI);
  g.setStartPos(kUiModuleStateAreaX5 + kUiModuleStateAreaXDeltaFloating, kUiModuleStateAreaY2);
  g.setColor(kUiModuleStateColor);
  g.setLayer(kDynamicUiLayer);
  g.setFontSize(kUiModuleStateFontSize);
  g.setLineWidth(kUiModuleStateLineWidth);
};
void UiDrawer::genGimbalJointAngYawFdb(hello_world::referee::FloatingNumber& g)
{
  g.setName(kUiNameGimbalYawFdb);
  g.setDisplayValue(gimbal_joint_ang_yaw_fdb_ * 180 / PI);
  g.setStartPos(kUiModuleStateAreaX5, kUiModuleStateAreaY2 + kUiModuleStateAreaYDelta);
  g.setColor(kUiModuleStateColor);
  g.setLayer(kDynamicUiLayer);
  g.setFontSize(kUiModuleStateFontSize);
  g.setLineWidth(kUiModuleStateLineWidth);
};
void UiDrawer::genGimbalJointAngYawRef(hello_world::referee::FloatingNumber& g)
{
  g.setName(kUiNameGimbalYawRef);
  g.setDisplayValue(gimbal_joint_ang_yaw_ref_ * 180 / PI);
  g.setStartPos(kUiModuleStateAreaX5 + kUiModuleStateAreaXDeltaFloating, kUiModuleStateAreaY2 + kUiModuleStateAreaYDelta);
  g.setColor(kUiModuleStateColor);
  g.setLayer(kDynamicUiLayer);
  g.setFontSize(kUiModuleStateFontSize);
  g.setLineWidth(kUiModuleStateLineWidth);
};
void UiDrawer::genPassSafe(hello_world::referee::Circle& g, bool is_safe)
{
  g.setName(kUiNamePassSafe);
  g.setCenterPos(kUiModuleStateAreaX3, kUiModuleStateAreaY1 + kUiModuleStateAreaYDelta);//todo确认该指示显示位置
  g.setRadius(25);
  g.setColor(is_safe ? hello_world::referee::Circle::Color::kGreen : hello_world::referee::Circle::Color::kPurple);
  g.setLayer(kDynamicUiLayer);
  g.setLineWidth(3);
};
#pragma endregion

#pragma region 发射机构相关 UI
/** 
 * @brief 编码左上方 UI 字符串 `Scope:`
 */

bool UiDrawer::encodeFeedTitle(uint8_t* data_ptr, size_t& data_len, UiDrawer::GraphicOperation opt)
{
  std::string str = "Feed:";
  hello_world::referee::String g = hello_world::referee::String(kUiNameFeedAngTitle, opt, kStaticUiLayer, kUiModuleStateColor, kUiModuleStateAreaX1,
                                    kUiModuleStateAreaY1 + 4 * kUiModuleStateAreaYDelta, kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);
  return encodeString(data_ptr, data_len, opt, g, str);
};
bool UiDrawer::encodeFricTitle(uint8_t* data_ptr, size_t& data_len, UiDrawer::GraphicOperation opt)
{
  std::string str = "Fric:";
  hello_world::referee::String g = hello_world::referee::String(kUiNameFricSpdTitle, opt, kStaticUiLayer, kUiModuleStateColor, kUiModuleStateAreaX1,
                                    kUiModuleStateAreaY1 + 5 * kUiModuleStateAreaYDelta, kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);
  return encodeString(data_ptr, data_len, opt, g, str);
};
void UiDrawer::genShooterHeat(hello_world::referee::Arc& g)
{
  float percent = heat_limit_ > 0 ? heat_ / heat_limit_ : 0.0f;
  percent = hello_world::Bound(percent, 0.0f, 1.0f);
  g.setName(kUiNameShooterHeat);
  g.setCenterPos(1920 / 2, 1080 / 2);
  hello_world::referee::GraphicColor color = hello_world::referee::Arc::Color::kGreen;
  if (percent > 0.8) {
    color = hello_world::referee::Arc::Color::kPurple;
  } else if (percent > 0.6) {
    color = hello_world::referee::Arc::Color::kOrange;
  } else if (percent > 0.3) {
    color = hello_world::referee::Arc::Color::kYellow;
  }
  g.setRadius(100, 100);
  g.setAng(360 - 360 * percent, 0);
  g.setColor(color);
  g.setLayer(kDynamicUiLayer);
  g.setLineWidth(percent == 0 ? 0 : 2);
};
void UiDrawer::genShooterFeedAngFdb(hello_world::referee::Integer& g)
{
  g.setName(kUiNameFeedAngFdb);
  g.setDisplayValue(feed_ang_fdb_ * 180.0f / PI);
  g.setStartPos(kUiModuleStateAreaX2, kUiModuleStateAreaY1 + 4 * kUiModuleStateAreaYDelta);
  g.setColor(kUiModuleStateColor);
  g.setLayer(kDynamicUiLayer);
  g.setFontSize(kUiModuleStateFontSize);
  g.setLineWidth(kUiModuleStateLineWidth);
};
void UiDrawer::genShooterFeedAngRef(hello_world::referee::Integer& g)
{
  g.setName(kUiNameFeedAngRef);
  g.setDisplayValue(feed_ang_ref_ * 180.0f / PI);
  g.setStartPos(kUiModuleStateAreaX2 + kUiModuleStateAreaXDelta, kUiModuleStateAreaY1 + 4 * kUiModuleStateAreaYDelta);
  g.setColor(kUiModuleStateColor);
  g.setLayer(kDynamicUiLayer);
  g.setFontSize(kUiModuleStateFontSize);
  g.setLineWidth(kUiModuleStateLineWidth);
};
void UiDrawer::genShooterFricSpdFdb(hello_world::referee::Integer& g)
{
  g.setName(kUiNameFricSpdContent);
  g.setDisplayValue(fric_spd_fdb_);
  g.setStartPos(kUiModuleStateAreaX2, kUiModuleStateAreaY1 + 5 * kUiModuleStateAreaYDelta);
  g.setColor(kUiModuleStateColor);
  g.setLayer(kDynamicUiLayer);
  g.setFontSize(kUiModuleStateFontSize);
  g.setLineWidth(kUiModuleStateLineWidth);
};
void UiDrawer::genShooterFricSpdRef(hello_world::referee::Integer& g)
{
  g.setName(kUiNameFricSpdRef);
  g.setDisplayValue(fric_spd_ref_);
  g.setStartPos(kUiModuleStateAreaX2 + kUiModuleStateAreaXDelta, kUiModuleStateAreaY1 + 5 * kUiModuleStateAreaYDelta);
  g.setColor(kUiModuleStateColor);
  g.setLayer(kDynamicUiLayer);
  g.setFontSize(kUiModuleStateFontSize);
  g.setLineWidth(kUiModuleStateLineWidth);
};

#pragma endregion

#pragma region 倍镜云台
/** 
 * @brief 编码左上方 UI 字符串 `Shooter:`
 */
bool UiDrawer::encodeShooterWorkStateTitle(uint8_t* data_ptr, size_t& data_len, UiDrawer::GraphicOperation opt)
{
  std::string str = "Shooter:";

  hello_world::referee::String g = hello_world::referee::String(kUiNameShooterWorkStateTitle, opt, kStaticUiLayer, kUiModuleStateColor, kUiModuleStateAreaX1,
                                    kUiModuleStateAreaY1 + 2 * kUiModuleStateAreaYDelta, kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);
  return encodeString(data_ptr, data_len, opt, g, str);
};
/** 
 * @brief 根据射门工作状态编码左上方 UI 字符串(`Shooter:` 之后的内容)
 */
bool UiDrawer::encodeShooterWorkStateContent(uint8_t* data_ptr, size_t& data_len, UiDrawer::GraphicOperation opt)
{
  std::string str = "Unkown";
  if (shooter_work_state_ != robot::PwrState::Working) {
    str = Shooter::WorkStateToStr(shooter_work_state_);
  } else {
    str = Shooter::WorkingModeToStr(shooter_working_mode_) + "-" + Shooter::CtrlModeSrcToStr(shooter_ctrl_mode_, shooter_manual_ctrl_src_);
  }

  hello_world::referee::String g = hello_world::referee::String(kUiNameShooterWorkStateContent, opt, kDynamicUiLayer, kUiModuleStateColor, kUiModuleStateAreaX3,
                                    kUiModuleStateAreaY1 + 2 * kUiModuleStateAreaYDelta, kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);

  return encodeString(data_ptr, data_len, opt, g, str);
};

#pragma endregion
#pragma region 视觉 UI

void UiDrawer::genVisTgt(hello_world::referee::Circle& g)
{
  g.setName(kUiNameVisionTgt);
  g.setCenterPos(vis_tgt_x_ , 1080 - vis_tgt_y_ );
  g.setRadius(35);
  g.setColor(hello_world::referee::String::Color::kGreen);
  g.setLayer(hello_world::referee::GraphicLayer::k1);
  g.setLineWidth(2);
};
void UiDrawer::genVisionbox(hello_world::referee::Rectangle& g_rect)
{
  uint16_t start_x = kPixelCenterXVisionBox - kPixelVisionBoxWidth / 2;
  uint16_t end_x = start_x + kPixelVisionBoxWidth ;

  if (is_vision_valid_) {
    g_rect.setColor(hello_world::referee::String::Color::kPurple);
  }
  else {
    g_rect.setColor(hello_world::referee::String::Color::kWhite);
  }

  g_rect.setName(kUiNameVisionBox);
  g_rect.setStartPos(start_x, kPixelCenterYVisionBox - kPixelVisionBoxHeight / 2);
  g_rect.setEndPos(end_x, kPixelCenterYVisionBox + kPixelVisionBoxHeight / 2);
  g_rect.setLayer(kStaticUiLayer);
  g_rect.setLineWidth(1.5);
};

#pragma endregion
/* Private function definitions ----------------------------------------------*/

}  // namespace hero