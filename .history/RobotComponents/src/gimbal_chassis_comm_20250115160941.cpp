/** 
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
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
#include "gimbal_chassis_comm.hpp"

#include <cstring>
/* Private macro -------------------------------------------------------------*/
namespace robot
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/

struct __attribute__((packed)) C2GPkg1 {
  uint8_t pkg_type;
  // gimbal
  int8_t gimbal_yaw_delta;
  int8_t gimbal_pitch_delta;
  uint8_t gimbal_turn_back_flag : 1;
  uint32_t gimbal_ctrl_mode : 1;
  uint32_t gimbal_working_mode : 2;
  // shooter
  uint32_t shoot_flag : 1;
  uint32_t shooter_ctrl_mode : 1;
  uint32_t shooter_working_mode : 2;
  // scope
  uint32_t scope_switch_flag : 1;
  uint32_t scope_ctrl_ang_flag : 1;
  uint32_t scope_working_mode : 2;

  static void encode(GimbalChassisComm &gc_comm, uint8_t *tx_data);

  static void decode(GimbalChassisComm &gc_comm, const uint8_t *rx_data);
};
static_assert(sizeof(C2GPkg1) <= 8, "C2GPkg1 size error");

struct __attribute__((packed)) C2GPkg2 {
  uint8_t pkg_type;
  // rfr
  uint8_t rfr_robot_id;
  uint8_t rfr_heat_limit;     ///< 热量限制 [0, 650] 都可以被 10 整除
  uint8_t rfr_st_cooling;     ///< 发射机构的冷却速率 [0, 120] 需要精确到个位
  uint16_t rfr_blt_spd : 12;  ///< 弹丸速度的理论范围 [0, 40.0m/s] 需要精确到小数点后 2 位
  uint16_t rfr_st_heat : 10;  ///< 发射机构的热量 [0, 650] 需要精确到个位
  uint16_t rfr_is_rfr_gimbal_power_on : 1;
  uint16_t rfr_is_rfr_shooter_power_on : 1;
  uint16_t rfr_is_rfr_on : 1;
  uint16_t rfr_is_new_bullet_shot : 1;
  static void encode(GimbalChassisComm &gc_comm, uint8_t *tx_data);

  static void decode(GimbalChassisComm &gc_comm, const uint8_t *rx_data);
};
static_assert(sizeof(C2GPkg2) <= 8, "C2GPkg2 size error");

struct __attribute__((packed)) G2CPkg1 {
  uint8_t pkg_type;
  // robot
  uint8_t gimbal_board_is_imu_ready : 1;
  // gimbal
  uint8_t gimbal_pwr_state : 2;
  int8_t gimbal_pitch_ref;  ///< [-PI, PI]
  int8_t gimbal_pitch_fdb;  ///< [-PI, PI]
  int8_t gimbal_yaw_ref;    ///< [-PI, PI]
  int8_t gimbal_yaw_fdb;    ///< [-PI, PI]

  // vision
  uint8_t vision_vtm_x;
  uint8_t vision_vtm_y;

  static void encode(GimbalChassisComm &gc_comm, uint8_t *tx_data);

  static void decode(GimbalChassisComm &gc_comm, const uint8_t *rx_data);
};

static_assert(sizeof(G2CPkg1) <= 8, "Gimbal2ChassisPkg size error");
struct __attribute__((packed)) G2CPkg2 {
  uint8_t pkg_type;
  // shooter
  uint8_t shooter_is_fric_stuck : 1;
  uint8_t shooter_feed_stuck_state : 2;
  int8_t shooter_fric_spd_ref;
  int8_t shooter_fric_spd_fdb;
  int8_t shooter_feed_ang_ref;
  int8_t shooter_feed_ang_fdb;
  // scope
  uint8_t scope_pwr_state : 2;
  int8_t scope_ang;
  // shooter

  static void encode(GimbalChassisComm &gc_comm, uint8_t *tx_data);

  static void decode(GimbalChassisComm &gc_comm, const uint8_t *rx_data);
};
static_assert(sizeof(G2CPkg2) <= 8, "Gimbal2ChassisPkg size error");
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

bool GimbalChassisComm::decode(size_t len, const uint8_t* data)
{
  if (data == nullptr || len != 8) {
    return false;
  }
  if (code_part_ == CodePart::Chassis) {
    decodeG2C(data);
  } else if (code_part_ == CodePart::Gimbal) {
    decodeC2G(data);
  } else {
    return false;
  }
  oc_.update();
  is_update_ = true;
  return true;
};

bool GimbalChassisComm::encode(size_t& len, uint8_t* data)
{
  if (data == nullptr) {
    return false;
  }

  if (code_part_ == CodePart::Chassis) {
    encodeC2G(data);
  } else if (code_part_ == CodePart::Gimbal) {
    encodeG2C(data);
  } else {
    return false;
  }
  len = 8;
  return true;
};

void GimbalChassisComm::encodeG2C(uint8_t tx_data[8])
{
  if (g2c_seq_ % 2 == 0) {
    G2CPkg1::encode(*this, tx_data);
    tx_data[0] = 1;
  } else {
    G2CPkg2::encode(*this, tx_data);
    tx_data[0] = 2;
  }
  g2c_seq_++;
};

void GimbalChassisComm::decodeG2C(const uint8_t rx_data[8])
{
  if (rx_data[0] == 1) {
    G2CPkg1::decode(*this, rx_data);

  } else if (rx_data[0] == 2) {
    G2CPkg2::decode(*this, rx_data);
  }
}

void GimbalChassisComm::encodeC2G(uint8_t tx_data[8])
{
  if (g2c_seq_ % 2 == 0) {
    C2GPkg1::encode(*this, tx_data);
    tx_data[0] = 1;
  } else {
    C2GPkg2::encode(*this, tx_data);
    tx_data[0] = 2;
  }
  g2c_seq_++;
};
void GimbalChassisComm::decodeC2G(const uint8_t rx_data[8])
{
  if (rx_data[0] == 1) {
    C2GPkg1::decode(*this, rx_data);
  } else if (rx_data[0] == 2) {
    C2GPkg2::decode(*this, rx_data);
  }
};
/* Private function definitions ----------------------------------------------*/

void C2GPkg1::encode(GimbalChassisComm &gc_comm, uint8_t *tx_data)
{
  C2GPkg1 *pkg_ptr = (C2GPkg1 *)tx_data;
  // gimbal
  pkg_ptr->gimbal_yaw_delta = hello_world::Bound(gc_comm.gimbal_data().cp.yaw_delta, -1.0f, 1.0f) * 127;
  pkg_ptr->gimbal_pitch_delta = hello_world::Bound(gc_comm.gimbal_data().cp.pitch_delta, -1.0f, 1.0f) * 127;
  pkg_ptr->gimbal_turn_back_flag = gc_comm.gimbal_data().cp.turn_back_flag;
  pkg_ptr->gimbal_ctrl_mode = (uint32_t)gc_comm.gimbal_data().cp.ctrl_mode;
  pkg_ptr->gimbal_working_mode = (uint32_t)gc_comm.gimbal_data().cp.working_mode;
  // shooter
  pkg_ptr->shoot_flag = gc_comm.shooter_data().cp.shoot_flag(true);
  pkg_ptr->shooter_ctrl_mode = (uint32_t)gc_comm.shooter_data().cp.ctrl_mode;
  //pkg_ptr->shooter_working_mode = (uint32_t)gc_comm.shooter_data().cp.working_mode;
  // scope
  pkg_ptr->scope_switch_flag = gc_comm.scope_data().cp.switch_flag;
  pkg_ptr->scope_ctrl_ang_flag = gc_comm.scope_data().cp.ctrl_angle_flag;
  pkg_ptr->scope_working_mode = (uint32_t)gc_comm.scope_data().cp.working_mode;
};

void C2GPkg1::decode(GimbalChassisComm &gc_comm, const uint8_t *rx_data)
{
  C2GPkg1 *pkg_ptr = (C2GPkg1 *)rx_data;
  // gimbal
  gc_comm.gimbal_data().cp.yaw_delta = hello_world::Bound(pkg_ptr->gimbal_yaw_delta / 127.0f, -1.0f, 1.0f);
  gc_comm.gimbal_data().cp.pitch_delta = hello_world::Bound(pkg_ptr->gimbal_pitch_delta / 127.0f, -1.0f, 1.0f);
  gc_comm.gimbal_data().cp.turn_back_flag = pkg_ptr->gimbal_turn_back_flag;
  gc_comm.gimbal_data().cp.ctrl_mode = (CtrlMode)pkg_ptr->gimbal_ctrl_mode;
  gc_comm.gimbal_data().cp.working_mode = (GimbalWorkingMode)pkg_ptr->gimbal_working_mode;
  // shooter
  gc_comm.shooter_data().cp.setShootFlag(pkg_ptr->shoot_flag);
  gc_comm.shooter_data().cp.ctrl_mode = (CtrlMode)pkg_ptr->shooter_ctrl_mode;
  //gc_comm.shooter_data().cp.working_mode = (ShooterWorkingMode)pkg_ptr->shooter_working_mode;
  // scope
  gc_comm.scope_data().cp.switch_flag = pkg_ptr->scope_switch_flag;
  gc_comm.scope_data().cp.ctrl_angle_flag = pkg_ptr->scope_ctrl_ang_flag;
  gc_comm.scope_data().cp.working_mode = (ScopeWorkingMode)pkg_ptr->scope_working_mode;
};

void C2GPkg2::encode(GimbalChassisComm &gc_comm, uint8_t *tx_data)
{
  C2GPkg2 *pkg_ptr = (C2GPkg2 *)tx_data;
  // rfr
  pkg_ptr->rfr_robot_id = (uint8_t)gc_comm.referee_data().cp.robot_id;
  pkg_ptr->rfr_heat_limit = gc_comm.referee_data().cp.shooter_heat_limit / 10;
  pkg_ptr->rfr_st_cooling = gc_comm.referee_data().cp.shooter_cooling;
  pkg_ptr->rfr_blt_spd = (uint16_t)(hello_world::Bound(gc_comm.referee_data().cp.bullet_speed, 0.0f, 40.0f) * 100);
  pkg_ptr->rfr_st_heat = gc_comm.referee_data().cp.shooter_heat;
  pkg_ptr->rfr_is_rfr_gimbal_power_on = gc_comm.referee_data().cp.is_rfr_gimbal_power_on;
  pkg_ptr->rfr_is_rfr_shooter_power_on = gc_comm.referee_data().cp.is_rfr_shooter_power_on;
  pkg_ptr->rfr_is_rfr_on = gc_comm.referee_data().cp.is_rfr_on;
  pkg_ptr->rfr_is_new_bullet_shot = gc_comm.referee_data().cp.is_new_bullet_shot;
};

void C2GPkg2::decode(GimbalChassisComm &gc_comm, const uint8_t *rx_data)
{
  C2GPkg2 *pkg_ptr = (C2GPkg2 *)rx_data;
  // rfr
  gc_comm.referee_data().cp.robot_id = (GimbalChassisComm::RobotId)pkg_ptr->rfr_robot_id;
  gc_comm.referee_data().cp.shooter_heat_limit = pkg_ptr->rfr_heat_limit * 10;
  gc_comm.referee_data().cp.shooter_cooling = pkg_ptr->rfr_st_cooling;
  gc_comm.referee_data().cp.bullet_speed = pkg_ptr->rfr_blt_spd / 100.0;
  gc_comm.referee_data().cp.shooter_heat = pkg_ptr->rfr_st_heat;
  gc_comm.referee_data().cp.is_rfr_gimbal_power_on = pkg_ptr->rfr_is_rfr_gimbal_power_on;
  gc_comm.referee_data().cp.is_rfr_shooter_power_on = pkg_ptr->rfr_is_rfr_shooter_power_on;
  gc_comm.referee_data().cp.is_rfr_on = pkg_ptr->rfr_is_rfr_on;
  gc_comm.referee_data().cp.is_new_bullet_shot = pkg_ptr->rfr_is_new_bullet_shot;
};

void G2CPkg1::encode(GimbalChassisComm &gc_comm, uint8_t *tx_data)
{
  G2CPkg1 *pkg_ptr = (G2CPkg1 *)tx_data;
  // robot
  pkg_ptr->gimbal_board_is_imu_ready = gc_comm.main_board_data().gp.is_gimbal_imu_ready;
  // gimbal
  pkg_ptr->gimbal_pwr_state = (uint8_t)gc_comm.gimbal_data().gp.pwr_state;
  // 都是周期数据，溢出不影响取值
  pkg_ptr->gimbal_pitch_ref = gc_comm.gimbal_data().gp.pitch_ref * 127.0f / M_PI;
  pkg_ptr->gimbal_pitch_fdb = gc_comm.gimbal_data().gp.pitch_fdb * 127.0f / M_PI;
  pkg_ptr->gimbal_yaw_ref = gc_comm.gimbal_data().gp.yaw_ref * 127.0f / M_PI;
  pkg_ptr->gimbal_yaw_fdb = gc_comm.gimbal_data().gp.yaw_fdb * 127.0f / M_PI;
  // vision
  pkg_ptr->vision_vtm_x = gc_comm.vision_data().gp.vtm_x;
  pkg_ptr->vision_vtm_y = gc_comm.vision_data().gp.vtm_y;
};

void G2CPkg1::decode(GimbalChassisComm &gc_comm, const uint8_t *rx_data)
{
  G2CPkg1 *pkg_ptr = (G2CPkg1 *)rx_data;
  // robot
  gc_comm.main_board_data().gp.is_gimbal_imu_ready = pkg_ptr->gimbal_board_is_imu_ready;
  // gimbal
  gc_comm.gimbal_data().gp.pwr_state = (PwrState)pkg_ptr->gimbal_pwr_state;
  // 都是周期数据，溢出不影响取值
  gc_comm.gimbal_data().gp.pitch_ref = pkg_ptr->gimbal_pitch_ref * M_PI / 127.0f;
  gc_comm.gimbal_data().gp.pitch_fdb = pkg_ptr->gimbal_pitch_fdb * M_PI / 127.0f;
  gc_comm.gimbal_data().gp.yaw_ref = pkg_ptr->gimbal_yaw_ref * M_PI / 127.0f;
  gc_comm.gimbal_data().gp.yaw_fdb = pkg_ptr->gimbal_yaw_fdb * M_PI / 127.0f;
  // vision
  gc_comm.vision_data().gp.vtm_x = pkg_ptr->vision_vtm_x;
  gc_comm.vision_data().gp.vtm_y = pkg_ptr->vision_vtm_y;
};

void G2CPkg2::encode(GimbalChassisComm &gc_comm, uint8_t *tx_data)
{
  G2CPkg2 *pkg_ptr = (G2CPkg2 *)tx_data;
  // shooter
  pkg_ptr->shooter_is_fric_stuck = gc_comm.shooter_data().gp.is_fric_stuck_;
  pkg_ptr->shooter_feed_stuck_state = gc_comm.shooter_data().gp.feed_stuck_state;
  pkg_ptr->shooter_fric_spd_ref = gc_comm.shooter_data().gp.fric_spd_ref * 127.0f / 840.0f;
  pkg_ptr->shooter_fric_spd_fdb = gc_comm.shooter_data().gp.fric_spd_fdb * 127.0f / 840.0f;
  pkg_ptr->shooter_feed_ang_ref = gc_comm.shooter_data().gp.feed_ang_ref * 127.0f / M_PI;
  pkg_ptr->shooter_feed_ang_fdb = gc_comm.shooter_data().gp.feed_ang_fdb * 127.0f / M_PI;
  // scope
  pkg_ptr->scope_pwr_state = (uint8_t)gc_comm.scope_data().gp.pwr_state;
  pkg_ptr->scope_ang = gc_comm.scope_data().gp.scope_ang * 127.0f / M_PI;
};

void G2CPkg2::decode(GimbalChassisComm &gc_comm, const uint8_t *rx_data)
{
  G2CPkg2 *pkg_ptr = (G2CPkg2 *)rx_data;
  // shooter
  gc_comm.shooter_data().gp.is_fric_stuck_ = pkg_ptr->shooter_is_fric_stuck;
  gc_comm.shooter_data().gp.feed_stuck_state = pkg_ptr->shooter_feed_stuck_state;
  gc_comm.shooter_data().gp.fric_spd_ref = pkg_ptr->shooter_fric_spd_ref * 840.0f / 127.0f;
  gc_comm.shooter_data().gp.fric_spd_fdb = pkg_ptr->shooter_fric_spd_fdb * 840.0f / 127.0f;
  gc_comm.shooter_data().gp.feed_ang_ref = pkg_ptr->shooter_feed_ang_ref * M_PI / 127.0f;
  gc_comm.shooter_data().gp.feed_ang_fdb = pkg_ptr->shooter_feed_ang_fdb * M_PI / 127.0f;
  // scope
  gc_comm.scope_data().gp.pwr_state = (PwrState)pkg_ptr->scope_pwr_state;
  gc_comm.scope_data().gp.scope_ang = pkg_ptr->scope_ang * M_PI / 127.0f;
};

}  // namespace robot