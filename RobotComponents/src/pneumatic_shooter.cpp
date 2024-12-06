/** 
 *******************************************************************************
 * @file      : pneumatic_shooter.cpp
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
#include "pneumatic_shooter.hpp"
/* Private macro -------------------------------------------------------------*/
namespace robot
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

bool PneumaticShooter::decode(size_t len, const uint8_t* data)
{
  if (data == nullptr || len != 8) {
    return false;
  }
  is_shoot_ready_ = (data[0] & 0x01) == 0x01;

  bullet_state_ = (BulletState)(data[1] & 0x03);

  is_bullet_cleared_ = (data[2] & 0x01) == 0x01;

  if (data[3] > (uint8_t)ShooterState::Finished) {
    shooter_state_ = ShooterState::Err;
  } else {
    shooter_state_ = (ShooterState)(data[3]);
  }

  is_air_bottle_ready_ = (data[4] & 0x01) == 0x01;

  is_air_pres_ready_ = (data[5] & 0x01) == 0x01;

  prop_valve_ref_ = ((float)data[6]) / 100.0f;

  is_update_ = true;
  return true;
};

bool PneumaticShooter::encode(size_t& len, uint8_t* data)
{
  if (data == nullptr) {
    return false;
  }

  data[0] = getShootFlag(true);
  uint16_t blt_spd = bullet_speed_ * 100;
  data[1] = (uint8_t)((blt_spd >> 8) & 0xff);
  data[2] = (uint8_t)(blt_spd & 0xff);

  len = 3;
  return true;
}

/* Private function definitions ----------------------------------------------*/

}  // namespace robot