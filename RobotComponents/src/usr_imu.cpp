/** 
 *******************************************************************************
 * @file      :usr_imu.cpp
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
#include "usr_imu.hpp"
#include "base.hpp"
#include "spi.h"
/* Private macro -------------------------------------------------------------*/

namespace robot
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

bool Imu::initHardware()
{
  if (bmi088_ptr_ == nullptr) {
    status_ = ImuStatus::kPointerUninitErr;
    return false;
  }
  while (bmi088_ptr_->imuInit() != hello_world::imu::BMI088ErrState::kBMI088ErrStateNoErr) {
    status_ = ImuStatus::kInitHardwareFailedErr;
  }
  status_ = ImuStatus::kWaitCalcOffset;
  return true;
};

bool Imu::update()
{
  if (status_ == ImuStatus::kNotInitHardware) {
    if (!initHardware()) {
      return false;
    }
  }

  if (status_ >= ImuStatus::kInitHardwareFailedErr) {
    return false;
  }

  getRawData();

  if (status_ == ImuStatus::kWaitCalcOffset || status_ == ImuStatus::kCalcingOffset) {
    calcOffset();
  }

  updateAccGyro();

  updateMahony();

  updateSlopeAng();
  return true;
};

void Imu::getRawData() { bmi088_ptr_->getData(raw_acc_.data, raw_gyro_.data, &temp_); };

void Imu::calcOffset()
{
  if (offset_count_ < offset_max_count_) {
    float threshold = fabs(gyro_threshold_);

    for (size_t i = 0; i < 3; i++) {
      if (raw_gyro_.data[i] > threshold || raw_gyro_.data[i] < -threshold) {
        continue;
      }
      gyro_offset_.data[i] += ((raw_gyro_.data[i] - gyro_offset_.data[i]) / (float)(offset_count_ + 1));
    }
    status_ = ImuStatus::kCalcingOffset;
    offset_count_++;
  } else {
    status_ = ImuStatus::kNormWorking;
  }
  // gyro_offset_.data[0] = ;
  // gyro_offset_.data[1] = ;
  // gyro_offset_.data[2] = ;

  // status_ = ImuStatus::kNormWorking;
};

void Imu::updateAccGyro()
{
  float acc_threshold = fabs(acc_threshold_);
  for (size_t i = 0; i < 3; i++) {
    if (raw_acc_.data[i] > acc_threshold) {
      acc_.data[i] = acc_threshold;
    } else if (raw_acc_.data[i] < -acc_threshold) {
      acc_.data[i] = -acc_threshold;
    } else {
      acc_.data[i] = raw_acc_.data[i];
    }
  }

  for (size_t i = 0; i < 3; i++) {
    gyro_.data[i] = raw_gyro_.data[i] - gyro_offset_.data[i];
  }
};

bool Imu::updateMahony()
{
  if (ahrs_ptr_ == nullptr) {
    status_ = ImuStatus::kPointerUninitErr;
    return false;
  }
  ahrs_ptr_->update(acc_.data, gyro_.data);
  ahrs_ptr_->getEulerAngle(ang_.data);
  return true;
};

void Imu::updateSlopeAng()
{
  float sin_pitch, cos_pitch, sin_roll, cos_roll;
  arm_sin_cos_f32(hello_world::Rad2Deg(ang_.y), &sin_pitch, &cos_pitch);
  arm_sin_cos_f32(hello_world::Rad2Deg(ang_.x), &sin_roll, &cos_roll);
  slope_ang_ = acosf(cos_pitch * cos_roll);
  g_x_ = -sin_pitch;
  g_y_ = cos_pitch * sin_roll;
}

/* Private function prototypes -----------------------------------------------*/
}  // namespace robot