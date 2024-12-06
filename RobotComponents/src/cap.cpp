/** 
 *******************************************************************************
 * @file      : cap.cpp
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
#include "cap.hpp"
/* Private macro -------------------------------------------------------------*/
namespace robot
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/**
 * @brief 启用或禁用超级电容
 * 
 * 如果之前已经在放电，则允许超电电压降至 kMinEnableVoltage 以下。
 * 否则，只有当超电电压大于等于 kMinEnableVoltage 时才允许开启超电。
 * 此举是为了防止操作手在不能完全不能放电时一直尝试开启超电。
 * 只有当超电电压大于等于 kMinValidVoltage 时才允许开启超电。
 * 低于此电压超电不会放太多电流，反而会对功率限制有影响。
 * 
 * @return bool 返回当前的 enable_flag_ 状态。
 */
bool Cap::enable()
{
  // 记录上一次的 enable_flag_ 状态
  last_enable_flag_ = enable_flag_;

  // 如果之前已经在放电，则允许超电电压降至 kMinEnableVoltage 以下
  // 否则，只有当超电电压大于等于 kMinEnableVoltage 时才允许开启超电
  // 此举是为了防止操作手在不能完全不能放电时一直尝试开启超电
  if (last_enable_flag_) {
    enable_flag_ = true;
  } else {
    enable_flag_ = remaining_power_percent_ >= kMinEnablePercent;
  }

  // 只有当超电电压大于等于 kMinValidVoltage 时才允许开启超电
  // 低于此电压超电不会放太多电流，反而会对功率限制有影响
  enable_flag_ = enable_flag_ && (remaining_power_percent_ >= kAutoDisablePercent);
  return enable_flag_;
};

bool Cap::encode(size_t& len, uint8_t* data)
{
  if (data == nullptr) {
    return false;
  }

  // ! 编码这里能正常运行不溢出是因为缓冲能量和功率都不会超过 255
  data[0] = (uint8_t)(rfr_chas_pwr_buffer_);
  data[1] = (uint8_t)(enable_flag_) & 0x01;
  data[2] = (uint8_t)(rfr_chas_pwr_limit_);

  len = 3;
  return true;
};

bool Cap::decode(size_t len, const uint8_t* data)
{
  if (data == nullptr || len != 8) {
    return false;
  }
  voltage_ = (float)((int16_t)(data[0] << 8 | data[1]));
  voltage_ *= 0.001;
  pwr_src_ = (PwrSrc)(data[2] & 0x01);
  remaining_power_percent_ = getRemainingPowerPercent();

  is_update_ = true;
  return true;
};
/* Private function prototypes -----------------------------------------------*/
}  // namespace robot