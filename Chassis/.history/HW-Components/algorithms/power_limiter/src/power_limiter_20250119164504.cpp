

/**
 * @file      :power_limiter.cpp
 * @brief     :
 *@author     : Yang Zhou (3200105353@zju.edu.cn)
 * @history   :
 *@date       : 2024-08-28
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 *  All Rights Reserved.
 *
 *******************************************************************************
 *| Version | Date | Author | Description |
 *******************************************************************************
 *******************************************************************************
 *@par last editor : Yang Zhou (3200105353@zju.edu.cn)
 *@par last edit time : 2024-08-28
 * @attention :
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 */
/* Includes ------------------------------------------------------------------*/
#include "power_limiter.hpp"

#include "assert.hpp"
#include "base.hpp"
#include "pid.hpp"
#include "string.h"
namespace hello_world
{
namespace power_limiter
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
PowerLimiter ::PowerLimiter(const MotorStaticParamsList &motor_static_list, const PwrLimitStaticParams &pwr_limit_static_params)
{
  memcpy(&this->pwr_limit_static_params_, &pwr_limit_static_params, sizeof(PwrLimitStaticParams));
  for (auto tmp = motor_static_list.begin(); tmp != motor_static_list.end(); ++tmp) {
    this->motor_static_list_.push_back(*tmp);
    motor_static_params_[tmp->type] = *tmp;
  }
  if (p_ref_pid_ == nullptr) {
    p_ref_pid_ = new PwrPid();
  }
}

void PowerLimiter::Init(const MotorStaticParamsList &motor_static_list, const PwrLimitStaticParams &pwr_limit_static_params)
{
  memcpy(&this->pwr_limit_static_params_, &pwr_limit_static_params, sizeof(PwrLimitStaticParams));
  for (auto tmp = motor_static_list.begin(); tmp != motor_static_list.end(); ++tmp) {
    this->motor_static_list_.push_back(*tmp);
    motor_static_params_[tmp->type] = *tmp;
  }
  if (p_ref_pid_ == nullptr) {
    p_ref_pid_ = new PwrPid();
  }
}

void PowerLimiter::setStaticParams(const PwrLimitStaticParams &pwr_limit_static_params)
{
  memcpy(&this->pwr_limit_static_params_, &pwr_limit_static_params, sizeof(PwrLimitStaticParams));
}

void PowerLimiter::PwrLimitUpdateRuntimeParams(const PwrLimitRuntimeParams &params)
{
  memcpy(&this->pwr_limit_runtime_params_, &params, sizeof(PwrLimitRuntimeParams));
}

void PowerLimiter::PwrLimitCalcSpd(const MotorRuntimeParamsList &motor_run_par_list, float *limited_spd_ref_radps)
{
  HW_ASSERT(limited_spd_ref_radps != nullptr, "limited_spd_ref_radps must not be nullptr");

  this->motor_run_par_list_ = motor_run_par_list;
  /* 判断各模块在线情况，决定使用的功率限制源 */
  uint16_t z_measure;
  uint16_t p_max;
  uint16_t z_ref;
  PwrLimitCapMode super_cap_mode;
  /*根据模块在线状态，决定当前功率限制使用的数据*/
  if (pwr_limit_runtime_params_.is_super_cap_online == true) {
    z_ref = pwr_limit_static_params_.z_ref_cap;
    super_cap_mode = pwr_limit_runtime_params_.super_cap_mode;
  } else {
    z_ref = pwr_limit_static_params_.z_ref_rfr;
    super_cap_mode = kPwrLimitSuperCapOff;
  }

  do {
    if (pwr_limit_runtime_params_.is_super_cap_online == true) {
      z_measure = pwr_limit_runtime_params_.z_dummy_measure;
      p_max = pwr_limit_runtime_params_.p_dummy_max;
    } else {
      z_measure = pwr_limit_runtime_params_.z_rfr_measure;
      p_max = pwr_limit_runtime_params_.p_rfr_max;
    }
    /* 当缓冲能量小于danger区域时，表示机器人即将超功率扣血，需停下保证血量*/
    if (z_measure < pwr_limit_static_params_.z_danger) {
      pwr_limit_calc_data_.k_limit = 0;
      break;
    }
    /*计算P_ref*/
    if (pwr_limit_runtime_params_.is_referee_online) {
      pwr_limit_calc_data_.p_ref = PwrLimitCalcPref(p_ref_pid_,
                                                    super_cap_mode,
                                                    p_max,
                                                    z_ref, z_measure);
    } else {
      /*裁判系统离线下，动态最高功率为裁判系统功率*/
      pwr_limit_calc_data_.p_ref = p_max;
    }

    float tmp1 = 0, tmp2 = 0;
    float motor_R = 0, motor_km = 0, kv = 0, ka = 0;
    tmp1 = tmp2 = 0;
    /*模型计算当前功率--供观察模型效果，顺便计算各自轮子的iq_ref*/
    pwr_limit_calc_data_.p_model = pwr_limit_static_params_.p_bias;
    pwr_limit_calc_data_.p_model_wheel = 0;
    pwr_limit_calc_data_.p_model_steer = 0;
    for (auto tmp = motor_run_par_list_.begin(); tmp != motor_run_par_list_.end(); ++tmp, tmp1 = tmp2 = 0) {
      /*根据电流模型，计算iq_ref = kv * v + ka * a */
      if (tmp->type == kMotorWheel) {
        tmp->iq_ref = motor_static_params_[tmp->type].kv * tmp->spd_ref_radps + motor_static_params_[tmp->type].ka * (tmp->spd_ref_radps - tmp->spd_measure_radps);
        tmp->iq_ref = hello_world::Bound(tmp->iq_ref, -20, 20);
      }
      /*根据整个电机模型，计算功率*/
      tmp1 = tmp->spd_measure_radps * tmp->iq_measure;
      tmp2 = motor_static_params_[tmp->type].R * tmp->iq_measure * tmp->iq_measure;
      pwr_limit_calc_data_.p_model += motor_static_params_[tmp->type].km * tmp1 + tmp2;
      if (tmp->type == kMotorWheel) {
        pwr_limit_calc_data_.p_model_wheel += motor_static_params_[tmp->type].km * tmp1 + tmp2;
      } else {
        pwr_limit_calc_data_.p_model_steer += motor_static_params_[tmp->type].km * tmp1 + tmp2;
      }
    }

    /*预测功率*/
    tmp1 = tmp2 = motor_R = motor_km = kv = ka = 0;
    pwr_limit_calc_data_.p_prediction = pwr_limit_static_params_.p_bias;
    pwr_limit_calc_data_.p_prediction_wheel = 0;
    pwr_limit_calc_data_.p_prediction_steer = 0;
    /*单独计算每个轮子的期望功率，然后叠加得到P_prediction*/
    for (auto tmp = motor_run_par_list_.begin(); tmp != motor_run_par_list_.end(); ++tmp, tmp1 = tmp2 = 0) {
      /*根据整个电机模型，计算功率*/
      tmp1 = tmp->spd_ref_radps * tmp->iq_ref;
      tmp2 = motor_static_params_[tmp->type].R * tmp->iq_ref * tmp->iq_ref;
      pwr_limit_calc_data_.p_prediction += motor_static_params_[tmp->type].km * tmp1 + tmp2;
      if (tmp->type == kMotorWheel) {
        pwr_limit_calc_data_.p_prediction_wheel += motor_static_params_[tmp->type].km * tmp1 + tmp2;
      } else {
        pwr_limit_calc_data_.p_prediction_steer += motor_static_params_[tmp->type].km * tmp1 + tmp2;
      }
    }

    float alpha = 0, beta = 0, gamma = 0;
    
    /*当预测功率P_prediction小于动态P_ref时，不限制功率*/
    if (pwr_limit_calc_data_.p_prediction < pwr_limit_calc_data_.p_ref) {
      pwr_limit_calc_data_.k_limit = 1.0f;
      break;
    } else {
      float spd_ref_pow, spd_measure_ref, spd_measure_pow, constant;
      spd_ref_pow = spd_measure_pow = spd_measure_ref = constant = 0;
      tmp1 = tmp2 = motor_R = motor_km = kv = ka = 0;
      /*为每个轮子计算一元二次方程的系数 alpha beta gamma，然后叠加*/
      for (auto tmp = motor_run_par_list_.begin(); tmp != motor_run_par_list_.end(); ++tmp) {
        if (tmp->type == kMotorSteer) {
          continue;
        }
        spd_ref_pow = spd_measure_pow = spd_measure_ref = constant = 0;
        /*根据type决定使用电机的静态参数*/
        motor_R = motor_static_params_[tmp->type].R;
        motor_km = motor_static_params_[tmp->type].km;
        kv = motor_static_params_[tmp->type].kv;
        ka = motor_static_params_[tmp->type].ka;
        /*计算各轮子alpha beta gamma*/
        spd_ref_pow = tmp->spd_ref_radps * tmp->spd_ref_radps;
        spd_measure_pow = tmp->spd_measure_radps * tmp->spd_measure_radps;
        spd_measure_ref = tmp->spd_measure_radps * tmp->spd_ref_radps;

        alpha += motor_R * kv * kv * spd_ref_pow + 2 * motor_R * ka * kv * spd_ref_pow + motor_R * ka * ka * spd_ref_pow;
        beta += motor_km * (ka + kv) * spd_measure_ref - 2 * motor_R * (ka * kv + ka * ka) * spd_measure_ref;
        gamma += motor_R * ka * ka * spd_measure_pow - motor_km * ka * spd_measure_pow;
      }
      constant = pwr_limit_static_params_.p_bias + pwr_limit_calc_data_.p_prediction_steer;
      gamma += constant - pwr_limit_calc_data_.p_ref;
      /*求根公式判断是否存在可行解*/
      tmp1 = beta * beta - 4 * alpha * gamma;
      /*根据求根公式结果，进行解*/
      if (alpha != 0) {
        if (tmp1 > 0) {
            pwr_limit_calc_data_.k_limit = (-beta + sqrt(tmp1)) / (2 * alpha);
        } else if (tmp1 <= 0)
          pwr_limit_calc_data_.k_limit = -beta / (2 * alpha);
      } else if (beta != 0) {
        pwr_limit_calc_data_.k_limit = (-gamma) / beta;
      } else {
        pwr_limit_calc_data_.k_limit = 0;
      }
    }
  } while (0);
  pwr_limit_calc_data_.k_limit = hello_world::Bound(pwr_limit_calc_data_.k_limit, 0, 1);
  size_t cnt = 0;
  for (auto tmp = motor_run_par_list_.begin(); tmp != motor_run_par_list_.end(); ++tmp) {
    if (tmp->type == kMotorWheel) {
      limited_spd_ref_radps[cnt] = pwr_limit_calc_data_.k_limit * tmp->spd_ref_radps;
      cnt++;
    }
  }
}

float PowerLimiter::PwrLimitCalcPref(PwrPid *pid, PwrLimitCapMode mode, float p_max, float z_ref, float z_measure)
{
  float u_z;
  float p_ref;
  hello_world::pid::BasicPidParams p_ref_pid_params =
      {
          .auto_reset = true,  ///< 是否自动清零
          .kp = 4.0f,
          .ki = 0.0f,
          .kd = this->pwr_limit_static_params_.pref_kd,
          .diff_filter = pid::DiffFilter(true, -10000.0f, 10000.0f, 0.9f),
          .out_limit = pid::OutLimit(true, -150.0f, 1000),
      };

  switch (mode) {
    case kPwrLimitSuperCapOff:
    case kPwrLimitSuperCapNormal:
      p_ref_pid_params.kp = p_max / z_ref;
      pid->params() = p_ref_pid_params;
      pid->calc(&z_ref, &z_measure, nullptr, &u_z);
      break;
    case kPwrLimitSuperCapBoost:
      p_ref_pid_params.kp = p_max / z_ref * 10;
      pid->params() = p_ref_pid_params;
      pid->calc(&z_ref, &z_measure, nullptr, &u_z);
      break;
    default:
      p_ref_pid_params.kp = 1.0f;
  };

  p_ref = p_max - u_z;
  float low_bound = p_max * 0.9;
  low_bound = hello_world::Bound(low_bound, 0, 80);
  p_ref = hello_world ::Bound(p_ref, low_bound, pwr_limit_static_params_.pwr_permission_max);
  return p_ref;
}

}  // namespace power_limiter
}  // namespace hello_world