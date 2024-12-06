#include "ins_pwr_limiter.hpp"
#include "power_limiter.hpp"

static const hw_pwr_limiter::PwrLimitStaticParams kStaticParams = {
    .z_ref_rfr =  10,       ///< 裁判系统功率控制下的缓冲能量收敛值
    .z_ref_cap =  10,       ///< 超级电容功率控制下的超电容量收敛值
    .z_danger = 5,          ///< 危险能量值，主要用于裁判系统功率控制
    .p_bias = 4.0f,         ///< 底盘静息功率
    .pref_kd = 30.0f,       ///< 期望功率PID微分系数
    .pwr_permission_max = 800,  ///< 可用功率最大值，默认800w
};

static const hw_pwr_limiter::PowerLimiter::MotorStaticParamsList kMotorStaticParamsList = {
   {
        .R = 1.0f,      ///< 电机铜损
        .ka = 0.1f,     ///< 电机加速度系数
        .kv = 0.125f,   ///< 电机转速系数
        .km = 0.3f,     ///< 电机转矩系数
        .type = hw_pwr_limiter::kMotor3508,  ///< 电机类型
   }
};

hw_pwr_limiter::PowerLimiter unique_pwr_limiter = hw_pwr_limiter::PowerLimiter(kMotorStaticParamsList, kStaticParams);

hw_pwr_limiter::PowerLimiter* CreatePwrLimiter() { return &unique_pwr_limiter; }