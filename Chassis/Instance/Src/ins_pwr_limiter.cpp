#include "ins_pwr_limiter.hpp"
#include "power_limiter.hpp"

// static const hw_pwr_limiter::PwrLimitStaticParams kStaticParams = {
//     .z_ref_rfr =  30,       ///< 裁判系统功率控制下的缓冲能量收敛值
//     .z_ref_cap =  30,       ///< 超级电容功率控制下的超电容量收敛值
//     .z_danger = 5,          ///< 危险能量值，主要用于裁判系统功率控制
//     .p_bias = 3.35f,         ///< 底盘静息功率
//     .pref_kd = 0.0f,       ///< 期望功率PID微分系数
//     .pwr_permission_max = 800,  ///< 可用功率最大值，默认800w
// };

static const hw_pwr_limiter::PowerLimiter::StaticParams kMotorStaticParamsList = {
        .wheel_motor_params = {
            .k1 = 0.265f,      ///< 电机铜损
            .k2 = 0.11f,      ///< 电机加速度系数
            .k3 = 0.15f,         ///< 电机转速系数
            .kp = 2.15f,       ///< 
            .out_limit = 20.0f, ///< 输出限幅，单位：A
            .motor_cnt = 4,     ///< 电机数量
        },
        .p_bias = 2.6f,         ///< 底盘静息功率
        .p_steering_ratio = 0.0f,  ///< 舵轮功率比例

};

hw_pwr_limiter::PowerLimiter unique_pwr_limiter = hw_pwr_limiter::PowerLimiter(kMotorStaticParamsList);

hw_pwr_limiter::PowerLimiter* CreatePwrLimiter() { return &unique_pwr_limiter; }