#include "ins_pwr_limiter.hpp"
#include "power_limiter.hpp"
#include "main_task.hpp"
static const hw_pwr_limiter::PowerLimiter::StaticParams kMotorStaticParamsList_1 = {
    .wheel_motor_params = {
        .k1 = 0.285f,       ///<
        .k2 = 0.11f,        ///<
        .k3 = 0.15f,        ///< 电机转速系数
        .kp = 2.15f,        ///<
        .out_limit = 20.0f, ///< 输出限幅，单位：A
        .motor_cnt = 4,     ///< 电机数量
    },
    .p_bias = 2.6f,           ///< 底盘静息功率
    .p_steering_ratio = 0.0f, ///< 舵轮功率比例
};
static const hw_pwr_limiter::PowerLimiter::StaticParams kMotorStaticParamsList_2 = {
    .wheel_motor_params = {
        .k1 = 0.285f,       ///<
        .k2 = 0.11f,        ///<
        .k3 = 0.15f,        ///< 电机转速系数
        .kp = 2.15f,        ///<
        .out_limit = 20.0f, ///< 输出限幅，单位：A
        .motor_cnt = 4,     ///< 电机数量
    },
    .p_bias = 3.6f,           ///< 底盘静息功率
    .p_steering_ratio = 0.0f, ///< 舵轮功率比例
};

hw_pwr_limiter::PowerLimiter unique_pwr_limiter_1 = hw_pwr_limiter::PowerLimiter(kMotorStaticParamsList_1);
hw_pwr_limiter::PowerLimiter unique_pwr_limiter_2 = hw_pwr_limiter::PowerLimiter(kMotorStaticParamsList_2);

hw_pwr_limiter::PowerLimiter *CreatePwrLimiter() { return &unique_pwr_limiter_1; }
// hw_pwr_limiter::PowerLimiter* CreatePwrLimiter()
// {
//     if (car_version == 0)
//     {
//         return &unique_pwr_limiter_1;
//     }
//     else if(car_version == 1)
//     {
//         return &unique_pwr_limiter_2;
//     }
// }