#ifndef HERO_INS_PWR_LIMITER_HPP_
#define HERO_INS_PWR_LIMITER_HPP_
#include "power_limiter.hpp"

namespace hw_pwr_limiter = hello_world::power_limiter;

hw_pwr_limiter::PowerLimiter* CreatePwrLimiter();
#endif