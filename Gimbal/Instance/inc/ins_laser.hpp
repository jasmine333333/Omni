/** 
 *******************************************************************************
 * @file      : ins_laser.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INSTANCE_INS_LASER_HPP_
#define INSTANCE_INS_LASER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "laser.hpp"

namespace hw_laser = hello_world::laser;

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hw_laser::Laser *CreateLaser(void);
#endif /* INSTANCE_INS_LASER_HPP_ */