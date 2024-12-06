/** 
 *******************************************************************************
 * @file      : ins_vision.hpp
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
#ifndef INSTANCE_INS_VISION_HPP_
#define INSTANCE_INS_VISION_HPP_

/* Includes ------------------------------------------------------------------*/
#include "vision.hpp"
namespace hw_vision = hello_world::vision;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hw_vision::Vision* CreateVision();

#endif /* INSTANCE_INS_VISION_HPP_ */
