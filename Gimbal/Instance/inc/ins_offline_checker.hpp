/** 
 *******************************************************************************
 * @file      : ins_offline_checker.hpp
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
#ifndef INSTANCE_INS_OFFLINE_CHECKER_HPP_
#define INSTANCE_INS_OFFLINE_CHECKER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "usr_offline_checker.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

robot::OfflineChecker* CreateOcMotorYaw();
robot::OfflineChecker* CreateOcMotorPitch();
robot::OfflineChecker* CreateOcMotorFricLeft();
robot::OfflineChecker* CreateOcMotorFricRight();
robot::OfflineChecker* CreateOcMotorFeed();
robot::OfflineChecker* CreateOcChassisControlBoard();
robot::OfflineChecker* CreateOcVision();

#endif /* INSTANCE_INS_OFFLINE_CHECKER_HPP_ */
