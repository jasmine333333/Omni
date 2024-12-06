/** 
 *******************************************************************************
 * @file      : ins_rfr.hpp
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
#ifndef INS_RFR_HPP_
#define INS_RFR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "referee.hpp"
#include "rfr_official_pkgs.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hello_world::referee::RobotPerformancePackage *CreateRobotPerformancePackage();
hello_world::referee::RobotPowerHeatPackage *CreateRobotPowerHeatPackage();
hello_world::referee::RobotShooterPackage *CreateRobotShooterPackage();
hello_world::referee::Referee *CreateReferee();

#endif /* INS_RFR_HPP_ */
