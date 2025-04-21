/** 
 *******************************************************************************
 * @file      :ins_rfr.cpp
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
#include "ins_rfr.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hello_world::referee::RobotPerformancePackage unique_robot_performance_package;
hello_world::referee::RobotPowerHeatPackage unique_robot_power_heat_package;
hello_world::referee::RobotShooterPackage unique_robot_shooter_package;
hello_world::referee::CompRobotsHpPackage unique_comp_robots_hp_package; 
hello_world::referee::RobotHurtPackage unique_robot_hurt_package;
hello_world::referee::Referee unique_referee;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

hello_world::referee::RobotPerformancePackage *CreateRobotPerformancePackage() { return &unique_robot_performance_package; };
hello_world::referee::RobotPowerHeatPackage *CreateRobotPowerHeatPackage() { return &unique_robot_power_heat_package; };
hello_world::referee::RobotShooterPackage *CreateRobotShooterPackage() { return &unique_robot_shooter_package; };
hello_world::referee::CompRobotsHpPackage *CreateCompRobotsHpPackage() { return &unique_comp_robots_hp_package; };
hello_world::referee::RobotHurtPackage *CreateRobotHurtPackage() { return &unique_robot_hurt_package; };
hello_world::referee::Referee *CreateReferee(void)
{
  unique_referee.appendRxPkg(CreateRobotPerformancePackage());
  unique_referee.appendRxPkg(CreateRobotPowerHeatPackage());
  unique_referee.appendRxPkg(CreateRobotShooterPackage());
  unique_referee.appendRxPkg(CreateCompRobotsHpPackage());
  unique_referee.appendRxPkg(CreateRobotHurtPackage());
  return &unique_referee;
};

/* Private function definitions ----------------------------------------------*/
