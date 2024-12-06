/** 
 *******************************************************************************
 * @file      : ins_offline_checker.cpp
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
#include "ins_offline_checker.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

robot::OfflineChecker unique_oc_motor_yaw(50);
robot::OfflineChecker unique_oc_motor_pitch(50);
robot::OfflineChecker unique_oc_motor_fric_left(50);
robot::OfflineChecker unique_oc_motor_fric_right(50);
robot::OfflineChecker unique_oc_motor_feed(50);
robot::OfflineChecker unique_oc_chassis_control_board(50);
robot::OfflineChecker unique_oc_vision(100);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

robot::OfflineChecker* CreateOcMotorYaw() { return &unique_oc_motor_yaw; };
robot::OfflineChecker* CreateOcMotorPitch() { return &unique_oc_motor_pitch; };
robot::OfflineChecker* CreateOcMotorFricLeft() { return &unique_oc_motor_fric_left; };
robot::OfflineChecker* CreateOcMotorFricRight() { return &unique_oc_motor_fric_right; };
robot::OfflineChecker* CreateOcMotorFeed() { return &unique_oc_chassis_control_board; };
robot::OfflineChecker* CreateOcChassisControlBoard() { return &unique_oc_chassis_control_board; };
robot::OfflineChecker* CreateOcVision() { return &unique_oc_vision; };

/* Private function definitions ----------------------------------------------*/
