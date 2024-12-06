/** 
 *******************************************************************************
 * @file      :ins_chassis_iksolver.cpp
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
#include "ins_chassis_iksolver.hpp"
/* Private constants ---------------------------------------------------------*/

const float kWheelRadius = 150 * 0.001 / 2;  ///< 轮子半径 [m]

const float kWheel2Center = 0.231f - 0.018f;  ///< 轮子中心距旋转中心的距离 [m]

const float kWheelBase = kWheel2Center * sqrt(2.0f);  ///< 左右轮距 [m]

const float kWheelTrack = kWheel2Center * sqrt(2.0f);  ///< 前后轮距 [m]

const hw_chassis_iksolver::PosVec kCenterPos = hw_chassis_iksolver::PosVec(0, 0);

/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_chassis_iksolver::ChassisIkSolver unique_chassis_iksolver = hw_chassis_iksolver::ChassisIkSolver(kCenterPos);
bool is_chassis_iksolver_init = false;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void InitChassisIkSolver(void)
{
  hw_chassis_iksolver::ChassisIkSolver& iksolver = unique_chassis_iksolver;
  if (iksolver.size() == 0) {
    hw_chassis_iksolver::WheelParams ik_solver_params[4] = {0};
    // 左前轮
    ik_solver_params[0].theta_vel_fdb = -PI * 1.0f / 4.0f;
    ik_solver_params[0].radius = kWheelRadius;
    ik_solver_params[0].wheel_pos = hw_chassis_iksolver::PosVec(kWheelTrack / 2, kWheelBase / 2);
    // 左后轮
    ik_solver_params[1].theta_vel_fdb = PI * 1.0f / 4.0f;
    ik_solver_params[1].radius = kWheelRadius;
    ik_solver_params[1].wheel_pos = hw_chassis_iksolver::PosVec(-kWheelTrack / 2, kWheelBase / 2);
    // 右后轮
    ik_solver_params[2].theta_vel_fdb = PI * 3.0f / 4.0f;
    ik_solver_params[2].radius = kWheelRadius;
    ik_solver_params[2].wheel_pos = hw_chassis_iksolver::PosVec(-kWheelTrack / 2, -kWheelBase / 2);

    // 右前轮
    ik_solver_params[3].theta_vel_fdb = -PI * 3.0f / 4.0f;
    ik_solver_params[3].radius = kWheelRadius;
    ik_solver_params[3].wheel_pos = hw_chassis_iksolver::PosVec(kWheelTrack / 2, -kWheelBase / 2);

    for (int i = 0; i < 4; i++) {
      iksolver.append(hw_chassis_iksolver::kOmni, ik_solver_params[i]);
    }
  }
}

/* Exported function definitions ---------------------------------------------*/
hw_chassis_iksolver::ChassisIkSolver* CreateChassisIkSolver(void)
{
  if (!is_chassis_iksolver_init) {
    is_chassis_iksolver_init = true;
    InitChassisIkSolver();
  }
  return &unique_chassis_iksolver;
};
/* Private function definitions ----------------------------------------------*/
