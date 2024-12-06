/** 
 *******************************************************************************
 * @file      : ins_chassis_iksolver.hpp
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
#ifndef INSTANCE_INS_CHASSIS_IKSOLVER_HPP_
#define INSTANCE_INS_CHASSIS_IKSOLVER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "chassis_iksolver.hpp"

namespace hw_chassis_iksolver = hello_world::chassis_ik_solver;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hw_chassis_iksolver::ChassisIkSolver* CreateChassisIkSolver(void);

#endif /* INSTANCE_INS_CHASSIS_IKSOLVER_HPP_ */
