/** 
 *******************************************************************************
 * @file      :main_task.cpp
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
#include "main_task.hpp"

#include "comm_task.hpp"
#include "communication_tools.hpp"
#include "ins_all.hpp"
#include "tim.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static robot::Robot* robot_ptr = nullptr;
static robot::Imu* imu_ptr = nullptr;

static float main_task_time_cost = 0;
static float comm_task_time_cost = 0;
static uint32_t tick = 0;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void PrivatePointerInit(void);
static void HardWareInit(void);

void MainTaskInit(void)
{ 
  PrivatePointerInit();
  HardWareInit();

  CommTaskInit();
};
void MainTask(void)
{
  HW_ASSERT(robot_ptr != nullptr, "robot::Robot is nullptr", robot_ptr);
  robot_ptr->update();
  robot_ptr->run();
};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  int64_t start_time;
  int64_t end_time;
  int64_t time_cost;
  if (htim == &htim6) {
    tick++;
    start_time = __HAL_TIM_GET_COUNTER(&htim2);
    MainTask();
    end_time = __HAL_TIM_GET_COUNTER(&htim2);
    time_cost = end_time - start_time;
    if (time_cost < 0) {
      time_cost += std::numeric_limits<uint32_t>::max();
    }
    main_task_time_cost = time_cost / (1000.0f * 84.0f);

    start_time = __HAL_TIM_GET_COUNTER(&htim6);
    CommTask();
    end_time = __HAL_TIM_GET_COUNTER(&htim6);
    time_cost = end_time - start_time;
    if (time_cost < 0) {
      time_cost += std::numeric_limits<uint32_t>::max();
    }
    comm_task_time_cost = time_cost / (1000.0f * 84.0f);
  }
}

static void PrivatePointerInit(void)
{
  imu_ptr = CreateImu();
  robot_ptr = CreateRobot();
};
static void HardWareInit(void)
{
  // IMU Init
  imu_ptr->initHardware();
  // buzzer init
  // TODO: 等后续组件更新
  // buzzer_ptr->initHardware();
};