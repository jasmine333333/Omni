/** 
 *******************************************************************************
 * @file      :ins_fsm.cpp
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
#include "ins_all.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
const robot::Gimbal::Config kGimbalConfig = {
    .sensitivity_yaw = 300 / 1000.0 * PI / 180.0,           ///< yaw角度灵敏度，单位 rad/ms
    .sensitivity_pitch = 300 / 1000.0 * PI / 180.0,  ///< pitch角度灵敏度，单位 rad/ms
    .max_pitch_ang = 0.52f,                                  ///< 最大俯仰角度，单位 rad
    .min_pitch_ang = -0.30f,                                 ///< 最小俯仰角度，单位 rad
    .max_pitch_torq = 0,  //0.8826                               ///< 云台水平时的重力矩，单位 N·m
    .pitch_center_offset = 0.00,                            ///??< 云台水平时，重心和pitch轴的连线与水平轴的夹角，单位 rad
};

/* const robot::Feed::Config kFeedConfig = {
    .feed_ang_ref_offset = 0.0f,  ///< 拨盘电机目标角度偏移量
    .feed_ang_per_blt = PI / 3,   ///< 1发弹丸拨盘的转动角度
    .heat_per_bullet = 100.0f,    ///< 1发弹耗热量

    .feed_stuck_curr_thre = 13.5f,  ///< 用于判断堵转的电流阈值
};

const robot::Fric::Config kFricConfig = {
    .min_bullet_speed = 14.0f,             ///< 最小合理弹丸射速 m/s
    .max_bullet_speed = 16.5f,             ///< 最大合理弹丸射速 m/s
    .tgt_max_bullet_speed = 15.9f,         ///< 最大目标弹丸速度 m/s
    .tgt_min_bullet_speed = 15.3f,         ///< 最小目标弹丸速度 m/s
    .tgt_fric_spd_ref = 640.0f,            ///< 摩擦轮期望速度预设值 rad/s
    .tgt_fric_spd_ref_backward = -100.0f,  ///< 摩擦轮反转目标速度

    .fric_stuck_curr_thre = 14.0f,  ///< 用于判断摩擦轮堵转的电流阈值
    .fric_spd_delta_thre = 10.0f,   ///< 用于判断摩擦轮速度保持恒定的阈值 (rad)
    .fric_spd_err_thre = 5.0f,      ///< 用于判断摩擦轮速度跟上期望转速的阈值 (rad)
};
 */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
robot::Gimbal unique_gimbal = robot::Gimbal(kGimbalConfig);
robot::Robot unique_robot = robot::Robot();
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Gimbal* CreateGimbal()
{
  static bool is_gimbal_created = false;
  if (!is_gimbal_created) {
    // 各组件指针
    // 无通信功能的组件指针
    unique_gimbal.registerPid(CreatePidMotorYaw(), robot::Gimbal::kJointYaw);
    unique_gimbal.registerPid(CreatePidMotorPitch(), robot::Gimbal::kJointPitch);

    unique_gimbal.registerTd(CreateTdYaw(), robot::Gimbal::kJointYaw);
    unique_gimbal.registerTd(CreateTdPitch(), robot::Gimbal::kJointPitch);

    // 只接收数据的组件指针
    unique_gimbal.registerImu(CreateImu());

    //  接收、发送数据的组件指针
    unique_gimbal.registerMotor(CreateMotorYaw(), robot::Gimbal::kJointYaw);
    unique_gimbal.registerMotor(CreateMotorPitch(), robot::Gimbal::kJointPitch);

    is_gimbal_created = true;
  }
  return &unique_gimbal;
};
/* robot::Feed* CreateFeed()
{
  static bool is_feed_created = false;
  if (!is_feed_created) {
    // 各组件指针
    // 无通信功能的组件指针
    unique_feed.registerPid(CreatePidMotorFeed(), robot::Feed::kPidIdxFeed);

    // 只接收数据的组件指针
    // 只发送数据的组件指针
    // 接收、发送数据的组件指针
    unique_feed.registerMotor(CreateMotorFeed(), robot::Feed::kMotorIdxFeed);

    is_feed_created = true;
  }
  return &unique_feed;
};
robot::Fric* CreateFric()
{
  static bool is_fric_created = false;
  if (!is_fric_created) {
    // 各组件指针
    // 无通信功能的组件指针
    unique_fric.registerPid(CreatePidMotorFricLeft(), robot::Fric::kPidIdxFricLeft);
    unique_fric.registerPid(CreatePidMotorFricRight(), robot::Fric::kPidIdxFricRight);

    // 只接收数据的组件指针
    // 只发送数据的组件指针
    // 接收、发送数据的组件指针
    unique_fric.registerMotor(CreateMotorFricLeft(), robot::Fric::kMotorIdxFricLeft);
    unique_fric.registerMotor(CreateMotorFricRight(), robot::Fric::kMotorIdxFricRight);

    is_fric_created = true;
  }
  return &unique_fric;
};  
 */
robot::Robot* CreateRobot()
{
  static bool is_robot_created = false;
  if (!is_robot_created) {
    // 各组件指针
    // 主要模块状态机组件指针
    unique_robot.registerGimbal(CreateGimbal());

    unique_robot.registerFeed(CreateFeed());
    unique_robot.registerFric(CreateFric());

    // 无通信功能的组件指针
    //unique_robot.registerBuzzer(CreateBuzzer());
    unique_robot.registerImu(CreateImu());

    // 有通信功能的组件指针

    hello_world::comm::CanTxMgr* can_tx_mgr_ptr;
    can_tx_mgr_ptr = CreateCan1TxMgr();
    unique_robot.registerGimbalChassisComm(CreateGimbalChassisComm(), can_tx_mgr_ptr);
    unique_robot.registerMotor(CreateMotorYaw(), robot::Robot::kMotorIdxYaw, can_tx_mgr_ptr);

    can_tx_mgr_ptr = CreateCan2TxMgr();
    unique_robot.registerMotor(CreateMotorFricLeft(), robot::Robot::kMotorIdxFricLeft, can_tx_mgr_ptr);
    unique_robot.registerMotor(CreateMotorFricRight(), robot::Robot::kMotorIdxFricRight, can_tx_mgr_ptr);
    unique_robot.registerMotor(CreateMotorFeed(), robot::Robot::kMotorIdxFeed, can_tx_mgr_ptr);
    unique_robot.registerMotor(CreateMotorPitch(), robot::Robot::kMotorIdxPitch, can_tx_mgr_ptr);

    hello_world::comm::UartTxMgr* uart_tx_mgr_ptr = CreateVisionTxMgr();
    unique_robot.registerVision(CreateVision(), uart_tx_mgr_ptr);

    is_robot_created = true;
  }
  return &unique_robot;
};
/* Private function definitions ----------------------------------------------*/
