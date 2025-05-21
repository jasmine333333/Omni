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
/* Private constants ---------------------------------------------------------*/
const robot::Chassis::Config kChassisConfig = {
    .normal_trans_vel = 5.0f,   ///< 正常平移速度
    .normal_rot_spd = 13.0f,//13.0f   ///< 正常旋转速度
    .max_trans_vel = 5.0f,      ///< 最大平移速度
    .max_rot_spd = 15,      ///< 最大旋转速度
    .cmd_smooth_factor = 0.8f,  ///< 运动指令平滑系数, 值域[0,1], 启用超电时默认为1
};
const float robotmass = 19.0f;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static bool is_robot_inited = false;
static bool is_chassis_inited = false;

robot::Robot unique_robot = robot::Robot();
robot::Chassis unique_chassis = robot::Chassis(kChassisConfig);
robot::Feed unique_feed = robot::Feed();
robot::Gimbal unique_gimbal = robot::Gimbal();
robot::Shooter unique_shooter = robot::Shooter();

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Chassis* CreateChassis()
{
  if (!is_chassis_inited) {
    // * 1. 无通信功能的组件指针
    // * - 底盘逆解
    unique_chassis.registerIkSolver(CreateChassisIkSolver());
    // * - pid
    // 轮组 pid
    unique_chassis.registerWheelPid(CreatePidMotorWheelLeftFront(), robot::Chassis::kWheelPidIdxLeftFront);
    unique_chassis.registerWheelPid(CreatePidMotorWheelLeftRear(), robot::Chassis::kWheelPidIdxLeftRear);
    unique_chassis.registerWheelPid(CreatePidMotorWheelRightRear(), robot::Chassis::kWheelPidIdxRightRear);
    unique_chassis.registerWheelPid(CreatePidMotorWheelRightFront(), robot::Chassis::kWheelPidIdxRightFront);
    // 随动速度
    unique_chassis.registerFollowOmegaPid(CreatePidFollowOmega());
    // * - 功率限制
    unique_chassis.registerPwrLimiter(CreatePwrLimiter());

    unique_chassis.registerImu(CreateImu());
    // * 2. 只接收数据的组件指针
    // 云台和底盘通信
    unique_chassis.registerGimbalChassisComm(CreateGimbalChassisComm());
    // YAW 轴电机
    unique_chassis.registerYawMotor(CreateMotorYaw());
    // * 3. 接收、发送数据的组件指针
    // 超级电容
    unique_chassis.registerCap(CreateCap());
    // 轮组电机
    unique_chassis.registerWheelMotor(CreateMotorWheelLeftFront(), robot::Chassis::kWheelMotorIdxLeftFront);
    unique_chassis.registerWheelMotor(CreateMotorWheelLeftRear(), robot::Chassis::kWheelMotorIdxLeftRear);
    unique_chassis.registerWheelMotor(CreateMotorWheelRightRear(), robot::Chassis::kWheelMotorIdxRightRear);
    unique_chassis.registerWheelMotor(CreateMotorWheelRightFront(), robot::Chassis::kWheelMotorIdxRightFront);
    is_chassis_inited = true;
  }
  return &unique_chassis;
};

robot::Gimbal* CreateGimbal() { return &unique_gimbal; };
robot::Shooter* CreateShooter() { return &unique_shooter; };
robot::Feed* CreateFeed() { return &unique_feed; }

robot::Robot* CreateRobot()
{
  if (!is_robot_inited) {
    // main 组件指针注册
    // 主要模块状态机组件指针
    unique_robot.registerChassis(CreateChassis());
    unique_robot.registerShooter(CreateShooter());
    unique_robot.registerGimbal(CreateGimbal());
    unique_robot.registerFeed(CreateFeed());

    // 无通信功能的组件指针
    // unique_robot.registerBuzzer(CreateBuzzer());
    unique_robot.registerImu(CreateImu());

    // 只接收数据的组件指针
    unique_robot.registerRc(CreateRemoteControl());
    // 只发送数据的组件指针
    unique_robot.registerCap(CreateCap());
    unique_robot.registerMotorWheels(CreateMotorWheelLeftFront(), robot::Chassis::kWheelMotorIdxLeftFront);
    unique_robot.registerMotorWheels(CreateMotorWheelLeftRear(), robot::Chassis::kWheelMotorIdxLeftRear);
    unique_robot.registerMotorWheels(CreateMotorWheelRightRear(), robot::Chassis::kWheelMotorIdxRightRear);
    unique_robot.registerMotorWheels(CreateMotorWheelRightFront(), robot::Chassis::kWheelMotorIdxRightFront);
    // 收发数据的组件指针
    unique_robot.registerGimbalChassisComm(CreateGimbalChassisComm());
    unique_robot.registerReferee(CreateReferee());

    unique_robot.registerPerformancePkg(CreateRobotPerformancePackage());
    unique_robot.registerPowerHeatPkg(CreateRobotPowerHeatPackage());
    unique_robot.registerShooterPkg(CreateRobotShooterPackage());
    unique_robot.registerCompRobotsHpPkg(CreateCompRobotsHpPackage());
    unique_robot.registerRobotHurtPkg(CreateRobotHurtPackage());
    is_robot_inited = true;
  }
  return &unique_robot;
};

/* Private function definitions ----------------------------------------------*/
