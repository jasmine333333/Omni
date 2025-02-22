/** 
 *******************************************************************************
 * @file      :usr_imu.hpp
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
#ifndef ROBOT_COMPONENTS_USR_IMU_HPP_
#define ROBOT_COMPONENTS_USR_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "imu.hpp"
#include "mahony.hpp"
#include "spi.h"
/* Exported macro ------------------------------------------------------------*/

namespace robot
{
/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
union Imu3AxisData {
  struct {
    float x;
    float y;
    float z;
  };
  float data[3];
};

class Imu : public hello_world::MemMgr
{
 public:
  typedef hello_world::imu::BMI088 BMI088;
  typedef hello_world::imu::BMI088HWConfig BMI088HWConfig;
  typedef hello_world::imu::BMI088Config BMI088Config;
  typedef hello_world::ahrs::Mahony Mahony;


  static constexpr BMI088HWConfig kBmi088DefaultHWConfig = {
      .hspi = &hspi1,
      .acc_cs_port = GPIOA,
      .acc_cs_pin = GPIO_PIN_4,
      .gyro_cs_port = GPIOB,
      .gyro_cs_pin = GPIO_PIN_0,
  };
  static constexpr float kImuRotMatFlatten[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static constexpr BMI088Config kBmi088DefaultConfig = {
      .acc_range = hello_world::imu::kBMI088AccRange3G,
      .acc_odr = hello_world::imu::kBMI088AccOdr1600,
      .acc_osr = hello_world::imu::kBMI088AccOsr4,
      .gyro_range = hello_world::imu::kBMI088GyroRange1000Dps,
      .gyro_odr_fbw = hello_world::imu::kBMI088GyroOdrFbw1000_116,
  };

  struct Config {
    size_t offset_max_count = 1000;
    float acc_threshold = 10.0f;
    float gyro_threshold = 0.1f;
    float samp_freq = 1000.0f;
    float kp = 1.0f;
    float ki = 0.0f;
    const BMI088HWConfig *bmi088_hw_config_ptr = &kBmi088DefaultHWConfig;
    const float *rot_mat_ptr = kImuRotMatFlatten;
    const BMI088Config *bmi088_config_ptr = &kBmi088DefaultConfig;
  };

  enum ImuStatus {
    kNotInitHardware,
    kWaitCalcOffset,
    kCalcingOffset,
    kNormWorking,
    kInitHardwareFailedErr,
    kPointerUninitErr,
    kUnkonwnErr,
  };

  explicit Imu(const Config &cfg )
  {
    offset_max_count_ = cfg.offset_max_count;
    acc_threshold_ = fabs(cfg.acc_threshold);
    gyro_threshold_ = fabs(cfg.gyro_threshold);
    bmi088_ptr_ = new BMI088(*cfg.bmi088_hw_config_ptr, cfg.rot_mat_ptr, *cfg.bmi088_config_ptr);
    ahrs_ptr_ = new Mahony(cfg.samp_freq, cfg.kp, cfg.ki);
  };
  ~Imu()
  {
    if (bmi088_ptr_) {
      delete bmi088_ptr_;
      bmi088_ptr_ = nullptr;
    }
    if (ahrs_ptr_) {
      delete ahrs_ptr_;
      ahrs_ptr_ = nullptr;
    }
  };

  bool initHardware();

  bool update();

  void resetOffset()
  {
    status_ = ImuStatus::kWaitCalcOffset;
    memset(gyro_offset_.data, 0, sizeof(gyro_offset_.data));
    offset_count_ = 0;
  };

  bool isNormWorking() { return status_ == ImuStatus::kNormWorking; };

  float getAngRoll() { return ang_.x; };
  float getAngPitch() { return ang_.y; };
  float getAngYaw() { return ang_.z; };

  float getGyroRoll() { return gyro_.x; };
  float getGyroPitch() { return gyro_.y; };
  float getGyroYaw() { return gyro_.z; };

  float getSlopeAng() {return slope_ang_; }
  float getGx() {return g_x_; }
  float getGy() {return g_y_; }

 private:
  void getRawData();

  void calcOffset();

  void updateAccGyro();

  bool updateMahony();

  void updateSlopeAng();  //计算倾斜角

  BMI088 *bmi088_ptr_ = nullptr;  ///< 硬件接口
  Mahony *ahrs_ptr_ = nullptr;    ///< 姿态算法

  // 抽象数据，与硬件无关
  Imu3AxisData raw_acc_ = {0};   ///< 原始加速度，单位：m/s^2
  Imu3AxisData raw_gyro_ = {0};  ///< 原始角速度，单位：rad/s
  float temp_ = 0.0f;            ///< 温度，单位：℃

  float acc_threshold_ = 10;        ///< 加速度阈值，用于过滤短时撞击，单位：m/s^2
  float gyro_threshold_ = 0.1;      ///< 角速度阈值，用于去除异常数据，计算零飘时使用，单位：rad/s
  Imu3AxisData gyro_offset_ = {0};  ///< 角速度零飘，单位：rad/s

  Imu3AxisData acc_ = {0};   ///< 加速度，供 mahony 算法使用，单位：m/s^2
  Imu3AxisData gyro_ = {0};  ///< 角速度，供 mahony 算法和其他外部程序使用，单位：rad/s

  Imu3AxisData ang_ = {0};  ///< 姿态角度，由 mahony 算法计算，供外部程序使用，单位：rad

  float slope_ang_ = 0;     /// IMU XY面与地面夹角，用于判断上坡
  float g_x_ = 0;           /// 重力在X轴上的分量,[0, 1]
  float g_y_ = 0;           /// 重力在Y轴上的分量,[0, 1]

  ImuStatus status_ = ImuStatus::kNotInitHardware;  ///< IMU 工作状态
  size_t offset_count_ = 0;                         ///< 计数器，用于计算角速度零飘
  size_t offset_max_count_ = 100;                   ///< 最大计数器值，用于计算角速度零飘
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* ROBOT_COMPONENTS_USR_IMU_HPP_ */
