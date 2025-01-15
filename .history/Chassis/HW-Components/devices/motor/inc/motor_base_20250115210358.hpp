/**
 *******************************************************************************
 * @file      : motor_base.hpp
 * @brief     : 电机基类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.0.1      2023-12-30      Caikunzhen      1. 修复电流输入转力矩问题
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. OptionalParams 结构体中具有三个输入限制，彼此存在转换关系，电机实际的约束会被设
 *  置为等效的最小值（例如，力矩与电流均被转换为报文输入，然后取最小值作为约束）。同时只
 *  有当设置的约束值小于电机原始约束值时才会起到作用
 *  2. 注意由于电机计圈功能依靠前后两次电机的反馈，因此对于有高速转动电机的需求，要求其
 *  反馈频率不能过低，最低要求为在两次反馈间隔中，电机转子端不能转动超过半圈
 *  3. 电机转向关系标定：先在电机配置中选择电机转动正方向与自定转动正方向相同，烧录程序，
 *  用手转动电机输出端，在 Debug 中查看数据变化是否与期望的相同，若不同则电机转动正方向
 *  与自定转动正方向相反，以此配置转向关系
 *  4. 电机减速比配置：组件中电机默认的的减速比为电机自带减速器的减速比。
 *    1）当电机实际使用时在自带减速器外还额外增加减速器时，应当在电机初始化时将初始化结
 *  构体中 ex_redu_rat 属性设置为外加减速器的减速比，同时将 remove_build_in_reducer
 *  属性设置为 false
 *    2）当电机实际使用时拆除了电机自带的减速器，又外加了减速器时，应当在电机初始化时将
 *  初始化结构体中 ex_redu_rat 属性设置为外加减速器的减速比，同时将
 *  remove_build_in_reducer 属性设置为 true
 *  5. 电机零点标定（在电机转向关系标定与电机减速比配置完成后进行）：
 *    1）通过机械限位标定：通过机械限位标定可采取在程序一开始使先使对电机进行速度闭环，
 *  使其往固定方向转动，当读取到反馈的速度的值小于设定转速一定程度时便可认为到达机械限位，
 *  此时可调用 setAngleValue 方法设定当前的角度，值为到达机械限位时应该到达的角度值。
 *  采用此方法的往往是没有输出端绝对编码的电机，因此对于这类电机在每次程序重启后都需要进
 *  行标定后才可正常使用
 *    2）通过绝对位置标定 通过绝对位置标定可采取在电机属性初始化时将 angle_offset 属性
 *  设置为 0，然后烧录程序，将电机输出端转动到实际定义的 0 位后，通过 Debug 读取此时对
 *  应的角度值，然后将 angle_offset 属性设置为读取到的角度值，便可完成标定
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_BASE_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <limits>

#include "allocator.hpp"
#include "base.hpp"
#include "offline_checker.hpp"
#include "receiver.hpp"
#include "system.hpp"
#include "td.hpp"
#include "transmitter.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

enum class Status : uint8_t {
  kOk,                  ///* 正常
  kInputValueOverflow,  ///* 输入溢出
  kInputTypeError,      ///* 输入类型错误
};

enum class InputType : uint8_t {
  kRaw,   ///* 报文原始输入
  kTorq,  ///* 输出端力矩输入
  kCurr,  ///* 转子电流输入
  kCmd,   ///* 电机指令输入
};

enum class RawMappingType : uint8_t {
  kTorq,  ///* 原始报文对应力矩
  kCurr,  ///* 原始报文对应电流
};

enum class AngleRange : uint8_t {
  kNegPiToPosPi,    ///* [-π, π)
  k0To2Pi,          ///* [0, 2π)
  kNegInfToPosInf,  ///* (-∞, +∞)
};

enum Dir : int8_t {  ///* 电机转动正方向与规定正方向的关系
  kDirFwd = 1,       ///* 同向
  kDirRev = -1,      ///* 反向
};
/* Exported constants --------------------------------------------------------*/

static const float kInvalidValue = 0;
/* Exported types ------------------------------------------------------------*/

struct MotorBaseInfo : public MemMgr {
  /** 电机输入限制 */
  float raw_input_lim = 0;   ///* 电调报文输入限制值
  float torq_input_lim = 0;  ///* 电机输出端限制力矩值，单位：N·m
  float curr_input_lim = 0;  ///* 电机电流输入限制值，单位：A

  float torq_const = 0;        ///* 力矩常数，单位：N·m/A
  float redu_rat = 0;          ///* 减速比
  float angle_rat = 0;         ///* 角度分辨率
  float vel_rat = 0;           ///* 角速度分辨率
  float curr_rat = 0;          ///* 电流分辨率
  float torq_rat = 0;          ///* 力矩分辨率
  uint16_t cross_0_value = 0;  ///* 编码器过零值
  RawMappingType raw_mapping_type = RawMappingType::kCurr;
};

struct MotorInfo : public MotorBaseInfo {
  Dir dir = kDirFwd;
  uint8_t id = 0;
  AngleRange angle_range = AngleRange::kNegInfToPosInf;
  InputType input_type = InputType::kRaw;
  uint32_t rx_id = 0;               ///* 电机发回的报文的当前ID
  uint32_t tx_id = 0;               ///* 发给电机的报文的当前ID
  comm::Receiver::RxIds rx_ids;     ///* 电机接收的报文的所有ID
  comm::Transmitter::TxIds tx_ids;  ///* 发给电机的报文的所有ID
  float angle_offset = 0;           ///* 电机输出端实际角度与规定角度的差值

  MotorInfo& operator=(const MotorBaseInfo& motor_base_info)
  {
    MotorBaseInfo* base = this;
    *base = motor_base_info;
    return *this;
  }
};

/** 电机初始化可选参数 */
struct OptionalParams : public MemMgr {
  InputType input_type = InputType::kRaw;
  AngleRange angle_range = AngleRange::kNegInfToPosInf;
  Dir dir = kDirFwd;
  /** 是否移除电机自带的减速器 */
  bool remove_build_in_reducer = false;
  /** 电机输出端实际角度与规定角度的差值 */
  float angle_offset = 0;
  /** 电机外置减速器的减速比（额外） */
  float ex_redu_rat = 1;
  /** 报文输入限制 */
  float max_raw_input_lim = std::numeric_limits<float>::max();
  /** 力矩输入限制 */
  float max_torq_input_lim = std::numeric_limits<float>::max();
  /** 电流输入限制 */
  float max_curr_input_lim = std::numeric_limits<float>::max();
  /** 掉线检测阈值，单位：ms */
  uint32_t offline_tick_thres = 100;
};

class Motor : public comm::Receiver, public comm::Transmitter
{
 public:
  Motor(void) = default;
  /**
   * @brief       构造函数
   * @param        offline_tick_thres: 掉线检测阈值，单位：ms
   * @retval       None
   * @note        None
   */
  Motor(uint32_t offline_tick_thres)
      : Receiver(), Transmitter(), oc_(offline_tick_thres), td_ptr_(nullptr) {}
  Motor(const Motor&) = default;
  Motor& operator=(const Motor& other);
  Motor(Motor&& other);
  Motor& operator=(Motor&& other);

  virtual ~Motor(void) = default;

  /* 重载方法 */

  virtual uint32_t rxId(void) const override { return motor_info_.rx_id; }

  virtual const RxIds &rxIds(void) const override { return motor_info_.rx_ids; }

  virtual bool isUpdate(void) const override { return is_update_; }

  virtual void clearUpdateFlag(void) override { is_update_ = false; }

  virtual void registerUpdateCallback(pUpdateCallback cb) override
  {
    update_cb_ = cb;
  }

  virtual uint32_t txId(void) const override { return motor_info_.tx_id; }

  virtual const TxIds &txIds(void) const override { return motor_info_.tx_ids; }

  virtual void txSuccessCb(void) override { transmit_success_cnt_++; }

  /* 功能性方法 */

  /**
   * @brief       注册角度微分跟踪器，用于计算角速度
   * @param        td_ptr: 角度微分跟踪器，要求 period 等于 2 * PI，dim 等于 1，请
   *               确保使用过程中不会释放该指针，不需要时可以设置为 nullptr
   * @retval       None
   * @note        None
   */
  void registerTd(filter::Td* td_ptr);

  void cancelTd(void) { td_ptr_ = nullptr; }

  /**
   * @brief       将原始报文内容转换为输出端力矩
   * @param        raw: 原始报文数值
   * @retval       原始报文对应的输出端力矩值，单位：N·m
   * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过 motor_info
   *              方法获取电机信息结构体，查看其中的 raw_mapping_type 变量
   */
  virtual float raw2Torq(float raw) const;

  /**
   * @brief       将输出端力矩转换为原始报文内容
   * @param        torq: 输出端力矩值，单位：N·m
   * @retval       输出端力矩值对应的原始报文
   * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过 motor_info
   *              方法获取电机信息结构体，查看其中的 raw_mapping_type 变量
   */
  virtual float torq2Raw(float torq) const;

  /**
   * @brief       将原始报文内容转换为转子电流
   * @param        raw: 原始报文数值
   * @retval       原始报文对应的转子电流值，单位：A
   * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过 motor_info
   *              方法获取电机信息结构体，查看其中的 raw_mapping_type 变量
   */
  virtual float raw2Curr(float raw) const;

  /**
   * @brief       将转子电流转换为原始报文内容
   * @param        curr: 转子电流值，单位：A
   * @retval       转子电流值对应的原始报文
   * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过 motor_info
   *              方法获取电机信息结构体，查看其中的 raw_mapping_type 变量
   */
  virtual float curr2Raw(float curr) const;

  /**
   * @brief       将输出端力矩转换为转子电流
   * @param        torq: 输出端力矩值，单位：N·m
   * @retval       输出端力矩值对应的转子电流，单位：A
   * @note        None
   */
  virtual float torq2Curr(float torq) const;

  /**
   * @brief       将转子电流转换为输出端力矩
   * @param        curr: 转子电流值，单位：A
   * @retval       转子电流值对应的输出端力矩，单位：N·m
   * @note        None
   */
  virtual float curr2Torq(float curr) const;

  /**
   * @brief       设定发给电调的期望值
   * @param        input: 发给电调的期望值
   * @retval       设定状态，可能的返回值有：
   *   @arg        Status::kOk: 设定成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   *   @arg        Status::kInputValueOverflow: 设定值超出范围
   * @note        1. 期望值的物理意义与电机当前的输入类型有关，可使用 get_input_type
   *              方法查看
   *              2. 设定的期望值会自动被限制到允许的范围内，当前实际的设定值可以通过
   *              getInput 方法查看
   */
  virtual Status setInput(float input);

  /**
   * @brief       获取发给电调的期望值
   * @retval       发给电调的期望值
   * @note        期望值的物理意义与电机当前的输入类型有关，可使用 get_input_type 方法
   *              查看
   */
  float getInput(void) const;

  /**
   * @brief       将当前电机输出端所处位置设置为指定的角度
   * @param        angle: 指定角度，单位：rad
   * @retval       None
   * @note        None
   */
  void setAngleValue(float angle)
  {
    motor_info_.angle_offset = actual_angle_ - angle;
    angle_ = angle;
  }

  bool isOffline(void) { return oc_.isOffline(); }

  /* 数据修改与获取 */

  /**
   * @brief       设置点击的输入类型
   * @param        input_type: 期望输入类型，可选值为：
   *   @arg        InputType::kRaw: 原始报文输入
   *   @arg        InputType::kTorq: 输出端力矩输入
   *   @arg        InputType::kCurr: 转子端电流输入
   * @retval       设置状态，可能的返回值有：
   *   @arg        Status::kOk: 设置成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   * @note        None
   */
  virtual Status set_input_type(InputType input_type);

  InputType get_input_type(void) const { return motor_info_.input_type; }

  const MotorInfo& motor_info(void) const { return motor_info_; }

  Dir dir(void) const { return motor_info_.dir; }

  uint8_t id(void) const { return motor_info_.id; }

  AngleRange angle_range(void) const { return motor_info_.angle_range; }

  float angle(void) const { return angle_; }

  float vel(void) const { return vel_; }

  float vel_raw(void) const { return vel_raw_; }

  float torq(void) const { return torq_; }

  float curr(void) const { return curr_; }

 protected:
  /* 功能性方法 */

  /**
   * @brief       将角度转换到指定范围
   * @param        angle: 待转换的角度，单位：rad
   * @retval       归一化后的角度，单位：rad
   * @note        None
   */
  float normAngle(float angle);

  /**
   * @brief       计算角速度
   * @retval       计算后的角速度，单位：rad/s
   * @note        当注册了微分跟踪器后，且微分跟踪器未发散，将使用微分跟踪器计算角速度，
   *              否则返回电机反馈的角速度，同时会重置微分跟踪器。 请在更新完角度与角
   *              速度后调用该方法
   */
  float calcVel(void);

  /* 电机状态 */

  float angle_ = 0;  ///* 电机输出端角度，单位：rad
  /** 电机输出端角速度（当注册微分跟踪器则由角度微分得到，若未注册则为电机反馈的角速度），
   *  单位：rad/s */
  float vel_ = 0;
  float vel_raw_ = 0;         ///* 电机输出端角速度原始值，单位：rad/s
  float torq_ = 0;            ///* 电机输出端实际力矩，单位：N·m
  float curr_ = 0;            ///* 电机转子实际电流，单位：A
  float round_ = 0;           ///* 电机转子端累计圈数
  float last_raw_angle_ = 0;  ///* 电机上次角度原始值
  /** 电机输出端实际输出角度（未扣除角度偏差），单位：rad */
  float actual_angle_ = 0;
  MotorInfo motor_info_;  ///* 电机状态

  /* 更新相关 */

  bool is_update_ = false;                   ///* 是否有更新数据
  pUpdateCallback update_cb_ = nullptr;      ///* 更新回调函数
  public:
  OfflineChecker oc_ = OfflineChecker(100);  ///* 掉线检查
  protected:
  /* 电机输入 */

  float raw_input_ = 0;   ///* 报文原始输入
  float torq_input_ = 0;  ///* 电机输出端期望力矩
  float curr_input_ = 0;  ///* 电机转子期望电流

  filter::Td* td_ptr_ = nullptr;  ///* 电机角度微分跟踪器

  /* 发送接收状态统计 */

  uint32_t decode_success_cnt_ = 0;    ///* 解码成功次数
  uint32_t decode_fail_cnt_ = 0;       ///* 解码失败次数
  uint32_t encode_success_cnt_ = 0;    ///* 编码成功次数
  uint32_t encode_fail_cnt_ = 0;       ///* 编码失败次数
  uint32_t transmit_success_cnt_ = 0;  ///* 发送成功次数

  static constexpr float kCross0ValueThres = 0.5f;  ///* 过零阈值
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_BASE_HPP_ */
