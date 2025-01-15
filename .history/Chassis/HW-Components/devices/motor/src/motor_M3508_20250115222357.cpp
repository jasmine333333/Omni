/**
 *******************************************************************************
 * @file      :motor_M3508.cpp
 * @brief     : 大疆 M3508 电机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 请先查看 motor_base.hpp 中的注意事项
 *  2. 电机 ID 范围为 1~8，其中 1~4 为同一条发送报文，5~8 为同一条发送报文
 *  3. 可使用的输入类型为 InputType::kRaw、InputType::kTorq 和 InputType::kCurr
 *  4. 需结合 C620 电调使用
 *  5. 可额外获得的数据为电机温度 temp
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_M3508.hpp"

#include "assert.hpp"

// debug:
size_t len = 0;
namespace hello_world
{
namespace motor
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static const MotorBaseInfo kM3508MotorBaseInfo{
    .raw_input_lim = 16384,
    .torq_input_lim = 6.0f,
    .curr_input_lim = 20.0f,
    .torq_const = 0.0156f,
    .redu_rat = 3591.0f / 187,
    .angle_rat = 2 * PI / 8191,
    .vel_rat = 2 * PI / 60,
    .curr_rat = 20.0f / 16384,
    .torq_rat = kInvalidValue,
    .cross_0_value = 8191U,
    .raw_mapping_type = RawMappingType::kCurr,
};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

M3508::M3508(uint8_t id, const OptionalParams& opt)
    : Motor(opt.offline_tick_thres)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(1 <= id && id <= 8, "Error id: %d", id);
  HW_ASSERT(opt.dir == kDirFwd || opt.dir == kDirRev,
            "Error dir: %d", opt.dir);
  HW_ASSERT(opt.angle_range == AngleRange::k0To2Pi ||
                opt.angle_range == AngleRange::kNegInfToPosInf ||
                opt.angle_range == AngleRange::kNegPiToPosPi,
            "Error angle range: %d", opt.angle_range);
  HW_ASSERT(opt.input_type == InputType::kRaw ||
                opt.input_type == InputType::kTorq ||
                opt.input_type == InputType::kCurr,
            "Error input type: %d", opt.input_type);
  HW_ASSERT(opt.ex_redu_rat > 0,
            "Error external reduction ration: %f", opt.ex_redu_rat);
  HW_ASSERT(opt.max_raw_input_lim > 0,
            "Error max raw input limit: %f", opt.max_raw_input_lim);
  HW_ASSERT(opt.max_torq_input_lim > 0,
            "Error max torque input limit: %f", opt.max_torq_input_lim);
  HW_ASSERT(opt.max_curr_input_lim > 0,
            "Error max current input limit: %f", opt.max_curr_input_lim);
#pragma endregion

  motor_info_ = kM3508MotorBaseInfo;

  /* 根据 ID 设定报文 ID */
  motor_info_.tx_id = id <= 4 ? kTx1_4_ : kTx5_8_;
  motor_info_.rx_id = kRx0_ + id;
  motor_info_.tx_ids = {motor_info_.tx_id};
  motor_info_.rx_ids = {motor_info_.rx_id};
  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;

  /* 转子端力矩限制 */
  float rotor_torq_lim = motor_info_.torq_input_lim / motor_info_.redu_rat;
  if (opt.remove_build_in_reducer) {
    motor_info_.redu_rat = opt.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= opt.ex_redu_rat;
  }

  /* 计算输入限制 */
  float max_raw_input = motor_info_.raw_input_lim;
  if (opt.max_raw_input_lim != std::numeric_limits<float>::max()) {
    max_raw_input = std::min(max_raw_input, opt.max_raw_input_lim);
  }
  float max_torq_input = rotor_torq_lim * motor_info_.redu_rat;
  if (opt.max_torq_input_lim != std::numeric_limits<float>::max()) {
    max_torq_input = std::min(max_torq_input, opt.max_torq_input_lim);
  }
  float max_curr_input = motor_info_.curr_input_lim;
  if (opt.max_curr_input_lim != std::numeric_limits<float>::max()) {
    max_curr_input = std::min(max_curr_input, opt.max_curr_input_lim);
  }

  max_raw_input = std::min(max_raw_input, torq2Raw(max_torq_input));
  max_raw_input = std::min(max_raw_input, curr2Raw(max_curr_input));
  motor_info_.raw_input_lim = max_raw_input;
  motor_info_.torq_input_lim = raw2Torq(max_raw_input);
  motor_info_.curr_input_lim = raw2Curr(max_raw_input);
}

M3508& M3508::operator=(const M3508& other)
{
  if (this != &other) {
    Motor::operator=(other);

    temp_ = other.temp_;
  }

  return *this;
}

M3508::M3508(M3508&& other) : Motor(std::move(other))
{
  temp_ = other.temp_;
}

M3508& M3508::operator=(M3508&& other)
{
  if (this != &other) {
    Motor::operator=(std::move(other));

    temp_ = other.temp_;
  }

  return *this;
}

bool M3508::decode(size_t len, const uint8_t* data)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data, "data is nullptr");
#pragma endregion

  if (len != 8) {
    decode_fail_cnt_++;
    return false;
  }

  uint16_t raw_angle = static_cast<uint16_t>((data[0] << 8) | data[1]);

  /* 判断是否旋转了一圈 */
  float round = round_;
  float delta_angle = raw_angle - last_raw_angle_;
  if (fabsf(delta_angle) > motor_info_.cross_0_value * kCross0ValueThres) {
    delta_angle < 0 ? round++ : round--;

    /* 避免圈数溢出 */
    if (motor_info_.angle_range != AngleRange::kNegInfToPosInf) {
      round = fmodf(round, motor_info_.redu_rat);
    }
  }

  float actual_ang =
      static_cast<float>(motor_info_.dir) *
      (raw_angle * motor_info_.angle_rat + round * 2 * PI) /
      motor_info_.redu_rat;
  float raw = static_cast<float>(motor_info_.dir) *
              static_cast<int16_t>((data[4] << 8) | data[5]);

  /* 统一对电机状态赋值 */
  round_ = round;
  actual_angle_ = normAngle(actual_ang);
  angle_ = normAngle(actual_ang - motor_info_.angle_offset);
  vel_raw_ = static_cast<float>(motor_info_.dir) *
             static_cast<int16_t>((data[2] << 8) | data[3]) *
             motor_info_.vel_rat / motor_info_.redu_rat;
  vel_ = calcVel();
  torq_ = raw2Torq(raw);
  curr_ = raw2Curr(raw);
  temp_ = data[6];

  last_raw_angle_ = raw_angle;

  oc_.update();

  decode_success_cnt_++;
  is_update_ = true;
  if (update_cb_) {
    update_cb_();
  }
  return true;
}

bool M3508::encode(size_t& len, uint8_t* data)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data, "data is nullptr");
  HW_ASSERT(len, "len is nullptr");
#pragma endregion

  if (len != 8) {
    encode_fail_cnt_++;
    HW_ASSERT(len!=8,"wronng len %d",len);
    return false;
  }

  uint8_t index = (motor_info_.id - 1) % 4;

  int16_t input =
      hello_world::Bound(static_cast<float>(motor_info_.dir) * raw_input_,
                         motor_info_.raw_input_lim, -motor_info_.raw_input_lim);
  data[2 * index] = input >> 8;
  data[2 * index + 1] = input;

  encode_success_cnt_++;
  return true;
}

void M3508::init(uint8_t id, const OptionalParams& opt)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(1 <= id && id <= 8, "Error id: %d", id);
  HW_ASSERT(opt.dir == kDirFwd || opt.dir == kDirRev,
            "Error dir: %d", opt.dir);
  HW_ASSERT(opt.angle_range == AngleRange::k0To2Pi ||
                opt.angle_range == AngleRange::kNegInfToPosInf ||
                opt.angle_range == AngleRange::kNegPiToPosPi,
            "Error angle range: %d", opt.angle_range);
  HW_ASSERT(opt.input_type == InputType::kRaw ||
                opt.input_type == InputType::kTorq ||
                opt.input_type == InputType::kCurr,
            "Error input type: %d", opt.input_type);
  HW_ASSERT(opt.ex_redu_rat > 0,
            "Error external reduction ration: %f", opt.ex_redu_rat);
  HW_ASSERT(opt.max_raw_input_lim > 0,
            "Error max raw input limit: %f", opt.max_raw_input_lim);
  HW_ASSERT(opt.max_torq_input_lim > 0,
            "Error max torque input limit: %f", opt.max_torq_input_lim);
  HW_ASSERT(opt.max_curr_input_lim > 0,
            "Error max current input limit: %f", opt.max_curr_input_lim);
#pragma endregion

  motor_info_ = kM3508MotorBaseInfo;

  /* 根据 ID 设定报文 ID */
  motor_info_.tx_id = id <= 4 ? kTx1_4_ : kTx5_8_;
  motor_info_.rx_id = kRx0_ + id;
  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;

  /* 转子端力矩限制 */
  float rotor_torq_lim = motor_info_.torq_input_lim / motor_info_.redu_rat;
  if (opt.remove_build_in_reducer) {
    motor_info_.redu_rat = opt.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= opt.ex_redu_rat;
  }

  /* 计算输入限制 */
  float max_raw_input = motor_info_.raw_input_lim;
  if (opt.max_raw_input_lim != std::numeric_limits<float>::max()) {
    max_raw_input = std::min(max_raw_input, opt.max_raw_input_lim);
  }
  float max_torq_input = rotor_torq_lim * motor_info_.redu_rat;
  if (opt.max_torq_input_lim != std::numeric_limits<float>::max()) {
    max_torq_input = std::min(max_torq_input, opt.max_torq_input_lim);
  }
  float max_curr_input = motor_info_.curr_input_lim;
  if (opt.max_curr_input_lim != std::numeric_limits<float>::max()) {
    max_curr_input = std::min(max_curr_input, opt.max_curr_input_lim);
  }

  max_raw_input = std::min(max_raw_input, torq2Raw(max_torq_input));
  max_raw_input = std::min(max_raw_input, curr2Raw(max_curr_input));
  motor_info_.raw_input_lim = max_raw_input;
  motor_info_.torq_input_lim = raw2Torq(max_raw_input);
  motor_info_.curr_input_lim = raw2Curr(max_raw_input);

  angle_ = 0.0f;
  vel_ = 0.0f;
  vel_raw_ = 0.0f;
  torq_ = 0.0f;
  curr_ = 0.0f;
  round_ = 0.0f;
  actual_angle_ = 0.0f;

  is_update_ = false;
  update_cb_ = nullptr;
  oc_.init(opt.offline_tick_thres);

  raw_input_ = 0;
  torq_input_ = 0;
  curr_input_ = 0;

  td_ptr_ = nullptr;

  decode_success_cnt_ = 0;
  decode_fail_cnt_ = 0;
  encode_success_cnt_ = 0;
  encode_fail_cnt_ = 0;
  transmit_success_cnt_ = 0;

  temp_ = 0;
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world
