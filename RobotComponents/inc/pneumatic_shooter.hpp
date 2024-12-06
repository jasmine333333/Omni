/** 
 *******************************************************************************
 * @file      : pneumatic_shooter.hpp
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
#ifndef ROBOT_COMPONENTS_PNEUMATIC_SHOOTER_HPP_
#define ROBOT_COMPONENTS_PNEUMATIC_SHOOTER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "offline_checker.hpp"
#include "receiver.hpp"
#include "transmitter.hpp"

namespace robot
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class PneumaticShooter : public hello_world::comm::Receiver, public hello_world::comm::Transmitter
{
 public:
  typedef hello_world::OfflineChecker OfflineChecker;

  enum class BulletState : uint8_t {
    Err = 0u,    ///< 错误，非正常数据
    Note = 1u,   ///< 注意，光电门检测到供弹，但发射机构未准备就绪
    Ready = 2u,  ///< 准备就绪
  };

  enum class ShooterState : uint8_t {
    Idle = 0u,            ///< 空闲状态
    WaitingValve23 = 1u,  ///< 等待
    WaitingValve25 = 2u,  ///< 等待
    Finished = 3u,        ///< 发弹动作完成
    Err = 0xFF,           ///< 错误，非正常数据
  };

  PneumaticShooter(uint32_t tx_id = 0x100, uint32_t rx_id = 0x101, uint32_t offline_threshold = 10) : oc_(offline_threshold)
  {
    setTxId(tx_id);
    setRxId(rx_id);
  };

  ~PneumaticShooter() = default;

  // can cfg
  void setTxId(uint32_t tx_id) { tx_id_ = tx_id; };
  void setRxId(uint32_t rx_id) { rx_id_ = rx_id; };

  bool isOffline() { return oc_.isOffline(); };
  void setOfflineThreshold(uint32_t offline_threshold) { oc_.set_offline_tick_thres(offline_threshold); };

  // tx data
  void setShootFlag(void) { shoot_cnt_++; };
  void clearShootFlag(void) { last_shoot_cnt_ = shoot_cnt_; };
  bool getShootFlag(bool auto_clear = true)
  {
    bool res = shoot_cnt_ != last_shoot_cnt_;
    if (auto_clear) {
      clearShootFlag();
    }
    return res;
  };
  bool getShootFlag(void) const { return shoot_cnt_ != last_shoot_cnt_; };

  void setBulletSpeed(float speed) { bullet_speed_ = speed; };
  float getBulletSpeed(void) const { return bullet_speed_; };

  // rx data
  /** 
   * @brief       气动发射机构是否准备就绪
   * 
   * 若发射机构处于准备状态，则可以发射弹丸，否则需要等待发射机构准备就绪
   * @note 发射机构离线或者正在发射时，都不处于准备状态
   * @retval      true: 准备就绪，false: 未准备就绪
   */
  bool isShootReady(void) const { return is_shoot_ready_; };
  /** 
   * @brief       弹丸是否清除完毕
   * 
   * 在气动控制板异常下电时，弹丸可能残留在两层密封圈中，此时无法被光电门检测到，需要清除完毕后才能按照正常流程发射
   * @retval      true: 弹丸清除完毕，false: 弹丸未清除完毕
   */
  bool isBulletCleared(void) const { return is_bullet_cleared_; };
  /** 
   * @brief       气压是否到达目标值
   * 
   * 若气压到达目标值，则可以发射弹丸，否则需要等待气压到达目标值
   * @retval      true: 到达目标值，false: 未到达目标值
   */
  bool isAirPresReady(void) const { return is_air_pres_ready_; };
  /** 
   * @brief       弹丸是否发射完毕
   * 
   * 若发射机构发射完毕，则弹丸进入目标位置，否则需要等待发射完毕
   * @retval      true: 弹丸发射完毕，false: 弹丸未发射完毕
   */
  bool isAirBottleReady(void) const { return is_air_bottle_ready_; };

  /**
   * @brief       获取弹丸状态
   * @retval      弹丸状态
   */
  BulletState getBulletState(void) const { return bullet_state_; };
  /**
   * @brief       获取发射机构状态
   * @retval      发射机构状态
   */
  ShooterState getShooterState(void) const { return shooter_state_; };
  /**
   * @brief       获取比例阀控制气压
   * @retval      比例阀控制气压，单位：MPa
   */
  float getPropValveRef(void) const { return prop_valve_ref_; };

  /**
   * @brief       获取接收器ID
   * @param        None
   * @retval       接收器ID
   * @note        None
   */
  virtual uint32_t rxId(void) const override { return rx_id_; };

  /**
   * @brief       解码
   * @param        len: 数据长度
   * @param        data: 数据指针
   * @retval       解码成功返回true，否则返回false
   * @note        此函数会将 update_flag 置为 true
   */
  virtual bool decode(size_t len, const uint8_t* data) override;

  /**
   * @brief       是否有更新数据
   * @retval       有更新数据返回true，否则返回false
   * @note        判断频率比 isOffline 更快 
   */
  virtual bool isUpdate(void) const { return is_update_; };

  /**
   * @brief       清除更新标志
   * @retval       None
   * @note        None
   */
  virtual void clearUpdateFlag(void) { is_update_ = false; };

  /**
   * @brief       注册更新回调函数
   * @param        cb: 回调函数指针，在decode函数解码成功后被调用，
   * 不使用时传入nullptr
   * @retval       None
   * @note        None
   */
  virtual void registerUpdateCallback(pUpdateCallback cb) { update_cb_ = cb; };

  /**
   * @brief       获取发送器ID
   * @param        None
   * @retval       发送器ID
   * @note        None
   */
  virtual uint32_t txId(void) const override { return tx_id_; };

  /**
   * @brief       编码
   * @param        len: 需传入缓冲区长度，传出编码后的数据长度
   * @param        data: 缓冲区指针
   * @retval       编码成功返回true，否则返回false
   * @note        编码成功后len需修改为编码后的数据长度
   */
  virtual bool encode(size_t& len, uint8_t* data) override;

  /**
   * @brief       发送成功回调
   * 
   * 成功发送后调用，用于统计发送成功的次数
   * @retval      None
   */
  void txSuccessCb(void) override { transmit_success_cnt_++; }

 private:
  // 解码相关
  uint32_t rx_id_ = 0x101;                   ///< 接收的CAN消息ID
  bool is_update_ = false;                   ///< 是否有更新数据
  pUpdateCallback update_cb_ = nullptr;      ///< 更新回调函数
  OfflineChecker oc_ = OfflineChecker(200);  ///< 离线检测器

  // rx data
  bool is_shoot_ready_ = false;                  ///< 发射机构是否处于准备状态，发射机构离线或者正在发射时，都不处于准备状态
  bool is_bullet_cleared_ = false;               ///< 是否执行弹丸清除动作
  bool is_air_pres_ready_ = false;               ///< 气室气压是否到达目标值，卡弹也有可能是气压不够导致的
  bool is_air_bottle_ready_ = false;             ///< 气瓶是否处于欠压状态，若欠压则气室气压将无法到达设定值
  BulletState bullet_state_ = BulletState::Err;  ///< 弹丸状态
  ShooterState shooter_state_ = ShooterState::Err;  ///< 发射机构状态
  float prop_valve_ref_ = 0;                        ///< 比例阀控制气压，单位：MPa

  // 编码相关
  uint32_t tx_id_ = 0x100;             ///< 发送的CAN消息ID
  uint32_t transmit_success_cnt_ = 0;  ///< 发送成功次数

  // tx data
  uint32_t shoot_cnt_ = 0;       ///< 发射指令，与上一次发射指令进行比较，判断是否需要发射
  uint32_t last_shoot_cnt_ = 0;  ///< 上一次发射指令
  float bullet_speed_ = 0;       ///< 弹丸速度，单位： m/s
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/  // namespace robot
}  // namespace robot

#endif /* ROBOT_COMPONENTS_PNEUMATIC_SHOOTER_HPP_ */
