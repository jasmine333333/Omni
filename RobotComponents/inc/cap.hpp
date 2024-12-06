/** 
 *******************************************************************************
 * @file      : cap.hpp
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
#ifndef ROBOT_COMPONENTS_CAP_HPP_
#define ROBOT_COMPONENTS_CAP_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "offline_checker.hpp"
#include "receiver.hpp"
#include "transmitter.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot
{
/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Cap : public hello_world::comm::Receiver, public hello_world::comm::Transmitter
{
 public:
  typedef hello_world::OfflineChecker OfflineChecker;

  enum PwrSrc {
    kPwrSrcSuperCap,  ///< 超级电容
    kPwrSrcBattery,   ///< 锂电池
  };
  const float kMaxChargedVoltage = 26.0f;  ///< 超级电容的最大充电电压，实际可能可以比该值更高，单位：V
  const float kMinValidVoltage = 18.0f;    ///< 超级电容的最小有效放电电压，小于此值之后不允许开启超电，单位：V
  const float kAutoDisablePercent = 0.2f;  ///< 超级电容的自动关闭百分比，当剩余能量低于此值时，自动关闭超电，单位：%
  const float kMinEnablePercent = 0.5f;    ///< 超级电容的最小开启百分比，当剩余能量高于此值时，才能开启超电，单位：%

  Cap(uint32_t tx_id = 0x111, uint32_t rx_id = 0x112, uint32_t offline_threshold = 10) : oc_(offline_threshold)
  {
    setTxId(tx_id);
    setRxId(rx_id);
  };

  ~Cap() = default;

  bool enable();

  bool disable()
  {
    enable_flag_ = false;
    return true;
  };

  bool isUsingSuperCap() const { return pwr_src_ == kPwrSrcSuperCap && enable_flag_ && (voltage_ >= kMinValidVoltage); };

  float getRemainingPowerPercent() const
  {
    const float kMinVal = kMinValidVoltage * kMinValidVoltage;
    const float kMaxVal = kMaxChargedVoltage * kMaxChargedVoltage;
    return (voltage_ * voltage_ - kMinVal) / (kMaxVal - kMinVal);
  };

  // 数据获取接口函数
  // rx data
  float getVoltage() const { return voltage_; };
  PwrSrc getPwrSrc() const { return pwr_src_; };
  // tx data
  bool getEnableFlag() const { return enable_flag_; };
  uint16_t getRfrChasPwrBuffer() const { return rfr_chas_pwr_buffer_; };
  uint16_t getRfrChasPwrLimit() const { return rfr_chas_pwr_limit_; };

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

  // 数据设置接口函数
  void reset()
  {
    voltage_ = 0;
    pwr_src_ = kPwrSrcBattery;
  };
  // tx data
  bool setEnableFlag(bool enable_flag)
  {
    if (enable_flag) {
      return enable();
    } else {
      return disable();
    }
  };
  void setRfrChasPwrBuffer(uint16_t rfr_chas_pwr_buffer) { rfr_chas_pwr_buffer_ = rfr_chas_pwr_buffer; };
  void setRfrChasPwrLimit(uint16_t rfr_chas_pwr_limit) { rfr_chas_pwr_limit_ = rfr_chas_pwr_limit; };
  // can cfg
  void setTxId(uint32_t tx_id) { tx_id_ = tx_id; };
  void setRxId(uint32_t rx_id) { rx_id_ = rx_id; };

  bool isOffline() { return oc_.isOffline(); };
  void setOfflineThreshold(uint32_t offline_threshold) { oc_.set_offline_tick_thres(offline_threshold); };

 private:
  // 解码相关
  uint32_t rx_id_ = 0x112;                   ///< 接收的CAN消息ID
  bool is_update_ = false;                   ///< 是否有更新数据
  pUpdateCallback update_cb_ = nullptr;      ///< 更新回调函数
  OfflineChecker oc_ = OfflineChecker(200);  ///< 离线检测器

  // rx data
  float voltage_ = 0.0;                   ///< 超级电容的电压，单位：V
  PwrSrc pwr_src_ = kPwrSrcBattery;       ///< 能量来源，超级电容或锂电池
  float remaining_power_percent_ = 0.0f;  ///< 剩余能量百分比，0~1，0表示无能量，1表示满能量

  // 编码相关
  uint32_t tx_id_ = 0x111;             ///< 发送的CAN消息ID
  uint32_t transmit_success_cnt_ = 0;  ///< 发送成功次数

  // tx data
  bool enable_flag_ = 0;  ///< 控制超级电容是否开启，开启后，若超电返回信息 pwr_scr_ 为 kPwrSrcSuperCap 表明现在是从超电获取能量
  bool last_enable_flag_ = 0;         ///< 上一次的 enable_flag_ 状态
  uint16_t rfr_chas_pwr_buffer_ = 0;  ///< 裁判系统的缓冲能量，单位：J
  uint16_t rfr_chas_pwr_limit_ = 0;   ///< 裁判系统的底盘功率限制，单位：J
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot
#endif /* ROBOT_COMPONENTS_CAP_HPP_ */
