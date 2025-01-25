/**
 *******************************************************************************
 * @file      :super_cap.hpp
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
#ifndef HW_COMPONENTS_DEVICES_CAP_SUPER_CAP_HPP_
#define HW_COMPONENTS_DEVICES_CAP_SUPER_CAP_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "offline_checker.hpp"
#include "receiver.hpp"
#include "system.hpp"
#include "transmitter.hpp"

namespace hello_world
{
namespace cap
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START

class SuperCap : public comm::Receiver, public comm::Transmitter
{
 public:
  typedef hello_world::OfflineChecker OfflineChecker;

  enum class PwrSrc {
    kSuperCap = 0u,  ///< 超级电容
    kBattary = 1u,   ///< 电池
  };

  enum class Version : uint8_t {
    kVer2021,  ///< 2021 版本
    kVer2024,  ///< 2024 国赛数控超电
  };

  struct Config : public MemMgr {
    float max_charge_volt = 26.0f;     ///< 最大充电电压，实际可能可以比该值更高，单位：V
    float min_valid_volt = 16.0f;      ///< 最小有效放电电压，小于此值之后不允许开启超电，单位：V
    float auto_disable_power = 20.0f;  ///< 自动关闭能量阈值，单位：%，当剩余能量低于此值时，自动关闭超电，[0, 100)
    float min_enable_power = 40.0f;    ///< 最小开启能量阈值，单位：%，当剩余能量高于此值时，才能开启超电，[0, 100)
    float pwr_filter_beta = 0.5f;      ///< 剩余能量的低通滤波系数，[0, 1]，为 0 时即不滤波
  };

  SuperCap(Version ver = Version::kVer2021) : version_(ver)
  {
    setConfig(Config());
  };
  SuperCap(const SuperCap &) = default;
  SuperCap &operator=(const SuperCap &other) = default;
  SuperCap(SuperCap &&other) = default;
  SuperCap &operator=(SuperCap &&other) = default;
  virtual ~SuperCap() = default;

  SuperCap(
      uint32_t offline_tick_thres, const Config &cfg,
      Version ver = Version::kVer2021)
      : oc_(offline_tick_thres), version_(ver) { setConfig(cfg); }

  /* 重载方法 */
  virtual uint32_t rxId(void) const override { return rx_id_; }

  virtual const RxIds &rxIds(void) const override { return rx_ids_; }

  virtual bool decode(size_t len, const uint8_t *data) override;

  virtual bool isUpdate(void) const override { return is_updated_; }

  virtual void clearUpdateFlag(void) override { is_updated_ = false; }

  virtual void registerUpdateCallback(pUpdateCallback cb) override
  {
    update_cb_ = cb;
  }

  virtual uint32_t txId(void) const override { return tx_id_; }

  virtual const TxIds &txIds(void) const override { return tx_ids_; }

  virtual bool encode(size_t &len, uint8_t *data) override;

  virtual void txSuccessCb(void) override { tx_success_cnt_++; }

  // 接口函数
  // 配置参数
  /**
   * @brief 设置超级电容的配置参数。
   *
   * 该函数检查并设置超级电容的配置参数。如果参数无效，则返回false。
   *
   * 有效参数：
   * - cfg.max_charge_volt: 必须大于0。
   * - cfg.min_valid_volt: 必须大于0且小于等于cfg.max_charge_volt。
   * - cfg.auto_disable_power: 必须在[0, 1)范围内。
   * - cfg.min_enable_power: 必须在[0, 1)范围内且小于cfg.auto_disable_power。
   *
   * @param cfg 配置参数的结构体。
   * @return 如果配置参数有效，则返回true；否则返回false。
   */
  bool setConfig(const Config &cfg);

  /**
   * @brief 设置超电版本，会根据版本号调用不同的编解码函数
   */
  void setVersion(Version ver) { version_ = ver; }

  /**
   * @brief 设置发送ID
   */
  void setTxId(uint32_t tx_id) { tx_id_ = tx_id; }

  /**
   * @brief 设置接收ID
   */
  void setRxId(uint32_t rx_id) { rx_id_ = rx_id; }

  /**
   * @brief 设置离线阈值
   */
  void setOfflineTickThres(uint32_t tick_thres) { oc_.set_offline_tick_thres(tick_thres); }

  // 设置发送数据
  /**
   * @brief 启用超级电容
   *
   * 如果之前已经在放电，则允许超电电压降至 cfg_.min_enable_power 以下。否则，只有当超电
   * 电压大于等于 cfg_.min_enable_power 时才允许开启超电。此举是为了防止操作手在不能完全
   * 不能放电时一直尝试开启超电。只有当超电电压大于等于 cfg_.auto_disable_power 时才允
   * 许开启超电。
   *
   * Ver2021 低于此电压超电不会放太多电流，反而会对功率限制有影响，并且在开启或禁用超电时，
   * 硬件问题造成的瞬时大电流会被裁判系统采集到，进而导致超功率扣血。
   *
   * @attention 并不能通过此函数的返回值判断是否成功禁用超电，需要通过 getPowerSource
   * 或者更严格的 isUsingSuperCap 来判断
   * @attention Ver2024 实际上不能关闭超电。
   * @return bool 返回当前的 enable_flag_ 状态。
   */
  bool enable(void)
  {
    last_enable_flag_ = enable_flag_;

    if (last_enable_flag_) {
      enable_flag_ = true;
    } else {
      enable_flag_ = remaining_pwr_ >= cfg_.min_enable_power;
    }

    enable_flag_ = enable_flag_ && (remaining_pwr_ >= cfg_.auto_disable_power);
    return enable_flag_;
  };

  /**
   * @brief 禁用超级电容器
   *
   * 将启用标志设置为false，并将请求的功率设置为0.0。
   *
   * @attention 并不能通过此函数的返回值判断是否成功禁用超电，需要通过 getPowerSource
   * 或者更严格的 isUsingSuperCap 来判断
   * @return 始终返回 true
   */
  bool disable(void)
  {
    enable_flag_ = false;
    return true;
  };

  /**
   * @brief 设置超电所需的裁判系统数据
   *
   * 设置裁判系统提供的功率缓冲、功率限制和机器人的血量。
   *
   * @param rfr_chas_pwr_buffer RFR的功率缓冲
   * @param rfr_chas_pwr_limit RFR的功率限制
   * @param robot_hp 机器人的血量，默认为0.0f (Ver2021不必提供)
   *
   * @attention Ver2024 在机器人血量为 0 时，会完全切断电源供给，即输出功率为 0，此举是为了防止“僵尸车”情况的出现
   */
  void setRfrData(uint16_t rfr_chas_pwr_buffer, uint16_t rfr_chas_pwr_limit,
                  uint16_t robot_hp = 0u)
  {
    rfr_chas_pwr_buffer_ = rfr_chas_pwr_buffer;
    rfr_chas_pwr_limit_ = rfr_chas_pwr_limit;
    robot_hp_ = robot_hp;
  };

  /**
   * @brief 设置请求的功率
   *
   * 如果启用标志为真，则将请求的功率设置为req_pwr，否则设置为0.0。
   *
   * @param req_pwr 请求的功率
   */
  void setRequestedPower(float req_pwr)
  {
    req_pwr_ = enable_flag_ ? req_pwr : 0.0f;
  }

  // 获取数据接口
  /**
   * @brief 获取超电配置
   */
  const Config &getConfig(void) const { return cfg_; }

  /**
   * @brief 获取离线阈值
   */
  uint32_t getOfflineTickThreshold(void) const
  {
    return oc_.get_offline_tick_thres();
  }

  /**
   * @brief 获取超电版本
   */
  Version getVersion(void) const { return version_; }

  /**
   * @brief 获取是否正在使用超级电容
   *
   * 如果当前电源为超级电容，且电压大于最小有效电压，且剩余功率大于自动禁用功率，则返回true，否则返回false。
   *
   * @retval true 正在使用超级电容
   * @retval false 未使用超级电容，或电压低于最小有效电压，或剩余功率低于自动禁用功率
   */
  bool isUsingSuperCap(void) const
  {
    return (pwr_src_ == PwrSrc::kSuperCap) && (volt_ >= cfg_.min_valid_volt) &&
           (remaining_pwr_ > cfg_.auto_disable_power);
  };

  /**
   * @brief 获取当前电源
   */
  PwrSrc getPowerSource(void) const { return pwr_src_; }

  /**
   * @brief 获取当前电压
   */
  float getVoltage(void) const { return volt_; }

  /**
   * @brief 获取剩余能量百分比
   *
   * @attention Ver2021 由此组件通过超电电压以及配置中的最小有效电压和最大充电电压计算得到， Ver2024 由超电提供
   * @return float 剩余能量百分比
   */
  float getRemainingPower(void) const { return remaining_pwr_; }

  /**
   * @brief 获取输出功率
   *
   * @attention 仅 Ver2024 会反馈有效值
   * @return float 输出功率
   */
  float getOutputPower(void) const { return out_pwr_; }

  /**
   * @brief 获取是否离线
   *
   * @attention 超电离线时，电压、剩余能量和输出功率会仍然保留最近一次接收到的值或者初始化值
   */
  bool isOffline(void) { return oc_.isOffline(); }

  /**
   * @brief 重置接收数据
   *
   * 将所有接收到的数据重置为默认值。
   */
  void resetRxData(void)
  {
    pwr_src_ = PwrSrc::kBattary;  ///< 当前能量来源
    volt_ = 0.0;                  ///< 电压，单位：V
    remaining_pwr_ = 0.0;         ///< 剩余能量，[0, 100]，单位：%
    out_pwr_ = 0.0;               ///< 输出功率，单位: W，仅限 Ver2024 有效
    is_updated_ = false;          ///< 数据是否更新
  }

 private:
  //  功能函数
  bool encodeVer2021(size_t &len, uint8_t *data);
  bool decodeVer2021(size_t len, const uint8_t *data);

  bool encodeVer2024(size_t &len, uint8_t *data);
  bool decodeVer2024(size_t len, const uint8_t *data);

  Version version_ = Version::kVer2021;  ///< 超电版本

  Config cfg_ = Config();  ///< 配置参数

  // rx
  bool is_updated_ = false;  ///< 数据是否更新
  uint32_t rx_id_ = 0x112;   ///< 接收超电 CAN 消息的 ID
  RxIds rx_ids_ = {rx_id_};  ///< 接收端 ID 列表

  pUpdateCallback update_cb_ = nullptr;  ///< 数据更新回调函数

  OfflineChecker oc_ = OfflineChecker(100);  ///< 离线检查器

  PwrSrc pwr_src_ = PwrSrc::kBattary;  ///< 当前能量来源
  float volt_ = 0.0;                   ///< 电压，单位：V
  float remaining_pwr_ = 0.0;          ///< 剩余能量，[0, 100]，单位：%
  float out_pwr_ = 0.0;                ///< 输出功率，单位: W，仅限 Ver2024 有效

  // tx
  uint32_t tx_id_ = 0x111;  ///< 给超电发送 CAN 消息的 ID
  TxIds tx_ids_ = {tx_id_};  ///< 发送端 ID 列表

  bool enable_flag_ = 0;  ///< 控制超级电容是否开启，开启后，若超电返回信息 pwr_scr_ 为 SuperCap 表明现在是从超电获取能量
  bool last_enable_flag_ = 0;         ///< 上一次的 enable_flag_ 状态
  uint16_t rfr_chas_pwr_buffer_ = 0;  ///< 裁判系统的缓冲能量，单位：J
  uint16_t rfr_chas_pwr_limit_ = 0;   ///< 裁判系统的底盘功率限制，单位：J
  uint16_t robot_hp_ = 0;             ///< 机器人生命值，只有 Ver2024 有效
  float req_pwr_ = 0.0;               ///< 要求超电提供的能量，单位：W，仅限 Ver2024 有效

  uint32_t decode_success_cnt_ = 0;  ///< 解码成功次数
  uint32_t decode_fail_cnt_ = 0;     ///< 解码失败次数
  uint32_t encode_success_cnt_ = 0;  ///< 编码成功次数
  uint32_t encode_fail_cnt_ = 0;     ///< 编码失败次数
  uint32_t tx_success_cnt_ = 0;      ///< 发送成功的次数
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace cap
}  // namespace hello_world
#endif /* HW_COMPONENTS_DEVICES_CAP_SUPER_CAP_HPP_ */
