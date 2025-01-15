/**
 *******************************************************************************
 * @file      :offline_checker.hpp
 * @brief     : 用于实现掉线检测功能
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_OFFLINE_CHECKER_HPP_
#define HW_COMPONENTS_TOOLS_OFFLINE_CHECKER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <limits>

#include "allocator.hpp"
#include "system.hpp"
#include "tick.hpp"

namespace hello_world
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START
class OfflineChecker : public MemMgr
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  OfflineChecker(void) = default;

  /**
   * @brief       构造函数
   * @param        offline_tick_thres: 掉线阈值，单位：ms
   * @retval       None
   * @note        None
   */
  explicit OfflineChecker(uint32_t offline_tick_thres)
      : offline_tick_thres_(offline_tick_thres) {}
  OfflineChecker(const OfflineChecker&) = default;
  OfflineChecker& operator=(const OfflineChecker&) = default;
  OfflineChecker(OfflineChecker&&) = default;
  OfflineChecker& operator=(OfflineChecker&&) = default;

  ~OfflineChecker() = default;

  /**
   * @brief       初始化，使用默认构造函数后请务必调用此函数
   * @param        offline_tick_thres: 掉线阈值，单位：ms
   * @retval       None
   * @note        None
   */
  void init(uint32_t offline_tick_thres)
  {
    offline_tick_thres_ = offline_tick_thres;
  }

  /**
   * @brief       更新状态
   * @retval       None
   * @note        在接收到设备反馈时调用，与 isOffline 搭配使用
   */
  void update(void)
  {
    last_update_tick_ = tick::GetTickMs();
    is_offline_ = false;
  }

  /* 数据修改与获取 */

  uint32_t get_offline_tick_thres(void) const { return offline_tick_thres_; }

  void set_offline_tick_thres(uint32_t offline_tick_thres)
  {
    offline_tick_thres_ = offline_tick_thres;
  }

  /**
   * @brief       掉线检查
   * @retval       是否掉线
   * @note        建议定时调用该函数以检查掉线情况，与 update 搭配使用
   */
  bool isOffline(void)
  {
    if (is_offline_) {
      return true;
    }

    uint32_t last_update_tick_copy = last_update_tick_;
    int64_t diff = tick::GetTickMs() - last_update_tick_copy;
    if (diff < 0) {
      /* 考虑定时器溢出情况 */
      diff += std::numeric_limits<uint32_t>::max();
    }
    
    diff_ = diff;

    if (diff > offline_tick_thres_) {
      is_offline_ = true;
    }
    return is_offline_;
  }
 
 public:
  float diff_ = 0;
 private:
  uint32_t offline_tick_thres_ = 100;  ///< 单位：ms
  uint32_t last_update_tick_ = 0;      ///< 单位：ms
  bool is_offline_ = true;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_OFFLINE_CHECKER_HPP_ */
