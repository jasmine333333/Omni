/**
 * @file      rfr_encoder.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-02-18
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | description |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_ENCODER_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_ENCODER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg/rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class RfrEncoder : public MemMgr
{
 public:
  struct __REFEREE_PACKED TxFrameHeader {
    uint8_t sof;
    DataLength data_length;
    uint8_t seq;
    Crc8 crc8;
    CmdId cmd_id;
  };
  enum TxIdx {
    kTxIdxData = sizeof(TxFrameHeader),
  };

  RfrEncoder(void) = default;
  RfrEncoder(const RfrEncoder &) = default;
  RfrEncoder &operator=(const RfrEncoder &) = default;
  RfrEncoder(RfrEncoder &&) = default;
  RfrEncoder &operator=(RfrEncoder &&) = default;

  ~RfrEncoder(void) = default;

  bool encodeFrame(
      ProtocolTxPackage *pkg_ptr, uint8_t *frame_ptr, size_t *frame_len_ptr);

 private:
  union {
    uint8_t raw[kRefereeMaxFrameLength];
    TxFrameHeader header;
  } data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_ENCODER_HPP_ */
