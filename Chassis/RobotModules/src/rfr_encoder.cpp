/**
 * @file      rfr_encoder.cpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-02-19
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

/* Includes ------------------------------------------------------------------*/
#include "rfr_encoder.hpp"

#include "rfr_crc.hpp"

namespace hello_world
{
namespace referee
{
using namespace internal;
/* Exported macro ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

bool RfrEncoder::encodeFrame(
    ProtocolTxPackage *pkg_ptr, uint8_t *frame_ptr, size_t *frame_len_ptr)
{
  HW_ASSERT(pkg_ptr != nullptr, "Invalid ProtocolTxPackage pointer %p", pkg_ptr);
  HW_ASSERT(frame_ptr != nullptr, "Invalid frame_ptr pointer %p", frame_ptr);
  HW_ASSERT(frame_len_ptr != nullptr, "Invalid frame_len_ptr pointer %p",
            frame_len_ptr);
  if (pkg_ptr == nullptr || frame_ptr == nullptr || frame_len_ptr == nullptr) {
    return false;
  }

  data_.header.sof = kRefereeFrameHeaderSof;
  data_.header.data_length = pkg_ptr->getDataLength();
  data_.header.seq = data_.header.seq == 0xFF ? 0 : data_.header.seq + 1;
  if (!SetEndCrc8CheckSum(data_.raw, sizeof(FrameHeader))) {
    return false;
  }
  data_.header.cmd_id = pkg_ptr->getCmdId();

  if (!pkg_ptr->encode(data_.raw + sizeof(TxFrameHeader))) {
    return false;
  }

  size_t frame_len =
      sizeof(TxFrameHeader) + pkg_ptr->getDataLength() + sizeof(Crc16);

  if (!SetEndCrc16CheckSum(data_.raw, frame_len)) {
    return false;
  }

  memcpy(frame_ptr, data_.raw, frame_len);
  *frame_len_ptr = frame_len;
  return true;
}
/* Private function definitions ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world