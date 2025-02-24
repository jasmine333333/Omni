/**
 *******************************************************************************
 * @file      : rfr_crc.hpp
 * @brief     : 裁判系统数据包CRC校验
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *  该文件为内部文件，仅供内部调用，不对外开放
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_CPP_DEVICES_REFEREE_RFR_CRC_H_
#define HW_COMPONENTS_CPP_DEVICES_REFEREE_RFR_CRC_H_

/* Includes ------------------------------------------------------------------*/
#include <cstdbool>
#include <cstdint>

namespace hello_world
{
namespace referee
{
namespace internal
{
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * Verifies the CRC8 checksum of a message.
 *
 * This function calculates the expected CRC8 checksum of the input message and compares it to the last byte
 * of the message to verify its integrity.
 *
 * @param p_message[uint8_t *] Pointer to the message data.
 * @param length[uint32_t] Length of the message in bytes.
 * @return true if the CRC8 checksum matches the last byte of the message, false otherwise.
 */
bool VerifyCrc8CheckSum(uint8_t *p_message, uint32_t length);

/**
 * Set the last byte of a message to a CRC8 checksum.
 *
 * This function calculates the CRC8 checksum of the input message and sets it as the last byte of the message.
 *
 * @param p_message Pointer to the message data.
 * @param length Length of the message in bytes.
 * @return true if the CRC8 checksum is successfully set as the last byte, false otherwise.
 */
bool SetEndCrc8CheckSum(uint8_t *p_message, uint32_t length);

/**
 * Verifies the CRC16 checksum of a message.
 *
 * This function calculates the expected CRC16 checksum of the input message and compares it to the last two bytes
 * of the message to verify its integrity.
 *
 * @param p_message Pointer to the message data.
 * @param length Length of the message in bytes.
 * @return true if the CRC16 checksum matches the last two bytes of the message, false otherwise.
 */
bool VerifyCrc16CheckSum(uint8_t *p_message, uint32_t length);

/**
 * Set the last two bytes of a message to a CRC16 checksum.
 *
 * This function calculates the CRC16 checksum of the input message and sets it as the last two bytes of the message.
 *
 * @param p_message Pointer to the message data.
 * @param length Length of the message in bytes.
 * @return true if the CRC16 checksum is successfully set as the last two bytes, false otherwise.
 */
bool SetEndCrc16CheckSum(uint8_t *p_message, uint32_t length);
}  // namespace internal
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_CPP_DEVICES_REFEREE_RFR_CRC_H_ */
