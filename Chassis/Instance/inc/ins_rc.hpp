/** 
 *******************************************************************************
 * @file      : ins_rc.hpp
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
#ifndef INS_RC_HPP_
#define INS_RC_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"

namespace hw_rc = hello_world::remote_control;

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hw_rc::DT7* CreateRemoteControl(void);

#endif /* INS_RC_HPP_ */
