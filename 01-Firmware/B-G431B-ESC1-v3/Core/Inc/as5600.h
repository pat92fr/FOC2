/*
 * as5600.h
 *
 *  Created on: 26.03.2021
 *      Author: kai
 */

#ifndef AS5600_H_
#define AS5600_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/


typedef struct {
    I2C_HandleTypeDef *i2cHandle;
    uint8_t i2cAddr;
} AS5600_TypeDef;


AS5600_TypeDef *AS5600_New(void);
HAL_StatusTypeDef AS5600_Init(AS5600_TypeDef *a);

HAL_StatusTypeDef AS5600_SetStartPosition(AS5600_TypeDef *const a, const uint16_t pos);
HAL_StatusTypeDef AS5600_SetStopPosition(AS5600_TypeDef *const a, const uint16_t pos);
HAL_StatusTypeDef AS5600_SetMaxAngle(AS5600_TypeDef *const a, const uint16_t angle);
HAL_StatusTypeDef AS5600_GetRawAngle(AS5600_TypeDef *const a, uint16_t *const angle);
HAL_StatusTypeDef AS5600_GetAngle(AS5600_TypeDef *const a, uint16_t *const angle);
HAL_StatusTypeDef AS5600_GetAGCSetting(AS5600_TypeDef *const a, uint8_t *const agc);
HAL_StatusTypeDef AS5600_GetCORDICMagnitude(AS5600_TypeDef *const a, uint16_t *const mag);

#endif /* AS5600_H_ */
