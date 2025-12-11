/*
 * as5600_hal.h
 *
 *  Created on: Oct 27, 2025
 *      Author: krzys
 */

#ifndef INC_AS5600_HAL_H_
#define INC_AS5600_HAL_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// Masz I2C na PA8/PC9 → I2C3
extern I2C_HandleTypeDef hi2c3;

// 7-bit address AS5600
#define AS5600_I2C_ADDR   (0x36)

// API
bool AS5600_ReadRaw12(uint16_t *raw12);
bool AS5600_ReadAngleRad(float *rad);
bool AS5600_ReadAngleDeg(float *deg);

#endif /* INC_AS5600_HAL_H_ */
