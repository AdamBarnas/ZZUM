#include "as5600_hal.h"

#define AS5600_REG_ANGLE_H   0x0E
#define AS5600_REG_ANGLE_L   0x0F

static bool as5600_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(&hi2c3, (AS5600_I2C_ADDR << 1), &reg, 1, 5) != HAL_OK)
        return false;

    if (HAL_I2C_Master_Receive(&hi2c3, (AS5600_I2C_ADDR << 1), buf, len, 5) != HAL_OK)
        return false;

    return true;
}

bool AS5600_ReadRaw12(uint16_t *raw12)
{
    uint8_t data[2];
    if (!as5600_read(AS5600_REG_ANGLE_H, data, 2))
        return false;

    *raw12 = ((uint16_t)data[0] << 8 | data[1]) & 0x0FFF;
    return true;
}
