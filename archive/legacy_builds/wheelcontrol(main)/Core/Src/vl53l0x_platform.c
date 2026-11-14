#include "vl53l0x_platform.h"

#ifndef HAL_I2C_MODULE_ENABLED
#error "Enable HAL_I2C_MODULE_ENABLED and compile stm32f4xx_hal_i2c.c"
#endif

static VL53L0X_Error convert_status(VL53L0X_DEV dev, HAL_StatusTypeDef status)
{
    if (status != HAL_OK) dev->io_failed = 1U;
    return status == HAL_OK ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

static int valid_request(VL53L0X_DEV dev, const void *data, uint32_t count)
{
    return dev != NULL && dev->I2cHandle != NULL && data != NULL &&
           count > 0U && count <= 32U;
}

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV dev)
{
    (void)dev;
    /* One foreground owner. Do not disable interrupts during sensor access. */
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV dev)
{
    (void)dev;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t index,
                               uint8_t *data, uint32_t count)
{
    if (!valid_request(dev, data, count)) return VL53L0X_ERROR_INVALID_PARAMS;
    if (dev->io_failed != 0U) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return convert_status(dev, HAL_I2C_Mem_Write(dev->I2cHandle, dev->I2cDevAddr,
        index, I2C_MEMADD_SIZE_8BIT, data, (uint16_t)count,
        VL53L0X_HAL_I2C_TIMEOUT_MS));
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t index,
                              uint8_t *data, uint32_t count)
{
    HAL_StatusTypeDef status;
    if (!valid_request(dev, data, count)) return VL53L0X_ERROR_INVALID_PARAMS;
    memset(data, 0, count);
    if (dev->io_failed != 0U) return VL53L0X_ERROR_CONTROL_INTERFACE;
    status = HAL_I2C_Mem_Read(dev->I2cHandle, dev->I2cDevAddr,
        index, I2C_MEMADD_SIZE_8BIT, data, (uint16_t)count,
        VL53L0X_HAL_I2C_TIMEOUT_MS);
    if (status != HAL_OK) memset(data, 0, count);
    return convert_status(dev, status);
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(dev, index, &data, 1U);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t index, uint16_t data)
{
    uint8_t bytes[2] = {(uint8_t)(data >> 8), (uint8_t)data};
    return VL53L0X_WriteMulti(dev, index, bytes, 2U);
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t index, uint32_t data)
{
    uint8_t bytes[4] = {(uint8_t)(data >> 24), (uint8_t)(data >> 16),
                        (uint8_t)(data >> 8), (uint8_t)data};
    return VL53L0X_WriteMulti(dev, index, bytes, 4U);
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(dev, index, data, 1U);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t index, uint16_t *data)
{
    uint8_t bytes[2] = {0U};
    VL53L0X_Error status;
    if (data == NULL) return VL53L0X_ERROR_INVALID_PARAMS;
    *data = 0U;
    status = VL53L0X_ReadMulti(dev, index, bytes, 2U);
    if (status == VL53L0X_ERROR_NONE)
        *data = ((uint16_t)bytes[0] << 8) | (uint16_t)bytes[1];
    return status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t index, uint32_t *data)
{
    uint8_t bytes[4] = {0U};
    VL53L0X_Error status;
    if (data == NULL) return VL53L0X_ERROR_INVALID_PARAMS;
    *data = 0U;
    status = VL53L0X_ReadMulti(dev, index, bytes, 4U);
    if (status == VL53L0X_ERROR_NONE)
        *data = ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
                ((uint32_t)bytes[2] << 8) | (uint32_t)bytes[3];
    return status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t index,
                               uint8_t and_data, uint8_t or_data)
{
    uint8_t data = 0U;
    VL53L0X_Error status = VL53L0X_RdByte(dev, index, &data);
    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_WrByte(dev, index, (uint8_t)((data & and_data) | or_data));
    return status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev)
{
    (void)dev;
    /* Used by blocking ST startup/calibration routines, not normal ranging. */
    HAL_Delay(1U);
    return VL53L0X_ERROR_NONE;
}
