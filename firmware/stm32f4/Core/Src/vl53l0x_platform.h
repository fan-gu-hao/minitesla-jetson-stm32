/* MiniTesla STM32F4 HAL platform for ST VL53L0X API.
 * Each sensor owns a VL53L0X_Dev_t; all calls run in foreground only.
 * Keep sensor I2C/API calls out of control and UART interrupt handlers.
 */
#ifndef MINITESLA_VL53L0X_PLATFORM_H
#define MINITESLA_VL53L0X_PLATFORM_H

#include "stm32f4xx_hal.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform_log.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    VL53L0X_DevData_t Data;
    I2C_HandleTypeDef *I2cHandle;
    uint8_t I2cDevAddr; /* HAL/ST API address: 7-bit address shifted left once. */
    uint8_t comms_type;
    uint16_t comms_speed_khz;
    uint8_t io_failed; /* Sticky HAL I2C failure; cleared only before cold init. */
} VL53L0X_Dev_t;
typedef VL53L0X_Dev_t *VL53L0X_DEV;

#define PALDevDataGet(Dev, field) ((Dev)->Data.field)
#define PALDevDataSet(Dev, field, data) ((Dev)->Data.field = (data))
#define VL53L0X_COPYSTRING(str, ...) strcpy(str, __VA_ARGS__)

/* No wait for sensor measurement here. HAL has finite transaction timeouts,
 * but its separate BUSY wait may exceed this timeout argument. All timeouts
 * still require a working SysTick interrupt; control runs independently.
 */
#ifndef VL53L0X_HAL_I2C_TIMEOUT_MS
#define VL53L0X_HAL_I2C_TIMEOUT_MS 5U
#endif

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV dev);
VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV dev);
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t index, uint8_t *data, uint32_t count);
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t index, uint8_t *data, uint32_t count);
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t index, uint8_t data);
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t index, uint16_t data);
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t index, uint32_t data);
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t index, uint8_t *data);
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t index, uint16_t *data);
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t index, uint32_t *data);
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t index, uint8_t and_data, uint8_t or_data);
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev);

#ifdef __cplusplus
}
#endif
#endif
