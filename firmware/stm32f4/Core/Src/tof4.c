/* Four VL53L0X on I2C1 PB8/PB9; XSHUT PG10/11/12/13.
 * Module VIN and pull-up voltage must be checked against the actual breakout.
 * XSHUT is open drain: drive LOW or RELEASE, never push 3.3 V into a 2.8 V pin.
 */
#include "tof4.h"
#include "vl53l0x_api.h"
#include <string.h>

static VL53L0X_Dev_t dev[TOF_COUNT];
static uint8_t running[TOF_COUNT];
static const uint8_t address7[TOF_COUNT] = {0x30, 0x31, 0x32, 0x33};
static const uint16_t xshut[TOF_COUNT] = {
    GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13
};
volatile uint16_t tof_distance_mm[TOF_COUNT];
volatile uint8_t tof_range_status[TOF_COUNT] = {255,255,255,255};
volatile int8_t tof_api_error[TOF_COUNT];
volatile uint32_t tof_samples[TOF_COUNT];
volatile uint32_t tof_last_read_ms[TOF_COUNT];
volatile uint8_t tof_init_failed_mask;

void TOF4_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOG_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10 | GPIO_PIN_11 |
                     GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
    g.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &g);
}

static void fail(unsigned i, VL53L0X_Error error)
{
    running[i] = 0;
    tof_api_error[i] = error;
    tof_range_status[i] = 254; /* Host I2C/API error, not a ST RangeStatus. */
    App_PublishToF(i, 0, 254, App_Millis());
    /* Do not hot-restart one device at default address while the car is active.
     * Fix the wiring/bus and reset the M4 to repeat full address assignment. */
}

static VL53L0X_Error init_one(unsigned i, I2C_HandleTypeDef *i2c)
{
    VL53L0X_Error e;
    uint32_t spads;
    uint8_t aperture, vhv, phase, model_id = 0;
    VL53L0X_DEV d = &dev[i];
    memset(d, 0, sizeof(*d));
    d->I2cHandle = i2c;
    d->I2cDevAddr = 0x52; /* ST/HAL 8-bit address; 7-bit default is 0x29. */
    d->comms_type = 1;
    d->comms_speed_khz = 100;
    HAL_GPIO_WritePin(GPIOG, xshut[i], GPIO_PIN_SET); /* Release only. */
    HAL_Delay(10);
#define TRY(call) do { \
    e = (call); \
    if (e != VL53L0X_ERROR_NONE) return e; \
    if (d->io_failed) return VL53L0X_ERROR_CONTROL_INTERFACE; \
} while (0)
    TRY(VL53L0X_RdByte(d, 0xC0, &model_id));
    if (model_id != 0xEE) return VL53L0X_ERROR_NOT_SUPPORTED;
    TRY(VL53L0X_DataInit(d));
    TRY(VL53L0X_SetDeviceAddress(d, (uint8_t)(address7[i] << 1)));
    d->I2cDevAddr = (uint8_t)(address7[i] << 1);
    TRY(VL53L0X_StaticInit(d));
    TRY(VL53L0X_PerformRefSpadManagement(d, &spads, &aperture));
    TRY(VL53L0X_PerformRefCalibration(d, &vhv, &phase));
    TRY(VL53L0X_SetDeviceMode(d, VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING));
    TRY(VL53L0X_SetMeasurementTimingBudgetMicroSeconds(d, TOF_TIMING_BUDGET_US));
    TRY(VL53L0X_SetInterMeasurementPeriodMilliSeconds(d, TOF_PERIOD_MS));
    TRY(VL53L0X_SetGpioConfig(d, 0,
        VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING,
        VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
        VL53L0X_INTERRUPTPOLARITY_LOW));
    TRY(VL53L0X_ClearInterruptMask(d, 0));
#undef TRY
    return VL53L0X_ERROR_NONE;
}

uint8_t TOF4_Init(I2C_HandleTypeDef *i2c)
{
    unsigned i;
    uint8_t mask = 0;
    HAL_Delay(5); /* All XSHUT pins were driven low before I2C was enabled. */
    for (i = 0; i < TOF_COUNT; ++i) {
        VL53L0X_Error e = init_one(i, i2c);
        if (e != VL53L0X_ERROR_NONE || dev[i].io_failed) {
            fail(i, e != VL53L0X_ERROR_NONE ? e : VL53L0X_ERROR_CONTROL_INTERFACE);
            mask |= (uint8_t)(1U << i);
            /* Prevent an unsuccessfully addressed sensor staying at 0x29. */
            HAL_GPIO_WritePin(GPIOG, xshut[i], GPIO_PIN_RESET);
        } else {
            running[i] = 1;
        }
    }
    /* Start only after every device has been addressed/calibrated. */
    for (i = 0; i < TOF_COUNT; ++i) {
        if (running[i]) {
            VL53L0X_Error e = VL53L0X_StartMeasurement(&dev[i]);
            if (e != VL53L0X_ERROR_NONE || dev[i].io_failed) {
                fail(i, e != VL53L0X_ERROR_NONE ? e : VL53L0X_ERROR_CONTROL_INTERFACE);
                mask |= (uint8_t)(1U << i);
            }
        }
    }
    tof_init_failed_mask = mask;
    return mask;
}

void TOF4_Service(void)
{
    static unsigned next;
    unsigned i = next;
    uint8_t ready = 0;
    uint32_t observed_ms;
    VL53L0X_RangingMeasurementData_t measurement;
    VL53L0X_Error e;
    next = (next + 1U) % TOF_COUNT;
    if (!running[i]) return;
    e = VL53L0X_GetMeasurementDataReady(&dev[i], &ready);
    if (e != VL53L0X_ERROR_NONE || dev[i].io_failed) {
        fail(i, VL53L0X_ERROR_CONTROL_INTERFACE);
        return;
    }
    if (!ready) return; /* Never refresh the timestamp for an old sample. */
    observed_ms = App_Millis(); /* Host observation, not exact exposure time. */
    memset(&measurement, 0, sizeof(measurement));
    e = VL53L0X_GetRangingMeasurementData(&dev[i], &measurement);
    if (e == VL53L0X_ERROR_NONE && !dev[i].io_failed)
        e = VL53L0X_ClearInterruptMask(&dev[i], 0);
    if (e != VL53L0X_ERROR_NONE || dev[i].io_failed) {
        fail(i, VL53L0X_ERROR_CONTROL_INTERFACE);
        return;
    }
    tof_distance_mm[i] = measurement.RangeMilliMeter;
    tof_range_status[i] = measurement.RangeStatus;
    tof_last_read_ms[i] = observed_ms;
    ++tof_samples[i];
    /* Publish invalid readings too: never hide failure with the last good one. */
    App_PublishToF(i, measurement.RangeMilliMeter, measurement.RangeStatus, observed_ms);
}
