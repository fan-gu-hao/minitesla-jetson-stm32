#ifndef MINITESLA_TOF4_H
#define MINITESLA_TOF4_H
#include <stdint.h>
#include "stm32f4xx_hal.h"

enum { TOF_FRONT, TOF_REAR, TOF_LEFT, TOF_RIGHT, TOF_COUNT };
#define TOF_TIMING_BUDGET_US 33000U
#define TOF_PERIOD_MS        50U

/* Diagnostic arrays: read with ST-LINK Live Expressions, no UART changes. */
extern volatile uint16_t tof_distance_mm[TOF_COUNT];
extern volatile uint8_t tof_range_status[TOF_COUNT];
extern volatile int8_t tof_api_error[TOF_COUNT];
extern volatile uint32_t tof_samples[TOF_COUNT];
extern volatile uint32_t tof_last_read_ms[TOF_COUNT];
extern volatile uint8_t tof_init_failed_mask;

void TOF4_GPIO_Init(void);
uint8_t TOF4_Init(I2C_HandleTypeDef *i2c);
/* One sensor is serviced per call. Ranging runs inside all four devices.
 * No wait-for-measurement loop; individual I2C transfers remain blocking. */
void TOF4_Service(void);

/* Implemented by main.c; publication is atomic with respect to TIM5 IRQ. */
uint32_t App_Millis(void);
void App_PublishToF(unsigned index, uint16_t mm, uint8_t status,
                    uint32_t read_ms);
#endif
