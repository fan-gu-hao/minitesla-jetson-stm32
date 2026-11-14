/* Replacement interrupt file. Do not compile alongside the old copy.
 * USART2 and TIM5 handlers are defined in main.c.
 * Keep the STM32F412 startup vector table from the original project.
 */
#include "stm32f4xx_hal.h"
extern void App_FaultStop(void);
void NMI_Handler(void)       { App_FaultStop(); }
void HardFault_Handler(void) { App_FaultStop(); }
void MemManage_Handler(void) { App_FaultStop(); }
void BusFault_Handler(void)  { App_FaultStop(); }
void UsageFault_Handler(void){ App_FaultStop(); }
void SVC_Handler(void)       { }
void DebugMon_Handler(void)  { }
void PendSV_Handler(void)    { }
void SysTick_Handler(void)   { HAL_IncTick(); }
