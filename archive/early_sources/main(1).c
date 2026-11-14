/* ===========================
 * STM32F412ZGT6  - USART2 on PA2(TX)/PA3(RX) + IRQ + Polling Fallback + CRC8 + Failsafe
 * ??: [0x55,0xAA,0x02,SEQ,ESC,SERVO,CRC8]
 * PWM: PSC=159, ARR=1013 (˜98.6 Hz), CCR = byte * 10us
 * =========================== */

#include "main.h"
#include "stm32f4xx.h"
#include <stdint.h>

/* ---- ???? ---- */
#define ESC_MIN     142   /*122*/
#define ESC_MID     152   /*152*/ 
#define ESC_MAX     160   /*182*/
#define SERVO_MIN    94
#define SERVO_MID   152
#define SERVO_MAX   210

/* ---- UART / ?? ---- */
#define UART_BAUD   115200       /* ? 115200;??? Jetson/STM ??? 460800/921600 */
#define SYNC1       0x55
#define SYNC2       0xAA
#define EXP_LEN     0x02

/* ---- ??/?? ---- */
static volatile uint8_t  esc_byte   = ESC_MID;
static volatile uint8_t  servo_byte = SERVO_MID;
static volatile uint8_t  new_frame  = 0;
static volatile uint32_t last_ok_ms = 0;

volatile uint32_t rxne_cnt = 0;
volatile uint32_t good_cnt = 0;
volatile uint32_t crc_fail_cnt = 0;
volatile uint32_t idle_cnt = 0;
volatile uint32_t ore_cnt = 0;
volatile uint8_t  last6[6] = {0};
volatile uint8_t  last_byte = 0;

/* ---- ???? ---- */
void SystemClock_Config(void);
static void GPIO_PWM_Init(void);
static void TIM1_PWM_Init(void);
static void TIM2_PWM_Init(void);

static void USART2_Init_PA2_PA3(void);
static inline void USART2_SetBaud(uint32_t baud);

static uint8_t crc8(const uint8_t* p, uint32_t n);
static inline void apply_pwm_now(void);
static inline void uart2_poll_rx(void);

/* ===== GPIO / PWM(????) ===== */
static void GPIO_PWM_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    /* PA8: TIM1_CH1 (SERVO) AF1 */
    GPIOA->MODER  &= ~(3U << (8U*2U));
    GPIOA->MODER  |=  (2U << (8U*2U));
    GPIOA->AFR[1] &= ~(0xFU << ((8U-8U)*4U));
    GPIOA->AFR[1] |=  (1U   << ((8U-8U)*4U));
    /* PA1: TIM2_CH2 (ESC) AF1 */
    GPIOA->MODER  &= ~(3U << (1U*2U));
    GPIOA->MODER  |=  (2U << (1U*2U));
    GPIOA->AFR[0] &= ~(0xFU << (1U*4U));
    GPIOA->AFR[0] |=  (1U   << (1U*4U));
}

static void TIM1_PWM_Init(void){
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC  = 159;
    TIM1->ARR  = 1013;
    TIM1->CCR1 = SERVO_MID;
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM1->CCER  |= TIM_CCER_CC1E;
    TIM1->BDTR  |= TIM_BDTR_MOE;
    TIM1->CR1   |= TIM_CR1_ARPE;
    TIM1->EGR   |= TIM_EGR_UG;
    TIM1->CR1   |= TIM_CR1_CEN;
}

static void TIM2_PWM_Init(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC  = 159;
    TIM2->ARR  = 1013;
    TIM2->CCR2 = ESC_MID;
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM2->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos);
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM2->CCER  |= TIM_CCER_CC2E;
    TIM2->CR1   |= TIM_CR1_ARPE;
    TIM2->EGR   |= TIM_EGR_UG;
    TIM2->CR1   |= TIM_CR1_CEN;
}

/* ===== USART2: PA2=TX, PA3=RX (AF7, APB1) ===== */
static inline void USART2_SetBaud(uint32_t baud){
    /* APB1 = HCLK(?????????1),Oversampling 16 */
    uint32_t fck = SystemCoreClock;
    uint32_t usartdiv16 = (fck + (baud/2U)) / baud;
    uint32_t mant = usartdiv16 / 16U;
    uint32_t frac = usartdiv16 % 16U;
    USART2->BRR = (mant << 4) | (frac & 0xFU);
}

static void USART2_Init_PA2_PA3(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    /* PA2/PA3 ? AF7, Very High;PA3(RX) ?? */
    GPIOA->MODER   &= ~((3U<<(2U*2U)) | (3U<<(3U*2U)));
    GPIOA->MODER   |=  ((2U<<(2U*2U)) | (2U<<(3U*2U)));
    GPIOA->AFR[0]  &= ~((0xFU<<(2U*4U)) | (0xFU<<(3U*4U)));
    GPIOA->AFR[0]  |=  ((7U<<(2U*4U)) | (7U<<(3U*4U)));
    GPIOA->OSPEEDR &= ~((3U<<(2U*2U)) | (3U<<(3U*2U)));
    GPIOA->OSPEEDR |=  ((3U<<(2U*2U)) | (3U<<(3U*2U)));  /* Very High */
    GPIOA->PUPDR   &= ~((3U<<(2U*2U)) | (3U<<(3U*2U)));
    GPIOA->PUPDR   |=  ((0U<<(2U*2U)) | (1U<<(3U*2U)));  /* PA2: no-pull, PA3: pull-up */

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->CR1 = 0; USART2->CR2 = 0; USART2->CR3 = 0;
    USART2_SetBaud(UART_BAUD);
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_IDLEIE;

    NVIC_SetPriority(USART2_IRQn, 5);
    NVIC_EnableIRQ(USART2_IRQn);

    USART2->CR1 |= USART_CR1_UE;
}

/* ===== CRC8 poly=0x07 ===== */
static uint8_t crc8(const uint8_t* p, uint32_t n){
    uint8_t c=0;
    for(uint32_t i=0;i<n;i++){
        c ^= p[i];
        for(int k=0;k<8;k++){ c = (c & 0x80) ? (uint8_t)(((uint16_t)c<<1) ^ 0x07) : (uint8_t)(c<<1); }
    }
    return c;
}

/* ===== ??? ===== */
typedef enum { ST_S1=0, ST_S2, ST_LEN, ST_SEQ, ST_ESC, ST_SERVO, ST_CRC } ustate_t;
static volatile ustate_t ust = ST_S1;
static uint8_t pkt[6];

static void parser_reset(void){ ust=ST_S1; }
static inline void parser_feed(uint8_t b){
    switch(ust){
    case ST_S1:   if(b==SYNC1){ pkt[0]=b; ust=ST_S2; } break;
    case ST_S2:   if(b==SYNC2){ pkt[1]=b; ust=ST_LEN; } else parser_reset(); break;
    case ST_LEN:  if(b==EXP_LEN){ pkt[2]=b; ust=ST_SEQ; } else parser_reset(); break;
    case ST_SEQ:  pkt[3]=b; ust=ST_ESC; break;
    case ST_ESC:  pkt[4]=b; ust=ST_SERVO; break;
    case ST_SERVO:pkt[5]=b; ust=ST_CRC; break;
    case ST_CRC:{
        uint8_t c = crc8(pkt,6);
        for(int i=0;i<6;i++) last6[i]=pkt[i];
        if(b==c){
            uint8_t e=pkt[4], s=pkt[5];
            if(e<ESC_MIN) e=ESC_MIN; if(e>ESC_MAX) e=ESC_MAX;
            if(s<SERVO_MIN) s=SERVO_MIN; if(s>SERVO_MAX) s=SERVO_MAX;
            esc_byte=e; servo_byte=s; new_frame=1;
            last_ok_ms = HAL_GetTick();
            good_cnt++;
        }else{ crc_fail_cnt++; }
        parser_reset();
    } break;
    }
}

/* ===== IRQ(USART2) ===== */
void USART2_IRQHandler(void){
    uint32_t sr = USART2->SR;
    if(sr & USART_SR_ORE){ (void)USART2->DR; (void)USART2->SR; ore_cnt++; }
    if(sr & USART_SR_RXNE){
        uint8_t b = (uint8_t)USART2->DR;
        last_byte=b; rxne_cnt++; parser_feed(b);
    }
    if(sr & USART_SR_IDLE){
        volatile uint32_t tmp = USART2->SR; (void)tmp;
        (void)USART2->DR; idle_cnt++;
    }
}

/* ---- ?? fallback:?? IRQ ????? ---- */
static inline void uart2_poll_rx(void){
    uint32_t sr = USART2->SR;
    if(sr & USART_SR_ORE){ (void)USART2->DR; (void)USART2->SR; ore_cnt++; }
    if(sr & USART_SR_RXNE){
        uint8_t b = (uint8_t)USART2->DR;
        last_byte=b; rxne_cnt++; parser_feed(b);
    }
}

/* ---- ???? PWM ---- */
static inline void apply_pwm_now(void){
    TIM2->CCR2 = esc_byte;    /* ESC */
    TIM1->CCR1 = servo_byte;  /* SERVO */
}

/* ---- ??? ---- */
int main(void){
    HAL_Init();
    SystemClock_Config();

    GPIO_PWM_Init();
    TIM1_PWM_Init();
    TIM2_PWM_Init();
    USART2_Init_PA2_PA3();

    last_ok_ms = HAL_GetTick();
    apply_pwm_now();

    while(1){
        uart2_poll_rx();                 /* fallback */
        if(new_frame){ new_frame=0; apply_pwm_now(); }
        if((HAL_GetTick() - last_ok_ms) > 300U){
            esc_byte=ESC_MID; servo_byte=SERVO_MID; apply_pwm_now(); last_ok_ms = HAL_GetTick();
        }
    }
}

/* ---- SystemClock:HSI 16MHz / ? PLL ---- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { while(1){} }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { while(1){} }
}

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
