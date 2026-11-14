
#include "spi.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;


/* ===================== SPI1 Slave ===================== */
static void SPI1_slave_Init(void){
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA4..PA7 -> AF5 (SPI1_NSS/SCK/MISO/MOSI) */
    GPIOA->MODER &= ~((0x3 << (4*2)) | (0x3 << (5*2)) | (0x3 << (6*2)) | (0x3 << (7*2)));
    GPIOA->MODER |=  ((0x2 << (4*2)) | (0x2 << (5*2)) | (0x2 << (6*2)) | (0x2 << (7*2)));
    GPIOA->AFR[0]  &= ~((0xF << (4*4)) | (0xF << (5*4)) | (0xF << (6*4)) | (0xF << (7*4)));
    GPIOA->AFR[0]  |=  ((5   << (4*4)) | (5   << (5*4)) | (5   << (6*4)) | (5   << (7*4)));

    /* SPI1: Slave, MODE1 (CPOL=0, CPHA=1), ?? NSS */
    SPI1->CR1 = 0;
    SPI1->CR1 &= ~SPI_CR1_MSTR;      // Slave
    SPI1->CR1 &= ~SPI_CR1_CPOL;      // CPOL = 0
    SPI1->CR1 |=  SPI_CR1_CPHA;      // CPHA = 1
    SPI1->CR1 &= ~SPI_CR1_SSM;       // ?? NSS

    SPI1->CR2 = 0;                   // 8-bit
    SPI1->CR2 |= SPI_CR2_RXDMAEN;    // ? RX DMA(TXDMA ??)

    SPI1->CR1 |= SPI_CR1_SPE;        // ?? SPI
}

/* ===================== DMA (SPI1_RX: DMA2/Stream0/Channel3) ===================== */


