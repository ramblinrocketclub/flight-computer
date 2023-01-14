#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "stm32h7xx.h"
#include "stm32h7xx_ll_dma.h"
#include "gps.h"
#include "hguide_imu.h"
#include "printf.h"
#include "rtc.h"
#include "FreeRTOS.h"
#include "task.h"

#define AF07                    7UL
#define AF08                    8UL
#define AF11                    11UL

#define BUFFER_EMPTY            0UL
#define BUFFER_FULL             1UL

#define GPS_BUFFER_SIZE         512UL
#define HGUIDE_BUFFER_SIZE      512UL
#define LOG_BUFFER_SIZE         512UL

#define SIZE(array)         (sizeof(array) / sizeof(array[0]))

__attribute__ ((section(".buffer"))) volatile uint8_t uart8_rx_data[GPS_BUFFER_SIZE];
__attribute__ ((section(".buffer"))) volatile uint8_t uart8_tx_data[GPS_BUFFER_SIZE];
__attribute__ ((section(".buffer"))) volatile uint8_t uart7_rx_data[HGUIDE_BUFFER_SIZE];
__attribute__ ((section(".buffer"))) volatile uint8_t uart7_tx_data[HGUIDE_BUFFER_SIZE];
__attribute__ ((section(".buffer"))) volatile uint8_t usart3_rx_data[LOG_BUFFER_SIZE];
__attribute__ ((section(".buffer"))) volatile uint8_t usart3_tx_data[LOG_BUFFER_SIZE];

volatile uint8_t usart3_tx_finished = 0;
volatile uint8_t usart3_rx_finished = 0;
volatile uint8_t uart8_tx_finished = 0;
volatile uint8_t uart8_rx_finished = 0;
volatile uint8_t uart7_tx_finished = 0;
volatile uint8_t uart7_rx_finished = 0;

int main(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;                                    // enable GPIOD clock
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;                                    // enable GPIOB clock
    RCC->APB1LENR |= RCC_APB1LENR_USART3EN;                                 // enable USART3 clock
    RCC->APB1LENR |= RCC_APB1LENR_UART8EN;
    RCC->APB1LENR |= RCC_APB1LENR_UART7EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;                                     // enable DMA1 clock

    GPIOD->MODER &= ~(GPIO_MODER_MODE8);
    GPIOD->MODER &= ~(GPIO_MODER_MODE9);

    GPIOE->MODER &= ~(GPIO_MODER_MODE0);
    GPIOE->MODER &= ~(GPIO_MODER_MODE1);

    GPIOB->MODER &= ~(GPIO_MODER_MODE3);                                    // reset PB3
    GPIOB->MODER &= ~(GPIO_MODER_MODE4);                                    // reset PB4

    GPIOB->MODER &= ~(GPIO_MODER_MODE0);

    GPIOD->MODER |= GPIO_MODER_MODE8_1;                                     // set PD8 to AF mode
    GPIOD->MODER |= GPIO_MODER_MODE9_1;                                     // set PD9 to AF mode

    GPIOE->MODER |= GPIO_MODER_MODE0_1;
    GPIOE->MODER |= GPIO_MODER_MODE1_1;

    GPIOB->MODER |= GPIO_MODER_MODE3_1;                                     // set PB3 to AF mode
    GPIOB->MODER |= GPIO_MODER_MODE4_1;                                     // set PB4 to AF mode

    GPIOB->MODER |= GPIO_MODER_MODE0_0;

    GPIOD->AFR[1] |= (AF07 << 0);                                           // set PD8 to AF7 (USART3_TX)
    GPIOD->AFR[1] |= (AF07 << 4);                                           // set PD9 to AF8 (USART3_RX)

    GPIOE->AFR[0] |= (AF08 << 0);
    GPIOE->AFR[0] |= (AF08 << 4);

    GPIOB->AFR[0] |= (AF11 << 4 * 3);                                       // set PB3 to AF11 (UART7_RX)
    GPIOB->AFR[0] |= (AF11 << 4 * 4);                                       // set PB4 to AF11 (UART7_TX)

    USART3->BRR = 0x0010;
    USART3->CR1 = 0;
    USART3->CR3 |= (USART_CR3_DMAT) | (USART_CR3_DMAR);
    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);

    UART8->BRR = 0x0683;
    UART8->CR1 = 0;
    UART8->CR3 |= (USART_CR3_DMAT) | (USART_CR3_DMAR);
    UART8->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);

    UART7->BRR = 0x0045;
    UART7->CR1 = 0;
    UART7->CR3 |= (USART_CR3_DMAT) | (USART_CR3_DMAR);
    UART7->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);


    DMA1_Stream1->CR &= ~(DMA_SxCR_EN);
    while((DMA1_Stream1->CR & (DMA_SxCR_EN)));

    DMA1_Stream3->CR &=~(DMA_SxCR_EN);
	while((DMA1_Stream3->CR &(DMA_SxCR_EN)));

    DMA1_Stream1->CR |= ((DMA_SxCR_MINC)
                        |(DMA_SxCR_TCIE)
                        |(DMA_SxCR_CIRC));

    DMA1_Stream3->CR |= ((DMA_SxCR_MINC)
                        |(DMA_SxCR_DIR_0)
                        |(DMA_SxCR_TCIE)
                        |(DMA_SxCR_TRBUFF));

    DMA1_Stream1->PAR = (uint32_t) &USART3->RDR;
    DMA1_Stream3->PAR = (uint32_t) &USART3->TDR;

    DMA1_Stream0->CR &= ~(DMA_SxCR_EN);
    while((DMA1_Stream0->CR & (DMA_SxCR_EN)));

    DMA1_Stream4->CR &= ~(DMA_SxCR_EN);
    while((DMA1_Stream4->CR & (DMA_SxCR_EN)));

    DMA1_Stream0->CR |= ((DMA_SxCR_MINC)
                        |(DMA_SxCR_TCIE)
                        |(DMA_SxCR_CIRC));

    DMA1_Stream4->CR |= ((DMA_SxCR_MINC)
                        |(DMA_SxCR_DIR_0)
                        |(DMA_SxCR_TCIE)
                        |(DMA_SxCR_TRBUFF));


    DMA1_Stream0->PAR = (uint32_t) &UART8->RDR;
    DMA1_Stream4->PAR = (uint32_t) &UART8->TDR;

    DMA1_Stream2->CR &= ~(DMA_SxCR_EN);
    while((DMA1_Stream2->CR & (DMA_SxCR_EN)));

    DMA1_Stream2->CR |= ((DMA_SxCR_MINC)
                        |(DMA_SxCR_TCIE)
                        |(DMA_SxCR_CIRC));


    DMA1_Stream2->PAR = (uint32_t) &UART7->RDR;

    LL_DMA_SetPeriphRequest(DMA1, 3, 46U);
    LL_DMA_SetPeriphRequest(DMA1, 1, 45U);
    LL_DMA_SetPeriphRequest(DMA1, 0, 81U);
    LL_DMA_SetPeriphRequest(DMA1, 4, 82U);
    LL_DMA_SetPeriphRequest(DMA1, 2, 79U);

    GPS_t _gps;
    GPS_t *gps = &_gps;

    HGuidei300Imu_t _hguide_imu;
    HGuidei300Imu_t *hguide_imu = &_hguide_imu;

    uint8_t empty_gps_data[2048] = {[0 ... 2047] = 0};
    uint8_t empty_hguide_imu_data[2048] = {[0 ... 2047] = 0};

    RingBuffer_t _gps_data;
    RingBuffer_t *gps_data = &_gps_data;

    RingBuffer_t _hguide_imu_data;
    RingBuffer_t *hguide_imu_data = &_hguide_imu_data;

    RingBuffer_Init(gps_data, empty_gps_data, SIZE(empty_gps_data));
    RingBuffer_Init(hguide_imu_data, empty_hguide_imu_data, SIZE(empty_hguide_imu_data));

    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    UART8_DMA1_Stream0_Read(uart8_rx_data, GPS_BUFFER_SIZE);
    UART7_DMA1_Stream2_Read(uart7_rx_data, HGUIDE_BUFFER_SIZE);

    while(1)
    {
        if (Is_UART8_Buffer_Full())
        {
            RingBuffer_Put(gps_data, (uint8_t *) uart8_rx_data, SIZE(uart8_rx_data));
            GPS_ProcessData(gps, gps_data);

            if (gps->positioning_mode != 'A')
            {
                printf("Fix mode: %c\n", gps->positioning_mode);
                continue;
            }

            printf("Latitude: %lf N/S: %c Longitude: %lf E/W: %c",
                    gps->latitude, gps->north_south_indicator, gps->longitude, gps->east_west_indicator);
        }

        if (Is_UART7_Buffer_Full())
        {
            RingBuffer_Put(hguide_imu_data, (uint8_t *) uart7_rx_data, SIZE(uart7_rx_data));
            ProcessHGuidei300(hguide_imu, hguide_imu_data);

            printf("%lf,%lf,%lf\n", GetLinearAccelerationX(hguide_imu), GetLinearAccelerationY(hguide_imu), GetLinearAccelerationZ(hguide_imu));
        }
    }
}

void USART3_DMA1_Stream3_Write(volatile uint8_t *data, uint16_t length)
{
    DMA1_Stream3->M0AR = (uint32_t) data;
    DMA1_Stream3->NDTR = length;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
    while (usart3_tx_finished == 0);
    usart3_tx_finished = 0;
}


void USART3_DMA1_Stream1_Read(volatile uint8_t *buffer, uint16_t length)
{
    DMA1_Stream1->M0AR = (uint32_t) buffer;
    DMA1_Stream1->NDTR = length;
    DMA1_Stream1->CR |= DMA_SxCR_EN;
}

void UART8_DMA1_Stream4_Write(volatile uint8_t *data, uint16_t length)
{
    DMA1_Stream4->M0AR = (uint32_t) data;
    DMA1_Stream4->NDTR = length;
    DMA1_Stream4->CR |= DMA_SxCR_EN;
    while (uart8_tx_finished == 0);
    uart8_tx_finished = 0;
}

void UART8_DMA1_Stream0_Read(volatile uint8_t *buffer, uint16_t length)
{
    DMA1_Stream0->M0AR = (uint32_t) buffer;
    DMA1_Stream0->NDTR = length;
    DMA1_Stream0->CR |= DMA_SxCR_EN;
}

void UART7_DMA1_Stream2_Read(volatile uint8_t *buffer, uint16_t length)
{
    DMA1_Stream2->M0AR = (uint32_t) buffer;
    DMA1_Stream2->NDTR = length;
    DMA1_Stream2->CR |= DMA_SxCR_EN;
}



uint8_t Is_USART3_Buffer_Full(void)
{
    if (usart3_rx_finished == 0)
    {
        return BUFFER_EMPTY;
    }
    else
    {
        usart3_rx_finished = 0;
        return BUFFER_FULL;
    }
}

uint8_t Is_UART8_Buffer_Full(void)
{
    if (uart8_rx_finished == 0)
    {
        return BUFFER_EMPTY;
    }
    else
    {
        uart8_rx_finished = 0;
        return BUFFER_FULL;
    }
}

uint8_t Is_UART7_Buffer_Full(void)
{
    if (uart7_rx_finished == 0)
    {
        return BUFFER_EMPTY;
    }
    else
    {
        uart7_rx_finished = 0;
        return BUFFER_FULL;
    }
}

void _putchar(char character)
{
    usart3_tx_data[0] = (uint8_t) character;
    usart3_tx_data[1] = '\0';
    USART3_DMA1_Stream3_Write((uint8_t *) usart3_tx_data, 1);
}

