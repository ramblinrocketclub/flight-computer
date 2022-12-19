#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "main.h"
#include "stm32h7xx.h"
#include "stm32h7xx_ll_dma.h"
#include "ringbuffer.h"
#include "gps.h"

#include "log.h"

#define AF07                (0x7UL)
#define AF08                (0x8UL)

#define BUFFER_EMPTY        (0x0UL)
#define BUFFER_FULL         (0x1UL)

#define GPS_BUF_SIZE        512

#define SIZE(array)         (sizeof(array) / sizeof(array[0]))

void USART3_DMA1_Stream3_Write(uint8_t *data, uint16_t length);
void USART3_DMA1_Stream1_Read(uint8_t *buffer, uint16_t length);
void UART8_DMA1_Stream4_Write(uint8_t *data, uint16_t length);
void UART8_DMA1_Stream0_Read(uint8_t *buffer, uint16_t length);
void UART7_DMA1_Stream1_Read(uint8_t *buffer, uint16_t length);

uint8_t Is_USART3_Buffer_Full(void);
uint8_t Is_UART8_Buffer_Full(void);
uint8_t Is_UART7_Buffer_Full(void);

uint8_t usart3_tx_finished = 0;
uint8_t usart3_rx_finished = 0;
uint8_t uart8_tx_finished = 0;
uint8_t uart8_rx_finished = 0;
uint8_t uart7_tx_finished = 0;
uint8_t uart7_rx_finished = 0;


__attribute__ ((section(".buffer"), used)) uint8_t uart8_rx_data[GPS_BUF_SIZE];
__attribute__ ((section(".buffer"), used)) uint8_t uart8_tx_data[GPS_BUF_SIZE];
__attribute__ ((section(".buffer"), used)) uint8_t uart7_rx_data[1024];


int main(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;                                    // enable GPIOD clock
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    RCC->APB1LENR |= RCC_APB1LENR_USART3EN;                                 // enable USART3 clock
    RCC->APB1LENR |= RCC_APB1LENR_UART8EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;                                     // enable DMA1 clock

    GPIOD->MODER &= ~(GPIO_MODER_MODE8);
    GPIOD->MODER &= ~(GPIO_MODER_MODE9);

    GPIOE->MODER &= ~(GPIO_MODER_MODE0);
    GPIOE->MODER &= ~(GPIO_MODER_MODE1);

    GPIOB->MODER &= ~(GPIO_MODER_MODE0);

    GPIOD->MODER |= GPIO_MODER_MODE8_1;                                     // set PD8 to AF mode
    GPIOD->MODER |= GPIO_MODER_MODE9_1;                                     // set PD9 to AF mode

    GPIOE->MODER |= GPIO_MODER_MODE0_1;
    GPIOE->MODER |= GPIO_MODER_MODE1_1;

    GPIOB->MODER |= GPIO_MODER_MODE0_0;

    GPIOD->AFR[1] |= (AF07 << 0);                                           // set PD8 to AF7 (USART3_TX)
    GPIOD->AFR[1] |= (AF07 << 4);                                           // set PD9 to AF8 (USART3_RX)

    GPIOE->AFR[0] |= (AF08 << 0);
    GPIOE->AFR[0] |= (AF08 << 4);

    USART3->BRR = 0x0683;
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

    GPS_Handle gps_struct;
    GPS_Handle *gps = &gps_struct;

    uint8_t empty[2048] = {[0 ... 2047] = 0};

    ringbuf_t buffer;
    ringbuf_t *buf = &buffer;

    ringbuf_init(buf, empty, SIZE(empty));

    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    UART8_DMA1_Stream0_Read(uart8_rx_data, GPS_BUF_SIZE);

    LOG_INFO("Initialization successful %d", 123);

    while(1)
    {
        if (Is_UART8_Buffer_Full())
        {
            for (int i = 0; i < SIZE(uart8_rx_data); i++)
                ringbuf_put(buf, uart8_rx_data[i]);

            parse_nmea_packet(gps, buf, 1, "GNGLL");

            if (gps->gll.pos_mode != 'A')
            {
                sprintf((char *) uart8_tx_data, "%c", gps->gll.pos_mode);
                USART3_DMA1_Stream3_Write((uint8_t *) "Fix mode: ", strlen((char *) "Fix mode: "));
                USART3_DMA1_Stream3_Write((uint8_t *) uart8_tx_data, strlen((char *) uart8_tx_data));
                USART3_DMA1_Stream3_Write((uint8_t *) "\n", strlen((char *) "\n"));
                continue;
            }

            sprintf((char *) uart8_tx_data, "%lf", gps->gll.lat);
            USART3_DMA1_Stream3_Write((uint8_t *) "Latitude: ", strlen((char *) "Latitude: "));
            USART3_DMA1_Stream3_Write((uint8_t *) uart8_tx_data, strlen((char *) uart8_tx_data));

            USART3_DMA1_Stream3_Write((uint8_t *) " ", strlen((char *) " "));

            sprintf((char *) uart8_tx_data, "%c", gps->gll.ns);
            USART3_DMA1_Stream3_Write((uint8_t *) "N/S: ", strlen((char *) "N/S: "));
            USART3_DMA1_Stream3_Write((uint8_t *) uart8_tx_data, strlen((char *) uart8_tx_data));

            USART3_DMA1_Stream3_Write((uint8_t *) " ", strlen((char *) " "));

            sprintf((char *) uart8_tx_data, "%lf", gps->gll.lon);
            USART3_DMA1_Stream3_Write((uint8_t *) "Longitude: ", strlen((char *) "Longitude: "));
            USART3_DMA1_Stream3_Write((uint8_t *) uart8_tx_data, strlen((char *) uart8_tx_data));

            USART3_DMA1_Stream3_Write((uint8_t *) " ", strlen((char *) " "));

            sprintf((char *) uart8_tx_data, "%c", gps->gll.ew);
            USART3_DMA1_Stream3_Write((uint8_t *) "E/W: ", strlen((char *) "E/W: "));
            USART3_DMA1_Stream3_Write((uint8_t *) uart8_tx_data, strlen((char *) uart8_tx_data));

            USART3_DMA1_Stream3_Write((uint8_t *) " ", strlen((char *) " "));

            sprintf((char *) uart8_tx_data, "%ld", gps->gll.checksum);
            USART3_DMA1_Stream3_Write((uint8_t *) "Checksum: ", strlen((char *) "Checksum: "));
            USART3_DMA1_Stream3_Write((uint8_t *) uart8_tx_data, strlen((char *) uart8_tx_data));

            USART3_DMA1_Stream3_Write((uint8_t *) "\n", strlen((char *) "\n"));
        }
	}
}

void USART3_DMA1_Stream3_Write(uint8_t *data, uint16_t length)
{
    DMA1_Stream3->M0AR = (uint32_t) data;
    DMA1_Stream3->NDTR = length;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
    while (usart3_tx_finished == 0);
    usart3_tx_finished = 0;
}


void USART3_DMA1_Stream1_Read(uint8_t *buffer, uint16_t length)
{
    DMA1_Stream1->M0AR = (uint32_t) buffer;
    DMA1_Stream1->NDTR = length;
    DMA1_Stream1->CR |= DMA_SxCR_EN;
}

void UART8_DMA1_Stream4_Write(uint8_t *data, uint16_t length)
{
    DMA1_Stream4->M0AR = (uint32_t) data;
    DMA1_Stream4->NDTR = length;
    DMA1_Stream4->CR |= DMA_SxCR_EN;
    while (uart8_tx_finished == 0);
    uart8_tx_finished = 0;
}

void UART8_DMA1_Stream0_Read(uint8_t *buffer, uint16_t length)
{
    DMA1_Stream0->M0AR = (uint32_t) buffer;
    DMA1_Stream0->NDTR = length;
    DMA1_Stream0->CR |= DMA_SxCR_EN;
}

void UART7_DMA1_Stream2_Read(uint8_t *buffer, uint16_t length)
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

