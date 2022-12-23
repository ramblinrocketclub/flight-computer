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
#include "kalman_filter.h"

#define AF07                (0x7UL)
#define AF08                (0x8UL)

#define BUFFER_EMPTY        (0x0UL)
#define BUFFER_FULL         (0x1UL)

#define GPS_BUF_SIZE        512

#define DT_SECONDS 0.02

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

void print_matrix(arm_matrix_instance_f32 *A, const char *name) {
	int row;
	int col;

	LOG_INFO("%s Matrix:", name);

	_LOG_INFO("[");

	for (row = 0; row < A->numRows; row++) {
		if (row > 0) {
			_LOG_INFO(" ");
		}

		_LOG_INFO("[");

		for (col = 0; col < A->numCols; col++) {
			if (col < A->numCols - 1) {
				_LOG_INFO("% 15.5f, ", A->pData[row * A->numCols + col]);
			} else {
				_LOG_INFO("% 15.5f", A->pData[row * A->numCols + col]);
			}
		}

		if (row < A->numRows - 1) {
			LOG_INFO("],");
		} else {
			_LOG_INFO("]");
		}
	}

	LOG_INFO("]");
}

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

    LOG_INFO("Hardware initialization successful %d", 69420);

	// Basic kalman filter for constant accelerating body
	KalmanFilter kf;
	arm_status status = ARM_MATH_SUCCESS;

	const float accelStd = 0.1;
	const float accelVar = accelStd * accelStd;

	float32_t F_f32[4] = {
		1, DT_SECONDS,
		0, 1
	};

	float32_t G_f32[2] = {
		0.5 * DT_SECONDS * DT_SECONDS,
		DT_SECONDS
	};

	float32_t P_f32[4] = {
			500, 0,
			0, 500
	};

	float32_t Q_f32[4] = {
			1.0, 1.0,
			1.0, 1.0
	};

	float32_t xHat_f32[2] = {
		// Position, velocity
		0, 0
	};

	float32_t stateStdDevs_f32[2] = {
		// Position std, velocity std
		DT_SECONDS * DT_SECONDS / 2.0 * accelVar, DT_SECONDS * accelVar
	};

	status = init_kalman_filter(&kf, 2, 1, &F_f32[0], &G_f32[0], &P_f32[0], &Q_f32[0], &xHat_f32[0], &stateStdDevs_f32[0]);

	print_matrix(&kf.Q, "Q");
	LOG_INFO("Kalman initialization status: %d", status);

	float32_t un_f32[1] = { 9.8 };

	status = predict_kalman_filter(&kf, un_f32);

	print_matrix(&kf.xHat, "xHat");
	print_matrix(&kf.P, "P");
	LOG_INFO("Step 1 predict status: %d", status);

	float32_t zn_f32[1] = { -32.40 };
	float32_t H_f32[2] = { 1, 0 };
	float32_t measurementStdDevs[1] = { 20 };

	status = correct_kalman_filter(&kf, 1, zn_f32, H_f32, measurementStdDevs);

	print_matrix(&kf.xHat, "xHat");
	print_matrix(&kf.P, "P");
	LOG_INFO("Step 1 correct status: %d", status);

	un_f32[0] = 39.72 - 9.8;

	status = predict_kalman_filter(&kf, un_f32);

	print_matrix(&kf.xHat, "xHat");
	print_matrix(&kf.P, "P");
	LOG_INFO("Step 2 predict status: %d", status);

	zn_f32[0] = -11.1;

	status = correct_kalman_filter(&kf, 1, zn_f32, H_f32, measurementStdDevs);

	print_matrix(&kf.xHat, "xHat");
	print_matrix(&kf.P, "P");
	LOG_INFO("Step 2 correct status: %d", status);

    while(1)
    {
        if (Is_UART8_Buffer_Full())
        {
            for (int i = 0; i < SIZE(uart8_rx_data); i++)
                ringbuf_put(buf, uart8_rx_data[i]);

            parse_nmea_packet(gps, buf, 1, "GNGLL");

            if (gps->gll.pos_mode != 'A')
            {
                LOG_INFO("Fix mode: %c", gps->gll.pos_mode);
                continue;
            }

            LOG_INFO("Latitude: %lf N/S: %c Longitude: %lf E/W: %c Checksum: %ld", 
                    gps->gll.lat, gps->gll.ns, gps->gll.lon, gps->gll.ew, gps->gll.checksum);
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

