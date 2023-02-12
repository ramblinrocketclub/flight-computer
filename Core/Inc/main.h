#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

void GPIO_Init(void);
void UART_Init(void);
void DMA_Init(void);

void USART3_DMA1_Stream3_Write(volatile uint8_t *data, uint16_t length);
void USART3_DMA1_Stream1_Read(volatile uint8_t *buffer, uint16_t length);
void UART9_DMA1_Stream4_Write(volatile uint8_t *data, uint16_t length);
void UART9_DMA1_Stream0_Read(volatile uint8_t *buffer, uint16_t length);
void UART7_DMA1_Stream2_Read(volatile uint8_t *buffer, uint16_t length);

uint8_t Is_USART3_Buffer_Full(void);
uint8_t Is_UART9_Buffer_Full(void);
uint8_t Is_UART7_Buffer_Full(void);

__attribute__ ((section(".buffer"))) extern volatile uint8_t uart9_rx_data[];
__attribute__ ((section(".buffer"))) extern volatile uint8_t uart9_tx_data[];
__attribute__ ((section(".buffer"))) extern volatile uint8_t uart7_rx_data[];
__attribute__ ((section(".buffer"))) extern volatile uint8_t uart7_tx_data[];
__attribute__ ((section(".buffer"))) extern volatile uint8_t usart3_rx_data[];
__attribute__ ((section(".buffer"))) extern volatile uint8_t usart3_tx_data[];

extern volatile uint8_t usart3_tx_finished;
extern volatile uint8_t usart3_rx_finished;
extern volatile uint8_t uart9_tx_finished;
extern volatile uint8_t uart9_rx_finished;
extern volatile uint8_t uart7_tx_finished;
extern volatile uint8_t uart7_rx_finished;

extern TaskHandle_t hguide_imu_processing_task_handle;
extern TaskHandle_t gps_processing_task_handle;

#endif /* MAIN_H */

