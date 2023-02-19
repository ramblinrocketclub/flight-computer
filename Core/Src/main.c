#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <limits.h>
#include "main.h"
#include "stm32h7xx.h"
#include "stm32h7xx_ll_dma.h"
#include "gps.h"
#include "hguide_imu.h"
#include "printf.h"
#include "rtc.h"
#include "printf.h"
#include "rocket.h"
#include "states/flight_state_variables.h"
#include "kalman_filter.h"

#define AF07                    7UL
#define AF08                    8UL
#define AF11                    11UL

#define BUFFER_EMPTY            0UL
#define BUFFER_FULL             1UL

#define GPS_BUFFER_SIZE         512UL
#define HGUIDE_BUFFER_SIZE      512UL
#define LOG_BUFFER_SIZE         512UL

#define SIZE(array)         (sizeof(array) / sizeof(array[0]))

__attribute__ ((section(".buffer"))) volatile uint8_t uart9_rx_data[GPS_BUFFER_SIZE];
__attribute__ ((section(".buffer"))) volatile uint8_t uart9_tx_data[GPS_BUFFER_SIZE];
__attribute__ ((section(".buffer"))) volatile uint8_t uart7_rx_data[HGUIDE_BUFFER_SIZE];
__attribute__ ((section(".buffer"))) volatile uint8_t uart7_tx_data[HGUIDE_BUFFER_SIZE];
__attribute__ ((section(".buffer"))) volatile uint8_t usart3_rx_data[LOG_BUFFER_SIZE];
__attribute__ ((section(".buffer"))) volatile uint8_t usart3_tx_data[LOG_BUFFER_SIZE];

volatile uint8_t usart3_tx_finished = 0;
volatile uint8_t usart3_rx_finished = 0;
volatile uint8_t uart9_tx_finished = 0;
volatile uint8_t uart9_rx_finished = 0;
volatile uint8_t uart7_tx_finished = 0;
volatile uint8_t uart7_rx_finished = 0;

Rocket rocket;
RTC_Time_t rtc_time;

double initialize_timestamp = 0;

TaskHandle_t hguide_imu_processing_task_handle = NULL;
TaskHandle_t gps_processing_task_handle = NULL;

void HGuideIMUProcessingTask(void *parameters)
{
    HGuideIMU_t hguide_imu;
    uint8_t empty_hguide_imu_data[2048] = {[0 ... 2047] = 0};
    RingBuffer_t hguide_imu_data;
    RingBuffer_Init(&hguide_imu_data, empty_hguide_imu_data, SIZE(empty_hguide_imu_data));

    for (;;)
    {
        vTaskSuspend(NULL);

        taskENTER_CRITICAL();

        RingBuffer_Put(&hguide_imu_data, (uint8_t *) uart7_rx_data, SIZE(uart7_rx_data));
        ProcessHGuidei300(&hguide_imu, &hguide_imu_data);

        double current_timestamp = RTC_GetTimestamp(&rtc_time);

        // sprintf((char *) usart3_tx_data, "%lf, %lf, %lf\r\n", GetLinearAccelerationXMsec2(&hguide_imu),
        //                                                         GetLinearAccelerationYMsec2(&hguide_imu),
        //                                                         GetLinearAccelerationZMsec2(&hguide_imu));

        USART3_DMA1_Stream3_Write((uint8_t *) usart3_tx_data, strlen((char *) usart3_tx_data));

        // New IMU data has arrived
        if (!rocket.has_calibrated) {
            // If it has been more than 0.5 seconds since initialization, calibrate IMU
            if (current_timestamp - initialize_timestamp > 0.5) {
                calibrate_rocket(&rocket, &hguide_imu);

                sprintf((char *) usart3_tx_data, "Calibration vector: %lf, %lf, %lf\r\n", GetLinearAccelerationXMsec2(&hguide_imu),
                                                        GetLinearAccelerationYMsec2(&hguide_imu),
                                                        GetLinearAccelerationZMsec2(&hguide_imu));

                sprintf((char *) usart3_tx_data, "Finished calibrating\r\n");

                USART3_DMA1_Stream3_Write((uint8_t *) usart3_tx_data, strlen((char *) usart3_tx_data));

                print_matrix(&rocket.hguide_local_to_world_3x3, "Calibration matrix: ");
            }
        } else {
            update_rocket_state_variables(&rocket, current_timestamp, &hguide_imu, NULL);

            sprintf((char *) usart3_tx_data, "%lf\r\n", get_vertical_accel_msec2(&rocket.fsv));
            USART3_DMA1_Stream3_Write((uint8_t *) usart3_tx_data, strlen((char *) usart3_tx_data));
        }

        taskEXIT_CRITICAL();
    }
}

void GPSProcessingTask(void *parameters)
{
    GPS_t gps;
    uint8_t empty_gps_data[2048] = {[0 ... 2047] = 0};
    RingBuffer_t gps_data;
    RingBuffer_Init(&gps_data, empty_gps_data, SIZE(empty_gps_data));

    for (;;)
    {
        vTaskSuspend(NULL);

        taskENTER_CRITICAL();

        RingBuffer_Put(&gps_data, (uint8_t *) uart9_rx_data, SIZE(uart9_rx_data));
        GPS_ProcessData(&gps, &gps_data);


        if (GPS_GetPositioningMode(&gps) != 'A')
        {
            sprintf((char *) usart3_tx_data, "Fix mode: %c\n", GPS_GetPositioningMode(&gps));
            USART3_DMA1_Stream3_Write((uint8_t *) usart3_tx_data, strlen((char *) usart3_tx_data));
        }

        else
        {
            sprintf((char *) usart3_tx_data, "Latitude: %lf N/S: %c Longitude: %lf E/W: %c", GPS_GetLatitude(&gps), GPS_GetNorthSouthIndicator(&gps), GPS_GetLongitude(&gps), GPS_GetEastWestIndicator(&gps));
            USART3_DMA1_Stream3_Write((uint8_t *) usart3_tx_data, strlen((char *) usart3_tx_data));
        }

        taskEXIT_CRITICAL();
    }
}

int main(void)
{
    GPIO_Init();
    UART_Init();
    DMA_Init();
    RTC_Init();

    GPS_t gps;

    gps.altitude_meters = 0;

    // Init rocket
    init_rocket(&rocket, &gps);

    xTaskCreate(
        HGuideIMUProcessingTask,                /* Function that implements the task. */
        "HGuideIMUProcessing",                  /* Text name for the task. */
        1024,                                   /* Stack size in words, not bytes. */
        NULL,                                   /* Parameter passed into the task. */
        tskIDLE_PRIORITY,                       /* Priority at which the task is created. */
        &hguide_imu_processing_task_handle);    /* Used to pass out the created task's handle. */

    xTaskCreate(
        GPSProcessingTask,              /* Function that implements the task. */
        "GPSProcessing",                /* Text name for the task. */
        1024,                           /* Stack size in words, not bytes. */
        NULL,                           /* Parameter passed into the task. */
        tskIDLE_PRIORITY,               /* Priority at which the task is created. */
        &gps_processing_task_handle);   /* Used to pass out the created task's handle. */

    UART9_DMA1_Stream0_Read(uart9_rx_data, GPS_BUFFER_SIZE);
    UART7_DMA1_Stream2_Read(uart7_rx_data, HGUIDE_BUFFER_SIZE);

    vTaskStartScheduler();

    while(1)
    {

    }
}

void GPIO_Init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;    // Enable GPIOD clock
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;    // Enable GPIOE clock
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;    // Enable GPIOB clock
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN;    // Enable GPIOG clock

    GPIOD->MODER &= ~(GPIO_MODER_MODE8);    // Reset mode of PD8
    GPIOD->MODER &= ~(GPIO_MODER_MODE9);    // Reset mode of PD9

    GPIOB->MODER &= ~(GPIO_MODER_MODE3);    // Reset mode of PB3
    GPIOB->MODER &= ~(GPIO_MODER_MODE4);    // Reset mode of PB4

    GPIOG->MODER &= ~(GPIO_MODER_MODE0);
    GPIOG->MODER &= ~(GPIO_MODER_MODE1);

    GPIOD->MODER |= GPIO_MODER_MODE8_1;     // Set PD8 to AF mode
    GPIOD->MODER |= GPIO_MODER_MODE9_1;     // Set PD9 to AF mode

    GPIOB->MODER |= GPIO_MODER_MODE3_1;     // set PB3 to AF mode
    GPIOB->MODER |= GPIO_MODER_MODE4_1;     // set PB4 to AF mode

    GPIOG->MODER |= GPIO_MODER_MODE0_1;     // Set PG0 to AF mode
    GPIOG->MODER |= GPIO_MODER_MODE1_1;     // Set PG1 to AF mode

    GPIOD->AFR[1] |= (AF07 << 0);           // Set PD8 to AF7 (USART3_TX)
    GPIOD->AFR[1] |= (AF07 << 4);           // Set PD9 to AF8 (USART3_RX)

    GPIOG->AFR[0] |= (AF11 << 4 * 0);       // Set PG0 to AF11 (UART9_RX)
    GPIOG->AFR[0] |= (AF11 << 4 * 1);       // Set PG1 to AF11 (UART9_TX)

    GPIOB->AFR[0] |= (AF11 << 4 * 3);       // set PB3 to AF11 (UART7_RX)
    GPIOB->AFR[0] |= (AF11 << 4 * 4);       // set PB4 to AF11 (UART7_TX)
}

void UART_Init(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_USART3EN;                         // Enable USART3 clock
    RCC->APB2ENR |= RCC_APB2ENR_UART9EN;                          // Enable uart9 clock
    RCC->APB1LENR |= RCC_APB1LENR_UART7EN;                          // Enable UART7 clock

    USART3->BRR = 0x0010;                                           // Set baud rate to 4MBd
    USART3->CR1 = 0;                                                // Reset CR1 register
    USART3->CR3 |= (USART_CR3_DMAT) | (USART_CR3_DMAR);             // Enable DMA transmit and receive
    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);    // Enable uart with transmit and receive

    UART9->BRR = 0x0683;                                            // Set baud rate to 38400 Bd
    UART9->CR1 = 0;                                                 // Reset CR1 register
    UART9->CR3 |= (USART_CR3_DMAT) | (USART_CR3_DMAR);              // Enable DMA transmit and receive
    UART9->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);     // Enable uart with transmit and receive

    UART7->BRR = 0x0045;                                            // Set baud rate of 921.6 kBd
    UART7->CR1 = 0;                                                 // Reset CR1 register
    UART7->CR3 |= (USART_CR3_DMAT) | (USART_CR3_DMAR);              // Enable DMA transmit and receive
    UART7->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);     // Enable uart with transmit and receive
}

void DMA_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

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


    DMA1_Stream0->PAR = (uint32_t) &UART9->RDR;
    DMA1_Stream4->PAR = (uint32_t) &UART9->TDR;

    DMA1_Stream2->CR &= ~(DMA_SxCR_EN);
    while((DMA1_Stream2->CR & (DMA_SxCR_EN)));

    DMA1_Stream2->CR |= ((DMA_SxCR_MINC)
                        |(DMA_SxCR_TCIE)
                        |(DMA_SxCR_CIRC));


    DMA1_Stream2->PAR = (uint32_t) &UART7->RDR;

    LL_DMA_SetPeriphRequest(DMA1, 0, 116U);
    LL_DMA_SetPeriphRequest(DMA1, 1, 45U);
    LL_DMA_SetPeriphRequest(DMA1, 2, 79U);
    LL_DMA_SetPeriphRequest(DMA1, 3, 46U);
    LL_DMA_SetPeriphRequest(DMA1, 4, 117U);

    NVIC_SetPriority(DMA1_Stream0_IRQn, 5);
    NVIC_SetPriority(DMA1_Stream1_IRQn, 5);
    NVIC_SetPriority(DMA1_Stream2_IRQn, 5);
    NVIC_SetPriority(DMA1_Stream3_IRQn, 4);
    NVIC_SetPriority(DMA1_Stream4_IRQn, 5);

    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
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

void UART9_DMA1_Stream4_Write(volatile uint8_t *data, uint16_t length)
{
    DMA1_Stream4->M0AR = (uint32_t) data;
    DMA1_Stream4->NDTR = length;
    DMA1_Stream4->CR |= DMA_SxCR_EN;
    while (uart9_tx_finished == 0);
    uart9_tx_finished = 0;
}

void UART9_DMA1_Stream0_Read(volatile uint8_t *buffer, uint16_t length)
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

uint8_t Is_UART9_Buffer_Full(void)
{
    if (uart9_rx_finished == 0)
    {
        return BUFFER_EMPTY;
    }
    else
    {
        uart9_rx_finished = 0;
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

