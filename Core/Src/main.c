#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "stm32h7xx.h"
#include "stm32h7xx_ll_dma.h"
#include "ringbuffer.h"
#include "gps.h"
#include "hguide_imu.h"
#include "printf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "kalman_filter.h"
#include "rtc.h"
#include "state_machine.h"
#include "imu_math_helper.h"

#define GRAVITY_CONSTANT_MSEC2 9.81

#define AF07                (0x7UL)
#define AF08                (0x8UL)
#define AF11                (11UL)

#define BUFFER_EMPTY        (0x0UL)
#define BUFFER_FULL         (0x1UL)

#define GPS_BUF_SIZE        512
#define HGUIDE_BUF_SIZE     512
#define LOG_BUF_SIZE        512

#define DT_SECONDS 0.02

#define SIZE(array)         (sizeof(array) / sizeof(array[0]))

// Contains all of the information needed to make flight decisions
typedef struct FlightStateVariables {
    double verticalAcceleration;
    double verticalVelocity;
    double verticalPosition; // This position is wrt starting launch height
    double tiltRadians;
} FlightStateVariables;

__attribute__ ((section(".buffer"), used)) volatile uint8_t uart8_rx_data[GPS_BUF_SIZE];
__attribute__ ((section(".buffer"), used)) volatile uint8_t uart8_tx_data[GPS_BUF_SIZE];
__attribute__ ((section(".buffer"), used)) volatile uint8_t uart7_rx_data[HGUIDE_BUF_SIZE];
__attribute__ ((section(".buffer"), used)) volatile uint8_t uart7_tx_data[HGUIDE_BUF_SIZE];
__attribute__ ((section(".buffer"), used)) volatile uint8_t usart3_rx_data[LOG_BUF_SIZE];
__attribute__ ((section(".buffer"), used)) volatile uint8_t usart3_tx_data[LOG_BUF_SIZE];

volatile uint8_t usart3_tx_finished = 0;
volatile uint8_t usart3_rx_finished = 0;
volatile uint8_t uart8_tx_finished = 0;
volatile uint8_t uart8_rx_finished = 0;
volatile uint8_t uart7_tx_finished = 0;
volatile uint8_t uart7_rx_finished = 0;

// Buffer for holding time data
RTCTimeBuffer timeBuffer;

HGuidei300Imu_t hguide_i300_imu;
HGuidei300Imu_t *pHGuidei300Imu = &hguide_i300_imu;

GPS_Handle gps_struct;
GPS_Handle *gps = &gps_struct;

uint8_t empty1[2048] = {[0 ... 2047] = 0};
uint8_t empty2[2048] = {[0 ... 2047] = 0};

ringbuf_t buffer;
ringbuf_t *buf = &buffer;

RingBuffer_t imu_data;
RingBuffer_t *pHGuidei300ImuData = &imu_data;

float32_t locToWorld3x3_f32[9] = {0};
float32_t globalOrientation3x3_f32[9] = {0};
float32_t Axyz_local_f32[3] = {0};
float32_t Axyz_global_f32[3] = {0};

arm_matrix_instance_f32 locToWorld3x3;
Quaternion localOrientation;
arm_matrix_instance_f32 globalOrientation3x3;

arm_matrix_instance_f32 Axyz_local;
arm_matrix_instance_f32 Axyz_global;

// State machine states
State safe;
State armed;
State boost1;
State fast1;
State preStage;
State failedStaging;
State postStage;
State sustainerIgnition;
State failedSustainerIgnition;
State boost2;
State fast2;
State apogee;
State coast;
State chute;
State landed;

StateMachine stateMachine;

FlightStateVariables stateVariables;

KalmanFilter heightEstimator;

// Define filtering constants and matrices
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

void setup();
void calibrate();
void update_flight_state_variables();

void safe_initialize();
State *safe_execute();
void safe_finish();
void armed_initialize();
State *armed_execute();
void armed_finish();
void boost1_initialize();
State *boost1_execute();
void boost1_finish();
void fast1_initialize();
State *fast1_execute();
void fast1_finish();
void pre_stage_initialize();
State *pre_stage_execute();
void pre_stage_finish();
void failed_stage_initialize();
State *failed_stage_execute();
void failed_stage_finish();
void post_stage_initialize();
State *post_stage_execute();
void post_stage_finish();
void sustainer_ignition_initialize();
State *sustainer_ignition_execute();
void sustainer_ignition_finish();
void failed_sustainer_ignition_initialize();
State *failed_sustainer_ignition_execute();
void failed_sustainer_ignition_finish();
void boost2_initialize();
State *boost2_execute();
void boost2_finish();
void fast2_initialize();
State *fast2_execute();
void fast2_finish();
void apogee_initialize();
State *apogee_execute();
void apogee_finish();
void coast_initialize();
State *coast_execute();
void coast_finish();
void chute_initialize();
State *chute_execute();
void chute_finish();
void landed_initialize();
State *landed_execute();
void landed_finish();

void USART3_DMA1_Stream3_Write(volatile uint8_t *data, uint16_t length);
void USART3_DMA1_Stream1_Read(volatile uint8_t *buffer, uint16_t length);
void UART8_DMA1_Stream4_Write(volatile uint8_t *data, uint16_t length);
void UART8_DMA1_Stream0_Read(volatile uint8_t *buffer, uint16_t length);
void UART7_DMA1_Stream2_Read(volatile uint8_t *buffer, uint16_t length);
uint8_t Is_USART3_Buffer_Full(void);
uint8_t Is_UART8_Buffer_Full(void);
uint8_t Is_UART7_Buffer_Full(void);
void readRTCTimeBuffer(RTCTimeBuffer *buffer);
void _putchar(char character);

int main(void)
{
    setup();

    printf("Hardware initialization successful %d\n", 69420);

    while(1)
    {
        readRTCTimeBuffer(&timeBuffer);

        printf("Time: %02d.%02d.%02d\n", timeBuffer.hours, 
                                         timeBuffer.minutes, 
                                         timeBuffer.seconds);

        if (Is_UART8_Buffer_Full())
        {
            for (int i = 0; i < SIZE(uart8_rx_data); i++)
            {
                ringbuf_put(buf, uart8_rx_data[i]);
            }

            parse_nmea_packet(gps, buf, 1, "GNGLL");

            if (gps->gll.pos_mode != 'A')
            {
                printf("Fix mode: %c\n", gps->gll.pos_mode);
                continue;
            }

            printf("Latitude: %lf N/S: %c Longitude: %lf E/W: %c Checksum: %ld\n",
                    gps->gll.lat, gps->gll.ns, gps->gll.lon, gps->gll.ew, gps->gll.checksum);
        }

        if (Is_UART7_Buffer_Full())
        {
            RingBuffer_Put(pHGuidei300ImuData, (uint8_t *) uart7_rx_data, SIZE(uart7_rx_data));
            ProcessHGuidei300(pHGuidei300Imu, pHGuidei300ImuData);

            printf("%lf,%lf,%lf\n", GetLinearAccelerationX(pHGuidei300Imu), GetLinearAccelerationY(pHGuidei300Imu), GetLinearAccelerationZ(pHGuidei300Imu));
        }
    }
}

// Performs all hardware initialization
void setup() {
    // RTC config constants
    // All config constants are in BCD format
    static const uint8_t hours = 0x10;
    static const uint8_t minutes = 0x20;
    static const uint8_t seconds = 0x30;

    static const uint8_t year = 0x22;
    static const uint8_t month = RTC_MONTH_NOVEMBER;
    static const uint8_t date = 0x19;
    static const uint8_t weekday = RTC_WEEKDAY_SATURDAY;

    // Temporary registers
    uint32_t tmpreg;

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

    ringbuf_init(buf, empty1, SIZE(empty1));
    RingBuffer_Init(pHGuidei300ImuData, empty2, SIZE(empty2));

    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    UART8_DMA1_Stream0_Read(uart8_rx_data, GPS_BUF_SIZE);
    UART7_DMA1_Stream2_Read(uart7_rx_data, HGUIDE_BUF_SIZE);

    // Disable RTC write protection
    RTC->WPR = 0xCAU;
    RTC->WPR = 0x53U;

    // Set initialiation mode
    RTC->ISR = RTC_ISR_INIT_Msk;

    // Wait until initialization mode entered
    while ((RTC->ISR & RTC_ISR_INITF) == 0U);

    RTC->CR &= ~(RTC_CR_FMT | RTC_CR_OSEL | RTC_CR_POL);

    // Configure 24 hour time, no output, high output polarity
    RTC->CR |= RTC_CR_POL;

    // Asynchronous predivider = 127, Sychronous predivider = 1023
    RTC->PRER = (127 << RTC_PRER_PREDIV_A_Pos) | (1023 << RTC_PRER_PREDIV_S_Pos);

    tmpreg = (((uint32_t)(hours) << RTC_TR_HU_Pos)  | \
                ((uint32_t)(minutes) << RTC_TR_MNU_Pos) | \
                ((uint32_t)(seconds) << RTC_TR_SU_Pos)  | \
                ((uint32_t)(0x00U) << RTC_TR_PM_Pos));

    RTC->TR = (uint32_t)(tmpreg & RTC_TR_RESERVED_MASK);

    // No DST
    RTC->CR &= ((uint32_t)~RTC_CR_BKP);
    RTC->CR |= (uint32_t)(RTC_DAYLIGHTSAVING_NONE | RTC_STOREOPERATION_RESET);

    // Set date
    tmpreg = ((((uint32_t)year)    << RTC_DR_YU_Pos) | \
                  (((uint32_t)month)   << RTC_DR_MU_Pos) | \
                  (((uint32_t)date)    << RTC_DR_DU_Pos) | \
                  (((uint32_t)weekday) << RTC_DR_WDU_Pos));

    RTC->DR = (uint32_t)(tmpreg & RTC_DR_RESERVED_MASK);

    // Exit init mode
    RTC->ISR &= ~(RTC_ISR_INITF);

    if ((RTC_CR_BYPSHAD & RTC->CR) == 0U) {
        RTC->ISR &= (uint32_t)RTC_ISR_RSF_Msk;

        // Wait for register to be synchronized
        while ((RTC->ISR & RTC_ISR_RSF) == 0U);
    } else {
        RTC->CR &= ~(RTC_CR_BYPSHAD);

        RTC->ISR &= (uint32_t)RTC_ISR_RSF_Msk;

        // Wait for register to be synchronized
        while ((RTC->ISR & RTC_ISR_RSF) == 0U);

        RTC->CR |= RTC_CR_BYPSHAD;
    }

    RTC->OR &= ~(RTC_OR_ALARMOUTTYPE | RTC_OR_OUT_RMP);

    // Set output type to push-pull, output remap to config 0
    RTC->OR |= (RTC_OR_ALARMOUTTYPE);

    // Enable RTC write protection
    RTC->WPR = 0xFFU;

    // Set up state machine
    safe.initPtr = &safe_initialize;
    safe.executePtr = &safe_execute;
    safe.finishPtr = &safe_finish;

    armed.initPtr = &armed_initialize;
    armed.executePtr = &armed_execute;
    armed.finishPtr = &armed_finish;

    boost1.initPtr = &boost1_initialize;
    boost1.executePtr = &boost1_execute;
    boost1.finishPtr = &boost1_finish;

    fast1.initPtr = &fast1_initialize;
    fast1.executePtr = &fast1_execute;
    fast1.finishPtr = &fast1_finish;

    preStage.initPtr = &pre_stage_initialize;
    preStage.executePtr = &pre_stage_execute;
    preStage.finishPtr = &pre_stage_finish;

    failedStaging.initPtr = &failed_stage_initialize;
    failedStaging.executePtr = &failed_stage_execute;
    failedStaging.finishPtr = &failed_stage_finish;

    postStage.initPtr = &post_stage_initialize;
    postStage.executePtr = &post_stage_execute;
    postStage.finishPtr = &post_stage_finish;

    sustainerIgnition.initPtr = &sustainer_ignition_initialize;
    sustainerIgnition.executePtr = &sustainer_ignition_execute;
    sustainerIgnition.finishPtr = &sustainer_ignition_finish;

    failedSustainerIgnition.initPtr = &failed_sustainer_ignition_initialize;
    failedSustainerIgnition.executePtr = &failed_sustainer_ignition_execute;
    failedSustainerIgnition.finishPtr = &failed_sustainer_ignition_finish;

    boost2.initPtr = &boost2_initialize;
    boost2.executePtr = &boost2_execute;
    boost2.finishPtr = &boost2_finish;

    fast2.initPtr = &fast2_initialize;
    fast2.executePtr = &fast2_execute;
    fast2.finishPtr = &fast2_finish;

    apogee.initPtr = &apogee_initialize;
    apogee.executePtr = &apogee_execute;
    apogee.finishPtr = &apogee_finish;

    coast.initPtr = &coast_initialize;
    coast.executePtr = &coast_execute;
    coast.finishPtr = &coast_finish;

    chute.initPtr = &chute_initialize;
    chute.executePtr = &chute_execute;
    chute.finishPtr = &chute_finish;

    landed.initPtr = &landed_initialize;
    landed.executePtr = &landed_execute;
    landed.finishPtr = &landed_finish;

    init_state_machine(&stateMachine, &safe);

	arm_status status = ARM_MATH_SUCCESS;

	status = init_kalman_filter(&heightEstimator, 2, 1, &F_f32[0], &G_f32[0], &P_f32[0], &Q_f32[0], &xHat_f32[0], &stateStdDevs_f32[0]);
}

// Calibrate sensors
void calibrate() {
    arm_mat_init_f32(&locToWorld3x3, 3, 3, locToWorld3x3_f32);

    // TODO: Make sure IMU data is synchronized
    float32_t Axyz[3] = {
        GetLinearAccelerationX(pHGuidei300Imu),
        GetLinearAccelerationY(pHGuidei300Imu),
        GetLinearAccelerationZ(pHGuidei300Imu)
    };

    calibrate_imu(Axyz, &locToWorld3x3);

    init_quaternion_xyzw(&localOrientation, 0, 0, 0, 1);

    arm_mat_init_f32(&globalOrientation3x3, 3, 3, globalOrientation3x3_f32);

    arm_mat_init_f32(&Axyz_local, 3, 1, Axyz_local_f32);
    arm_mat_init_f32(&Axyz_global, 3, 1, Axyz_global_f32);
}

// Updates the necessary variables to make flight decisions
// from sensor measurements and dynamic models of the system
void update_flight_state_variables() {
    arm_status status = ARM_MATH_SUCCESS;

    // TODO: Make sure IMU data is synchronized
    double wx = GetAngularRateX(pHGuidei300Imu);
    double wy = GetAngularRateY(pHGuidei300Imu);
    double wz = GetAngularRateZ(pHGuidei300Imu);

    // Update orientation
    localOrientation = update_local_orientation(&localOrientation, wx, wy, wz, DT_SECONDS);

    get_world_rotation_matrix(&locToWorld3x3, &localOrientation, &globalOrientation3x3);

    // Transform linear accelerations
    Axyz_local_f32[0] = GetLinearAccelerationX(pHGuidei300Imu);
    Axyz_local_f32[1] = GetLinearAccelerationY(pHGuidei300Imu);
    Axyz_local_f32[2] = GetLinearAccelerationZ(pHGuidei300Imu);

    arm_mat_mult_f32(&globalOrientation3x3, &Axyz_local, &Axyz_global);

    stateVariables.verticalAcceleration = Axyz_global_f32[2];

    float32_t un_f32[1] = { stateVariables.verticalAcceleration - GRAVITY_CONSTANT_MSEC2 };

    status = predict_kalman_filter(&heightEstimator, un_f32);
}

// State machine events
// Safe state
void safe_initialize() {

}

State *safe_execute() {
    return &safe;
}

void safe_finish() {

}

// Armed state
void armed_initialize() {

}

State *armed_execute() {
    return &safe;
}

void armed_finish() {
    
}

// Boost1 state
void boost1_initialize() {

}

State *boost1_execute() {
    return &safe;
}

void boost1_finish() {
    
}

// Fast1 state
void fast1_initialize() {

}

State *fast1_execute() {
    return &safe;
}

void fast1_finish() {
    
}

// Prestage state
void pre_stage_initialize() {

}

State *pre_stage_execute() {
    return &safe;
}

void pre_stage_finish() {
    
}

// Failed staging state
void failed_stage_initialize() {

}

State *failed_stage_execute() {
    return &safe;
}

void failed_stage_finish() {
    
}

// Post stage state
void post_stage_initialize() {

}

State *post_stage_execute() {
    return &safe;
}

void post_stage_finish() {
    
}

// Sustainer ignition state
void sustainer_ignition_initialize() {

}

State *sustainer_ignition_execute() {
    return &safe;
}

void sustainer_ignition_finish() {
    
}

// Failed sustainer ignition state
void failed_sustainer_ignition_initialize() {

}

State *failed_sustainer_ignition_execute() {
    return &safe;
}

void failed_sustainer_ignition_finish() {
    
}

// Boost2 state
void boost2_initialize() {

}

State *boost2_execute() {
    return &safe;
}

void boost2_finish() {
    
}

// Fast2 state
void fast2_initialize() {

}

State *fast2_execute() {
    return &safe;
}

void fast2_finish() {
    
}

// Apogee state
void apogee_initialize() {

}

State *apogee_execute() {
    return &safe;
}

void apogee_finish() {
    
}

// Coast state
void coast_initialize() {

}

State *coast_execute() {
    return &safe;
}

void coast_finish() {
    
}

// Chute state
void chute_initialize() {

}

State *chute_execute() {
    return &safe;
}

void chute_finish() {

}

// Landed state
void landed_initialize() {

}

State *landed_execute() {
    return &safe;
}

void landed_finish() {
    
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

void readRTCTimeBuffer(RTCTimeBuffer *buffer) {
    uint32_t tmpreg;

    if (buffer != NULL) {
        buffer->subSeconds = (uint32_t)(RTC->SSR);
        buffer->secondFraction = (uint32_t)(RTC->PRER & RTC_PRER_PREDIV_S);

        tmpreg = (uint32_t)(RTC->TR & RTC_TR_RESERVED_MASK);

        /* Fill the structure fields with the read parameters */
        buffer->hours      = (uint8_t)((tmpreg & (RTC_TR_HT  | RTC_TR_HU))  >> RTC_TR_HU_Pos);
        buffer->minutes    = (uint8_t)((tmpreg & (RTC_TR_MNT | RTC_TR_MNU)) >> RTC_TR_MNU_Pos);
        buffer->seconds    = (uint8_t)((tmpreg & (RTC_TR_ST  | RTC_TR_SU))  >> RTC_TR_SU_Pos);

         /* Convert the time structure parameters to Binary format */
        buffer->hours   = (uint8_t)convertBCDToBinary(buffer->hours);
        buffer->minutes = (uint8_t)convertBCDToBinary(buffer->minutes);
        buffer->seconds = (uint8_t)convertBCDToBinary(buffer->seconds);
    }
}

void _putchar(char character)
{
    usart3_tx_data[0] = (uint8_t) character;
    usart3_tx_data[1] = '\0';
    USART3_DMA1_Stream3_Write((uint8_t *) usart3_tx_data, 1);
}