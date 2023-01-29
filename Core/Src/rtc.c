#include "stm32h723xx.h"
#include "rtc.h"

void RTC_Init(void)
{
    PWR->CR1 |= PWR_CR1_DBP;
    RCC->CSR |= RCC_CSR_LSION;
    while((RCC->CSR & RCC_CSR_LSIRDY) == 0);
    RCC->BDCR |= RCC_BDCR_RTCSEL_1;
    RCC->BDCR |= RCC_BDCR_RTCEN;
    RTC->WPR = 0xCAUL;
    RTC->WPR = 0x53UL;
    RTC->ISR |= RTC_ISR_INIT;
    while((RTC->ISR & RTC_ISR_INITF) == 0);
    RTC->PRER = 0x007F00F9UL;
    RTC->CR &= ~(RTC_CR_FMT);
    RTC->TR = 0;
    RTC->DR = 0x00232101UL;
    RTC->CR |= RTC_CR_BYPSHAD;
    RTC->ISR &= ~(RTC_ISR_INIT);
    RTC->WPR = 0xFFUL;
    PWR->CR1 &= ~(PWR_CR1_DBP);
}

uint8_t ConvertBCDToBinary(uint8_t byte)
{
    return (((byte & 0xF0) >> 4) * 10) + (byte & 0x0F);
}

void RTC_GetTime(RTC_Time_t *pTime)
{
    pTime->hours        = ConvertBCDToBinary((uint8_t) ((RTC->TR & RTC_TR_HT)   >> (RTC_TR_HT_Pos - 4))     + (uint8_t) ((RTC->TR & RTC_TR_HU)  >> RTC_TR_HU_Pos));
    pTime->minutes      = ConvertBCDToBinary((uint8_t) ((RTC->TR & RTC_TR_MNT)  >> (RTC_TR_MNT_Pos - 4))    + (uint8_t) ((RTC->TR & RTC_TR_MNU) >> RTC_TR_MNU_Pos));
    pTime->seconds      = ConvertBCDToBinary((uint8_t) ((RTC->TR & RTC_TR_ST)   >> (RTC_TR_ST_Pos - 4))     + (uint8_t) ((RTC->TR & RTC_TR_SU)  >> RTC_TR_SU_Pos));
    pTime->subseconds   = (1000 * (RTC->PRER & RTC_PRER_PREDIV_S) - RTC->SSR) / (RTC->SSR + 1);
}

void RTC_GetDate(RTC_Date_t *pDate)
{
    pDate->year     = ConvertBCDToBinary((uint8_t) ((RTC->DR & RTC_DR_YT) >> (RTC_DR_YT_Pos - 4)) + (uint8_t) ((RTC->DR & RTC_DR_YU) >> RTC_DR_YU_Pos));
    pDate->month    = ConvertBCDToBinary((uint8_t) ((RTC->DR & RTC_DR_MT) >> (RTC_DR_MT_Pos - 4)) + (uint8_t) ((RTC->DR & RTC_DR_MU) >> RTC_DR_MU_Pos));
    pDate->day      = ConvertBCDToBinary((uint8_t) ((RTC->DR & RTC_DR_DT) >> (RTC_DR_DT_Pos - 4)) + (uint8_t) ((RTC->DR & RTC_DR_DU) >> RTC_DR_DU_Pos));
    pDate->weekday  = ConvertBCDToBinary((uint8_t) ((RTC->DR & RTC_DR_WDU) >> RTC_DR_WDU_Pos));
}

double RTC_GetTimestamp(RTC_Time_t *pTime) {
    return pTime->hours * 3600.0 + pTime->minutes * 60.0 + pTime->seconds + pTime->subseconds;
}