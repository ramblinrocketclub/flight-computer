#ifndef RTC_H
#define RTC_H

#include "stm32h7xx.h"

// Daylight savings defs
#define RTC_DAYLIGHTSAVING_SUB1H       RTC_CR_SUB1H
#define RTC_DAYLIGHTSAVING_ADD1H       RTC_CR_ADD1H
#define RTC_DAYLIGHTSAVING_NONE        0x00000000u

// Store operation defs
#define RTC_STOREOPERATION_RESET        0x00000000u
#define RTC_STOREOPERATION_SET          RTC_CR_BKP

// Months
#define RTC_MONTH_JANUARY                   ((uint8_t)0x01)
#define RTC_MONTH_FEBRUARY                  ((uint8_t)0x02)
#define RTC_MONTH_MARCH                     ((uint8_t)0x03)
#define RTC_MONTH_APRIL                     ((uint8_t)0x04)
#define RTC_MONTH_MAY                       ((uint8_t)0x05)
#define RTC_MONTH_JUNE                      ((uint8_t)0x06)
#define RTC_MONTH_JULY                      ((uint8_t)0x07)
#define RTC_MONTH_AUGUST                    ((uint8_t)0x08)
#define RTC_MONTH_SEPTEMBER                 ((uint8_t)0x09)
#define RTC_MONTH_OCTOBER                   ((uint8_t)0x10)
#define RTC_MONTH_NOVEMBER                  ((uint8_t)0x11)
#define RTC_MONTH_DECEMBER                  ((uint8_t)0x12)

// Days of week
#define RTC_WEEKDAY_MONDAY                  ((uint8_t)0x01)
#define RTC_WEEKDAY_TUESDAY                 ((uint8_t)0x02)
#define RTC_WEEKDAY_WEDNESDAY               ((uint8_t)0x03)
#define RTC_WEEKDAY_THURSDAY                ((uint8_t)0x04)
#define RTC_WEEKDAY_FRIDAY                  ((uint8_t)0x05)
#define RTC_WEEKDAY_SATURDAY                ((uint8_t)0x06)
#define RTC_WEEKDAY_SUNDAY                  ((uint8_t)0x07)

#define RTC_TR_RESERVED_MASK    (RTC_TR_PM  | RTC_TR_HT | RTC_TR_HU | \
                                 RTC_TR_MNT | RTC_TR_MNU| RTC_TR_ST | \
                                 RTC_TR_SU)

#define RTC_DR_RESERVED_MASK    (RTC_DR_YT | RTC_DR_YU | RTC_DR_WDU | \
                                 RTC_DR_MT | RTC_DR_MU | RTC_DR_DT  | \
                                 RTC_DR_DU)

typedef struct RTCTimeBuffer {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t subSeconds;
    uint8_t secondFraction;
} RTCTimeBuffer;

uint8_t convertBCDToBinary(const uint8_t value);

#endif /* RTC_H */