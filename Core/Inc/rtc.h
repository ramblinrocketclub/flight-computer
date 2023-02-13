#ifndef RTC_H
#define RTC_H

typedef struct
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint32_t subseconds;
} RTC_Time_t;

typedef struct
{
    uint32_t year;
    uint32_t month;
    uint32_t day;
    uint32_t weekday;
} RTC_Date_t;

void RTC_Init(void);
void RTC_GetTime(RTC_Time_t *pTime);
void RTC_GetDate(RTC_Date_t *pDate);
double RTC_GetTimestamp(RTC_Time_t *pTime);

#endif /* RTC_H */

