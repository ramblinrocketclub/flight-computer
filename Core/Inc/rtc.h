#ifndef RTC_H
#define RTC_H

typedef struct
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t subseconds;
} RTC_Time_t;

typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t weekday;
} RTC_Date_t;

void RTC_Init(void);
void RTC_GetTime(RTC_Time_t *pTime);
void RTC_GetDate(RTC_Date_t *pDate);
double RTC_GetTimestamp(RTC_Time_t *pTime);

#endif /* RTC_H */

