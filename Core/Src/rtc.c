#include "rtc.h"

uint8_t convertBCDToBinary(const uint8_t value) {
    uint8_t tmp;
    tmp = ((value & 0xF0U) >> 4U) * 10U;
    return (tmp + (value & 0x0FU));
}