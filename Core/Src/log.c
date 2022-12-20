#include "log.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

extern uint8_t *uart8_tx_data;

void USART3_DMA1_Stream3_Write(uint8_t *data, uint16_t length);

void log_message(int level, bool newline, const char *format, ...) {
    // Log levels:
    // [FATAL]: 
    // [ERROR]:
    // [WARNING]: 
    // [INFO]: 

    uint16_t write_len = 0;
    
    va_list args;

    switch (level) {
        case LOG_LEVEL_FATAL:
            write_len = sprintf((char *)uart8_tx_data, "%s", "[FATAL]: ");
            break;
        case LOG_LEVEL_ERROR:
            write_len = sprintf((char *)uart8_tx_data, "%s", "[ERROR]: ");
            break;
        case LOG_LEVEL_WARN:
            write_len = sprintf((char *)uart8_tx_data, "%s", "[WARNING]: ");
            break;
        case LOG_LEVEL_INFO:
            write_len = sprintf((char *)uart8_tx_data, "%s", "[INFO]: ");
            break;
        default:
            break;
    }
    
    va_start(args, format);
    write_len += vsprintf((char *)(uart8_tx_data + write_len), format, args);

    if (newline == true) {
        uart8_tx_data[write_len] = '\n';
        write_len++;
        uart8_tx_data[write_len] = '\0';
    }
    
    USART3_DMA1_Stream3_Write((uint8_t *) uart8_tx_data, write_len);
    
    va_end(args);
}