#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "gps.h"

void parse_nmea_packet(GPS_Handle *gps, ringbuf_t *ringbuf, int num_messages, ...)
{
    char requested_message_id[num_messages][6];
    va_list arguments;
    va_start(arguments, num_messages);

    for (int i = 0; i < num_messages; i++)
    {
       strcpy(requested_message_id[i], va_arg(arguments, char *));
    }

    while ((ringbuf->end - ringbuf->start + ringbuf->size) % (ringbuf->size) > 82)
    {
        // traverse the ring buffer until the beginning of a packet
        while (ringbuf_get(ringbuf) != '$');
        uint8_t message_id[6];

        // store the message id
        for (int i = 0; i < 5; i++)
            message_id[i] = ringbuf_get(ringbuf);

        message_id[5] = '\0';

        // search through requested messages
        for (int j = 0; j < num_messages; j++)
        {
            // compare the requested messages with the current message read
            if (strcmp((char *) message_id, requested_message_id[j]) == 0)
            {
                uint8_t data[NUM_FIELDS_MAX][FIELD_LENGTH_MAX] = {[0 ... (NUM_FIELDS_MAX - 1)] = {[0 ... (FIELD_LENGTH_MAX - 1)] = 0}};

                ringbuf_get(ringbuf);
                uint8_t tmp = '\0';
                char int_string[FIELD_LENGTH_MAX];

                uint8_t num_fields = 0;

                // parse through the packet and store data into gps structure
                while (tmp != '\r')
                {
                    do
                    {
                        tmp = ringbuf_get(ringbuf);
                        if (tmp == ',' || tmp == '*' || tmp == '\r')
                        {
                            save_packet(gps, (char *) message_id, data, num_fields);
                            num_fields++;
                            continue;
                        }
                        sprintf(int_string, "%c", tmp);
                        strcat((char *) data[num_fields], int_string);
                    } while (tmp != ',' && tmp != '*' && tmp != '\r');
                }
            }
        }
    }
}

void save_packet(GPS_Handle *gps, char *message_id, uint8_t data[NUM_FIELDS_MAX][FIELD_LENGTH_MAX], uint8_t field_num)
{
    if (strcmp(message_id, "GNGNS") == 0)
    {
        // asm("nop");
    }

    else if (strcmp(message_id, "GNGLL") == 0)
    {
        switch (field_num)
        {
            case 0:
                gps->gll.lat = (double) atof((char *) data[field_num]);
                break;
            case 1:
                gps->gll.ns = (char) data[field_num][0];
                break;
            case 2:
                gps->gll.lon = (double) atof((char *) data[field_num]);
                break;
            case 3:
                gps->gll.ew = (char) data[field_num][0];
                break;
            case 4:
                gps->gll.time = (double) atof((char *) data[field_num]);
                break;
            case 5:
                gps->gll.status = (char) data[field_num][0];
                break;
            case 6:
                gps->gll.pos_mode = (char) data[field_num][0];
                break;
            case 7:
                gps->gll.checksum = 0;

                if ((uint8_t) data[field_num][0] - (uint8_t) 'A' >= 0)
                    gps->gll.checksum += 16 * ((uint8_t) data[field_num][0] - (uint8_t) 'A' + 10);
                else
                    gps->gll.checksum += 16 * ((uint32_t) data[field_num][0] - (uint32_t) '0');

                if ((uint8_t) data[field_num][1] - (uint8_t) 'A' >= 0)
                    gps->gll.checksum += ((uint8_t) data[field_num][1] - (uint8_t) 'A' + 10);
                else
                    gps->gll.checksum += ((uint8_t) data[field_num][1] - (uint8_t) '0');

                break;
            default:
                for(;;);
        }
    }
}

