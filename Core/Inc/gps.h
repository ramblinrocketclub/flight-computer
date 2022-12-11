#ifndef GPS_H
#define GPS_H

#include "ringbuffer.h"

#define NUM_FIELDS_MAX      32
#define FIELD_LENGTH_MAX    32

typedef struct GLL {
    double  lat;
    char    ns;
    double  lon;
    char    ew;
    double  time;
    char    status;
    char    pos_mode;
    uint32_t checksum;
} GLL_t;

typedef struct GGA {
    float   time;
    float   lat;
    char    ns;
    float   lon;
    char    ew;
    uint8_t quality;
    uint8_t numSV;
    float   hdop;
    float   alt;
    char    altUnit;
    float   sep;
    char    sepUnit;
    float   diffAge;
    float   diffStation;
    uint8_t checksum;
} GGA_t;

typedef struct GNS {
    float   time;
    float   lat;
    char    ns;
    float   lon;
    char    ew;
    char    pos_mode;
    uint8_t numSV;
    float   hdop;
    float   alt;
    float   sep;
    float   diffAge;
    float   diffStation;
    char    navStatus;
    uint32_t checksum;
} GNS_t;

typedef struct {
    GNS_t gns;
    GLL_t gll;
} GPS_Handle;

void parse_nmea_packet(GPS_Handle *gps, ringbuf_t *ringbuf, int num_messages, ...);
void save_packet(GPS_Handle *gps, char *message_id, uint8_t data[NUM_FIELDS_MAX][FIELD_LENGTH_MAX], uint8_t field_num);


#endif /* GPS_H */

