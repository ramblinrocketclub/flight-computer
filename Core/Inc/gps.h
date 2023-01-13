#ifndef GTXR_GPS_H
#define GTXR_GPS_H

#include "ringbuffer.h"

#define NUM_FIELDS_MAX      32
#define FIELD_LENGTH_MAX    32

typedef struct {
    char        message_id[6];
    double      latitude;
    char        north_south_indicator;
    double      longitude;
    char        east_west_indicator;
    double      time;
    char        status;
    char        positioning_mode;
    uint32_t    checksum;
} GPS_GLL_t;

typedef struct {
    char        message_id[6];
    double      time;
    double      latitude;
    char        north_south_indicator;
    double      longitutde;
    char        east_west_indicator;
    uint8_t     quality;
    uint8_t     number_of_satelites;
    double      horizontal_dilution_of_precision;
    double      altitude;
    char        altitude_units;
    double      geoid_separation;
    char        geoid_separation_units;
    double      differential_corrections_age;
    double      differential_corrections_station_id;
    uint32_t    checksum;
} GPS_GGA_t;

typedef struct {
    double      time;
    double      latitude;
    char        north_south_indicator;
    double      longitude;
    char        east_west_indicator;
    char        status;
    char        positioning_mode;
} GPS_t;

void GPS_ProcessData(GPS_t *gps, RingBuffer_t *data);

#endif /* GPS_H */

// #ifndef GPS_H
// #define GPS_H
//
// #include "ringbuffer.h"
//
// #define NUM_FIELDS_MAX      32
// #define FIELD_LENGTH_MAX    32
//
// typedef struct GLL {
//     double  lat;
//     char    ns;
//     double  lon;
//     char    ew;
//     double  time;
//     char    status;
//     char    pos_mode;
//     uint32_t checksum;
// } GLL_t;
//
// typedef struct GGA {
//     float   time;
//     float   lat;
//     char    ns;
//     float   lon;
//     char    ew;
//     uint8_t quality;
//     uint8_t numSV;
//     float   hdop;
//     float   alt;
//     char    altUnit;
//     float   sep;
//     char    sepUnit;
//     float   diffAge;
//     float   diffStation;
//     uint8_t checksum;
// } GGA_t;
//
// typedef struct GNS {
//     float   time;
//     float   lat;
//     char    ns;
//     float   lon;
//     char    ew;
//     char    pos_mode;
//     uint8_t numSV;
//     float   hdop;
//     float   alt;
//     float   sep;
//     float   diffAge;
//     float   diffStation;
//     char    navStatus;
//     uint32_t checksum;
// } GNS_t;
//
// typedef struct {
//     GNS_t gns;
//     GLL_t gll;
// } GPS_Handle;
//
// // void GPS_ProcessData(GPS_Handle *pGPS, RingBuffer_t pRingBuffer, int number_of_messages, ...);
// void parse_nmea_packet(GPS_Handle *gps, ringbuf_t *ringbuf, int num_messages, ...);
// void save_packet(GPS_Handle *gps, char *message_id, uint8_t data[NUM_FIELDS_MAX][FIELD_LENGTH_MAX], uint8_t field_num);
//
//
// #endif /* GPS_H */

