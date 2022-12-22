#ifndef HGUIDE_IMU_H
#define HGUIDE_IMU_H

#include <stdbool.h>

typedef enum
{
    MSG_CTRL_0_RATE    = 0, //  Message is not being outputted
	MSG_CTRL_600_RATE  = 1,
	MSG_CTRL_1200_RATE = 2,
	MSG_CTRL_1800_RATE = 3,
	MSG_CTRL_3600_RATE = 4,
	MSG_CTRL_1000_RATE = 5,
	MSG_CTRL_900_RATE  = 6,
	MSG_CTRL_1500_RATE = 7
} ControlFrequency_t;

typedef enum
{
    MSG_GUID_0_RATE   = 0, //  Message is not being outputted
	MSG_GUID_100_RATE = 1,
	MSG_GUID_200_RATE = 2,
	MSG_GUID_300_RATE = 3,
	MSG_GUID_400_RATE = 4,
	MSG_GUID_500_RATE = 5,
	MSG_GUID_600_RATE = 6,
	MSG_GUID_250_RATE = 7
} GuidanceFrequency_t;

typedef struct
{
    uint8_t             status_word_2_id;
    ControlFrequency_t  control_frequency;
    GuidanceFrequency_t guidance_frequency;
    bool                gyro_bit_summary;
    bool                accelerometer_bit_summary;
    bool                magnetometer_bit_summary;
    bool                cbit_status;
} HGuideIMUStatusWord1_t;

typedef struct
{
    uint8_t embedded_software_version;
    uint8_t device_id;
    uint8_t performance_grade;
    bool    gyro_statistics_summary;
    bool    gyro_temperature_summary;
    bool    accelerometer_statistics_summary;
    bool    accelerometer_temperature_summary;
    bool    magnetometer_statistics_summary;
    bool    magnetometer_temperature_summary;
    bool    normal_mode_primary_crc;
    bool    normal_mode_secondary_crc;
    bool    factory_config_crc;
    bool    factory_coefficient_crc;
    bool    io_config_crc;
    bool    primary_image_boot;
    bool    memory_test_summary;
    bool    processor_test_summary;
    bool    wdt_loop_completion_summary;
    bool    power_up_bit_status;
    bool    continuous_bit_status;
    double  device_temperature;
} HGuideIMUMultiplexStatusWord2_t;

typedef struct
{
    uint8_t                         sync_byte;
    uint8_t                         message_id;
    double                          angular_rate_x;
    double                          angular_rate_y;
    double                          angular_rate_z;
    double                          linear_acceleration_x;
    double                          linear_acceleration_y;
    double                          linear_acceleration_z;
    HGuideIMUStatusWord1_t          status_word_1;
    HGuideIMUMultiplexStatusWord2_t multiplexed_status_word_2;
    uint16_t                        checksum;
} MessageA1_t;

typedef struct
{
    uint8_t                         sync_byte;
    uint8_t                         message_id;
    double                          angular_rate_x;
    double                          angular_rate_y;
    double                          angular_rate_z;
    double                          linear_acceleration_x;
    double                          linear_acceleration_y;
    double                          linear_acceleration_z;
    HGuideIMUStatusWord1_t          status_word_1;
    HGuideIMUMultiplexStatusWord2_t multiplexed_status_word_2;
    double                          delta_angle_x;
    double                          delta_angle_y;
    double                          delta_angle_z;
    double                          delta_velocity_x;
    double                          delta_velocity_y;
    double                          delta_velocity_z;
    uint16_t                        checksum;
} MessageA2_t;

typedef struct
{
    MessageA1_t message_a1;
    MessageA2_t message_a2;
} HGuidei300Imu_t;

uint8_t DeserializeMessageA1(ringbuf_t *data, MessageA1_t *message_a1);
uint8_t ProcessHGuidei300(ringbuf_t *data);


#endif /* HGUIDE_IMU_H */

