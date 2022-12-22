#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "hguide_imu.h"

uint8_t DeserializeMessageA1(ringbuf_t *data, MessageA1_t *message_a1)
{
    if ((data->end - data->start + data->size) % data->size < 20)
    {
        return 1;
    }

    if (data->buf[data->start] != 0x0E || data->buf[data->start + 1] != 0xA1)
    {
        return 2;
    }

    message_a1->sync_byte                               = ringbuf_get(data);
    message_a1->message_id                              = ringbuf_get(data);
    message_a1->angular_rate_x                          = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -11));
    message_a1->angular_rate_y                          = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -11));
    message_a1->angular_rate_z                          = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -11));
    message_a1->linear_acceleration_x                   = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -5) * 0.3048);
    message_a1->linear_acceleration_y                   = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -5) * 0.3048);
    message_a1->linear_acceleration_z                   = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -5) * 0.3048);

    uint8_t lsb_status_word_1                           = ringbuf_get(data);
    uint8_t msb_status_word_1                           = ringbuf_get(data);

    message_a1->status_word_1.status_word_2_id          = lsb_status_word_1 & 0x0F;
    message_a1->status_word_1.control_frequency         = (lsb_status_word_1 >> 4) & 0x0F;
    message_a1->status_word_1.guidance_frequency        = msb_status_word_1 & 0x0F;
    message_a1->status_word_1.gyro_bit_summary          = (msb_status_word_1 >> 4) & 0x01;
    message_a1->status_word_1.accelerometer_bit_summary = (msb_status_word_1 >> 5) & 0x01;
    message_a1->status_word_1.magnetometer_bit_summary  = (msb_status_word_1 >> 6) & 0x01;
    message_a1->status_word_1.cbit_status               = (msb_status_word_1 >> 7) & 0x01;

    uint8_t lsb_multiplexed_status_word_2               = ringbuf_get(data);
    uint8_t msb_multiplexed_status_word_2               = ringbuf_get(data);

    if ((message_a1->status_word_1.status_word_2_id & 0x03) == 0)
    {
        message_a1->multiplexed_status_word_2.embedded_software_version     = lsb_multiplexed_status_word_2;
        message_a1->multiplexed_status_word_2.device_id                     = msb_multiplexed_status_word_2 & 0x0F;
        message_a1->multiplexed_status_word_2.performance_grade             = (msb_multiplexed_status_word_2 >> 4) & 0x0F;
    }

    else if ((message_a1->status_word_1.status_word_2_id & 0x03) == 1)
    {
        message_a1->multiplexed_status_word_2.gyro_statistics_summary           = (lsb_multiplexed_status_word_2 >> 0) & 0x01;
        message_a1->multiplexed_status_word_2.gyro_temperature_summary          = (lsb_multiplexed_status_word_2 >> 1) & 0x01;
        message_a1->multiplexed_status_word_2.accelerometer_statistics_summary  = (lsb_multiplexed_status_word_2 >> 2) & 0x01;
        message_a1->multiplexed_status_word_2.accelerometer_temperature_summary = (lsb_multiplexed_status_word_2 >> 3) & 0x01;
        message_a1->multiplexed_status_word_2.magnetometer_statistics_summary   = (lsb_multiplexed_status_word_2 >> 4) & 0x01;
        message_a1->multiplexed_status_word_2.magnetometer_temperature_summary  = (lsb_multiplexed_status_word_2 >> 5) & 0x01;
    }

    else if ((message_a1->status_word_1.status_word_2_id & 0x03) == 2)
    {
        message_a1->multiplexed_status_word_2.normal_mode_primary_crc       = (lsb_multiplexed_status_word_2 >> 0) & 0x01;
        message_a1->multiplexed_status_word_2.normal_mode_secondary_crc     = (lsb_multiplexed_status_word_2 >> 1) & 0x01;
        message_a1->multiplexed_status_word_2.factory_config_crc            = (lsb_multiplexed_status_word_2 >> 2) & 0x01;
        message_a1->multiplexed_status_word_2.factory_coefficient_crc       = (lsb_multiplexed_status_word_2 >> 3) & 0x01;
        message_a1->multiplexed_status_word_2.io_config_crc                 = (lsb_multiplexed_status_word_2 >> 4) & 0x01;
        message_a1->multiplexed_status_word_2.primary_image_boot            = (msb_multiplexed_status_word_2 >> (10 - 8)) & 0x01;
        message_a1->multiplexed_status_word_2.memory_test_summary           = (msb_multiplexed_status_word_2 >> (11 - 8)) & 0x01;
        message_a1->multiplexed_status_word_2.processor_test_summary        = (msb_multiplexed_status_word_2 >> (12 - 8)) & 0x01;
        message_a1->multiplexed_status_word_2.wdt_loop_completion_summary   = (msb_multiplexed_status_word_2 >> (13 - 8)) & 0x01;
        message_a1->multiplexed_status_word_2.power_up_bit_status           = (msb_multiplexed_status_word_2 >> (14 - 8)) & 0x01;
        message_a1->multiplexed_status_word_2.continuous_bit_status         = (msb_multiplexed_status_word_2 >> (15 - 8)) & 0x01;
    }

    else if ((message_a1->status_word_1.status_word_2_id & 0x03) == 3)
    {
        message_a1->multiplexed_status_word_2.device_temperature    = (lsb_multiplexed_status_word_2 + (msb_multiplexed_status_word_2 << 8)) * pow((double) 2, (double) -8);
    }

    message_a1->checksum = ringbuf_get(data) + (ringbuf_get(data) << 8);

    return 0;
}

uint8_t DeserializeMessageA2(ringbuf_t *data, MessageA2_t *message_a2)
{
    if ((data->end - data->start + data->size) % data->size < 20)
    {
        return 1;
    }

    if (data->buf[data->start] != 0x0E || data->buf[data->start + 1] != 0xA2)
    {
        return 2;
    }

    message_a2->sync_byte                               = ringbuf_get(data);
    message_a2->message_id                              = ringbuf_get(data);
    message_a2->angular_rate_x                          = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -11));
    message_a2->angular_rate_y                          = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -11));
    message_a2->angular_rate_z                          = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -11));
    message_a2->linear_acceleration_x                   = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -5) * 0.3048);
    message_a2->linear_acceleration_y                   = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -5) * 0.3048);
    message_a2->linear_acceleration_z                   = (ringbuf_get(data) + (ringbuf_get(data) << 8)) * (pow((double) 2, (double) -5) * 0.3048);

    uint8_t lsb_status_word_1                           = ringbuf_get(data);
    uint8_t msb_status_word_1                           = ringbuf_get(data);

    message_a2->status_word_1.status_word_2_id          = lsb_status_word_1 & 0x0F;
    message_a2->status_word_1.control_frequency         = (lsb_status_word_1 >> 4) & 0x0F;
    message_a2->status_word_1.guidance_frequency        = msb_status_word_1 & 0x0F;
    message_a2->status_word_1.gyro_bit_summary          = (msb_status_word_1 >> 4) & 0x01;
    message_a2->status_word_1.accelerometer_bit_summary = (msb_status_word_1 >> 5) & 0x01;
    message_a2->status_word_1.magnetometer_bit_summary  = (msb_status_word_1 >> 6) & 0x01;
    message_a2->status_word_1.cbit_status               = (msb_status_word_1 >> 7) & 0x01;

    uint8_t lsb_multiplexed_status_word_2               = ringbuf_get(data);
    uint8_t msb_multiplexed_status_word_2               = ringbuf_get(data);

    if ((message_a2->status_word_1.status_word_2_id & 0x03) == 0)
    {
        message_a2->multiplexed_status_word_2.embedded_software_version     = lsb_multiplexed_status_word_2;
        message_a2->multiplexed_status_word_2.device_id                     = msb_multiplexed_status_word_2 & 0x0F;
        message_a2->multiplexed_status_word_2.performance_grade             = (msb_multiplexed_status_word_2 >> 4) & 0x0F;
    }

    else if ((message_a2->status_word_1.status_word_2_id & 0x03) == 1)
    {
        message_a2->multiplexed_status_word_2.gyro_statistics_summary           = (lsb_multiplexed_status_word_2 >> 0) & 0x01;
        message_a2->multiplexed_status_word_2.gyro_temperature_summary          = (lsb_multiplexed_status_word_2 >> 1) & 0x01;
        message_a2->multiplexed_status_word_2.accelerometer_statistics_summary  = (lsb_multiplexed_status_word_2 >> 2) & 0x01;
        message_a2->multiplexed_status_word_2.accelerometer_temperature_summary = (lsb_multiplexed_status_word_2 >> 3) & 0x01;
        message_a2->multiplexed_status_word_2.magnetometer_statistics_summary   = (lsb_multiplexed_status_word_2 >> 4) & 0x01;
        message_a2->multiplexed_status_word_2.magnetometer_temperature_summary  = (lsb_multiplexed_status_word_2 >> 5) & 0x01;
    }

    else if ((message_a2->status_word_1.status_word_2_id & 0x03) == 2)
    {
        message_a2->multiplexed_status_word_2.normal_mode_primary_crc       = (lsb_multiplexed_status_word_2 >> 0) & 0x01;
        message_a2->multiplexed_status_word_2.normal_mode_secondary_crc     = (lsb_multiplexed_status_word_2 >> 1) & 0x01;
        message_a2->multiplexed_status_word_2.factory_config_crc            = (lsb_multiplexed_status_word_2 >> 2) & 0x01;
        message_a2->multiplexed_status_word_2.factory_coefficient_crc       = (lsb_multiplexed_status_word_2 >> 3) & 0x01;
        message_a2->multiplexed_status_word_2.io_config_crc                 = (lsb_multiplexed_status_word_2 >> 4) & 0x01;
        message_a2->multiplexed_status_word_2.primary_image_boot            = (msb_multiplexed_status_word_2 >> (10 - 8)) & 0x01;
        message_a2->multiplexed_status_word_2.memory_test_summary           = (msb_multiplexed_status_word_2 >> (11 - 8)) & 0x01;
        message_a2->multiplexed_status_word_2.processor_test_summary        = (msb_multiplexed_status_word_2 >> (12 - 8)) & 0x01;
        message_a2->multiplexed_status_word_2.wdt_loop_completion_summary   = (msb_multiplexed_status_word_2 >> (13 - 8)) & 0x01;
        message_a2->multiplexed_status_word_2.power_up_bit_status           = (msb_multiplexed_status_word_2 >> (14 - 8)) & 0x01;
        message_a2->multiplexed_status_word_2.continuous_bit_status         = (msb_multiplexed_status_word_2 >> (15 - 8)) & 0x01;
    }

    else if ((message_a2->status_word_1.status_word_2_id & 0x03) == 3)
    {
        message_a2->multiplexed_status_word_2.device_temperature    = (lsb_multiplexed_status_word_2 + (msb_multiplexed_status_word_2 << 8)) * pow((double) 2, (double) -8);
    }

    message_a2->delta_angle_x       = (ringbuf_get(data) + (ringbuf_get(data) << 8) + (ringbuf_get(data) << 16) + (ringbuf_get(data) << 24)) * (pow((double) 2, (double) -33));
    message_a2->delta_angle_y       = (ringbuf_get(data) + (ringbuf_get(data) << 8) + (ringbuf_get(data) << 16) + (ringbuf_get(data) << 24)) * (pow((double) 2, (double) -33));
    message_a2->delta_angle_z       = (ringbuf_get(data) + (ringbuf_get(data) << 8) + (ringbuf_get(data) << 16) + (ringbuf_get(data) << 24)) * (pow((double) 2, (double) -33));
    message_a2->delta_velocity_x    = (ringbuf_get(data) + (ringbuf_get(data) << 8) + (ringbuf_get(data) << 16) + (ringbuf_get(data) << 24)) * (pow((double) 2, (double) -27) * 0.3048);
    message_a2->delta_velocity_y    = (ringbuf_get(data) + (ringbuf_get(data) << 8) + (ringbuf_get(data) << 16) + (ringbuf_get(data) << 24)) * (pow((double) 2, (double) -27) * 0.3048);
    message_a2->delta_velocity_z    = (ringbuf_get(data) + (ringbuf_get(data) << 8) + (ringbuf_get(data) << 16) + (ringbuf_get(data) << 24)) * (pow((double) 2, (double) -27) * 0.3048);


    message_a2->checksum = ringbuf_get(data) + (ringbuf_get(data) << 8);

    return 0;
}

uint8_t ProcessHGuidei300(ringbuf_t *data)
{
    if ((data->end - data->start + data->size) % data->size < 20)
    {
        return 1;
    }

    if (data->buf[data->start] == 0x0E && data->buf[data->start + 1] == 0xA1)
    {
        MessageA1_t _control_message;
        MessageA1_t *control_message = &_control_message;
        DeserializeMessageA1(data, control_message);
    }

    else if (data->buf[data->start] == 0x0E && data->buf[data->start + 1] == 0xA2)
    {
        MessageA2_t _inertial_message;
        MessageA2_t *inertial_message = &_inertial_message;
        DeserializeMessageA2(data, inertial_message);
    }

    return 0;
}

