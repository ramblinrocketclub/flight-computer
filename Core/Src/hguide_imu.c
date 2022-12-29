#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "ringbuffer_new.h"
#include "hguide_imu.h"

const double LUT_NEG_POWERS_OF_TWO[] =
{   1.0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.5000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.2500000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.1250000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0625000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0312500000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0156250000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0078125000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0039062500000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0019531250000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0009765625000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0004882812500000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0002441406250000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0001220703125000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000610351562500000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000305175781250000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000152587890625000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000076293945312500000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000038146972656250000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000019073486328125000000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000009536743164062500000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000004768371582031250000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000002384185791015625000000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000001192092895507812500000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000596046447753906250000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000298023223876953125000000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000149011611938476562500000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000074505805969238281250000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000037252902984619140625000000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000018626451492309570312500000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000009313225746154785156250000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000004656612873077392578125000000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000002328306436538696289062500000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000001164153218269348144531250000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000000582076609134674072265625000000000000000000000000000000000000000000000000000000000000000000,
    0.0000000000291038304567337036132812500000000000000000000000000000000000000000000000000000000000000000,
    0.0000000000145519152283668518066406250000000000000000000000000000000000000000000000000000000000000000,
    0.0000000000072759576141834259033203125000000000000000000000000000000000000000000000000000000000000000,
    0.0000000000036379788070917129516601562500000000000000000000000000000000000000000000000000000000000000,
    0.0000000000018189894035458564758300781250000000000000000000000000000000000000000000000000000000000000,
    0.0000000000009094947017729282379150390625000000000000000000000000000000000000000000000000000000000000,
    0.0000000000004547473508864641189575195312500000000000000000000000000000000000000000000000000000000000,
    0.0000000000002273736754432320594787597656250000000000000000000000000000000000000000000000000000000000,
    0.0000000000001136868377216160297393798828125000000000000000000000000000000000000000000000000000000000,
    0.0000000000000568434188608080148696899414062500000000000000000000000000000000000000000000000000000000,
    0.0000000000000284217094304040074348449707031250000000000000000000000000000000000000000000000000000000,
    0.0000000000000142108547152020037174224853515625000000000000000000000000000000000000000000000000000000,
    0.0000000000000071054273576010018587112426757812500000000000000000000000000000000000000000000000000000,
    0.0000000000000035527136788005009293556213378906250000000000000000000000000000000000000000000000000000,
    0.0000000000000017763568394002504646778106689453125000000000000000000000000000000000000000000000000000,
    0.0000000000000008881784197001252323389053344726562500000000000000000000000000000000000000000000000000,
    0.0000000000000004440892098500626161694526672363281250000000000000000000000000000000000000000000000000,
    0.0000000000000002220446049250313080847263336181640625000000000000000000000000000000000000000000000000,
    0.0000000000000001110223024625156540423631668090820312500000000000000000000000000000000000000000000000,
    0.0000000000000000555111512312578270211815834045410156250000000000000000000000000000000000000000000000,
    0.0000000000000000277555756156289135105907917022705078125000000000000000000000000000000000000000000000,
    0.0000000000000000138777878078144567552953958511352539062500000000000000000000000000000000000000000000,
    0.0000000000000000069388939039072283776476979255676269531250000000000000000000000000000000000000000000,
    0.0000000000000000034694469519536141888238489627838134765625000000000000000000000000000000000000000000,
    0.0000000000000000017347234759768070944119244813919067382812500000000000000000000000000000000000000000,
    0.0000000000000000008673617379884035472059622406959533691406250000000000000000000000000000000000000000,
    0.0000000000000000004336808689942017736029811203479766845703125000000000000000000000000000000000000000,
    0.0000000000000000002168404344971008868014905601739883422851562500000000000000000000000000000000000000,
    0.0000000000000000001084202172485504434007452800869941711425781250000000000000000000000000000000000000,
    0.0000000000000000000542101086242752217003726400434970855712890625000000000000000000000000000000000000,
    0.0000000000000000000271050543121376108501863200217485427856445312500000000000000000000000000000000000,
    0.0000000000000000000135525271560688054250931600108742713928222656250000000000000000000000000000000000,
    0.0000000000000000000067762635780344027125465800054371356964111328125000000000000000000000000000000000,
    0.0000000000000000000033881317890172013562732900027185678482055664062500000000000000000000000000000000,
    0.0000000000000000000016940658945086006781366450013592839241027832031250000000000000000000000000000000,
    0.0000000000000000000008470329472543003390683225006796419620513916015625000000000000000000000000000000,
    0.0000000000000000000004235164736271501695341612503398209810256958007812500000000000000000000000000000,
    0.0000000000000000000002117582368135750847670806251699104905128479003906250000000000000000000000000000,
    0.0000000000000000000001058791184067875423835403125849552452564239501953125000000000000000000000000000,
    0.0000000000000000000000529395592033937711917701562924776226282119750976562500000000000000000000000000,
    0.0000000000000000000000264697796016968855958850781462388113141059875488281250000000000000000000000000,
    0.0000000000000000000000132348898008484427979425390731194056570529937744140625000000000000000000000000,
    0.0000000000000000000000066174449004242213989712695365597028285264968872070312500000000000000000000000,
    0.0000000000000000000000033087224502121106994856347682798514142632484436035156250000000000000000000000,
    0.0000000000000000000000016543612251060553497428173841399257071316242218017578125000000000000000000000,
    0.0000000000000000000000008271806125530276748714086920699628535658121109008789062500000000000000000000,
    0.0000000000000000000000004135903062765138374357043460349814267829060554504394531250000000000000000000,
    0.0000000000000000000000002067951531382569187178521730174907133914530277252197265625000000000000000000,
    0.0000000000000000000000001033975765691284593589260865087453566957265138626098632812500000000000000000,
    0.0000000000000000000000000516987882845642296794630432543726783478632569313049316406250000000000000000,
    0.0000000000000000000000000258493941422821148397315216271863391739316284656524658203125000000000000000,
    0.0000000000000000000000000129246970711410574198657608135931695869658142328262329101562500000000000000,
    0.0000000000000000000000000064623485355705287099328804067965847934829071164131164550781250000000000000,
    0.0000000000000000000000000032311742677852643549664402033982923967414535582065582275390625000000000000,
    0.0000000000000000000000000016155871338926321774832201016991461983707267791032791137695312500000000000,
    0.0000000000000000000000000008077935669463160887416100508495730991853633895516395568847656250000000000,
    0.0000000000000000000000000004038967834731580443708050254247865495926816947758197784423828125000000000,
    0.0000000000000000000000000002019483917365790221854025127123932747963408473879098892211914062500000000,
    0.0000000000000000000000000001009741958682895110927012563561966373981704236939549446105957031250000000,
    0.0000000000000000000000000000504870979341447555463506281780983186990852118469774723052978515625000000,
    0.0000000000000000000000000000252435489670723777731753140890491593495426059234887361526489257812500000,
    0.0000000000000000000000000000126217744835361888865876570445245796747713029617443680763244628906250000,
    0.0000000000000000000000000000063108872417680944432938285222622898373856514808721840381622314453125000,
    0.0000000000000000000000000000031554436208840472216469142611311449186928257404360920190811157226562500,
    0.0000000000000000000000000000015777218104420236108234571305655724593464128702180460095405578613281250  };

uint8_t DeserializeMessageA1(RingBuffer_t *data, MessageA1_t *message_a1)
{
    if (data->count < 20)
    {
        return 1;
    }

    if (data->buffer[data->tail] != 0x0E || data->buffer[data->tail + 1] != 0xA1)
    {
        return 2;
    }

    uint8_t message_bytes[20];
    RingBuffer_Get(data, message_bytes, 20);

    message_a1->sync_byte               = message_bytes[0];
    message_a1->message_id              = message_bytes[1];

    message_a1->angular_rate_x          = (double)((int16_t) (message_bytes[2] + (message_bytes[3] << 8))) * (LUT_NEG_POWERS_OF_TWO[11]);
    message_a1->angular_rate_y          = (double)((int16_t) (message_bytes[4] + (message_bytes[5] << 8))) * (LUT_NEG_POWERS_OF_TWO[11]);
    message_a1->angular_rate_z          = (double)((int16_t) (message_bytes[6] + (message_bytes[7] << 8))) * (LUT_NEG_POWERS_OF_TWO[11]);

    message_a1->linear_acceleration_x   = (double)((int16_t) (message_bytes[8] + (message_bytes[9] << 8))) * (LUT_NEG_POWERS_OF_TWO[5] * 0.3048);
    message_a1->linear_acceleration_y   = (double)((int16_t) (message_bytes[10] + (message_bytes[11] << 8))) * (LUT_NEG_POWERS_OF_TWO[5] * 0.3048);
    message_a1->linear_acceleration_z   = (double)((int16_t) (message_bytes[12] + (message_bytes[13] << 8))) * (LUT_NEG_POWERS_OF_TWO[5] * 0.3048);

    uint8_t lsb_status_word_1                           = message_bytes[14];
    uint8_t msb_status_word_1                           = message_bytes[15];

    message_a1->status_word_1.status_word_2_id          = lsb_status_word_1 & 0x0F;
    message_a1->status_word_1.control_frequency         = (lsb_status_word_1 >> 4) & 0x0F;
    message_a1->status_word_1.guidance_frequency        = msb_status_word_1 & 0x0F;
    message_a1->status_word_1.gyro_bit_summary          = (msb_status_word_1 >> 4) & 0x01;
    message_a1->status_word_1.accelerometer_bit_summary = (msb_status_word_1 >> 5) & 0x01;
    message_a1->status_word_1.magnetometer_bit_summary  = (msb_status_word_1 >> 6) & 0x01;
    message_a1->status_word_1.cbit_status               = (msb_status_word_1 >> 7) & 0x01;

    uint8_t lsb_multiplexed_status_word_2               = message_bytes[16];
    uint8_t msb_multiplexed_status_word_2               = message_bytes[17];

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

    message_a1->checksum = message_bytes[18] + (message_bytes[19] << 8);

    return 0;
}

uint8_t DeserializeMessageA2(RingBuffer_t *data, MessageA2_t *message_a2)
{
    if (data->count < 44)
    {
        return 1;
    }

    if (data->buffer[data->tail] != 0x0E || data->buffer[data->tail + 1] != 0xA2)
    {
        return 2;
    }

    uint8_t message_bytes[44];
    RingBuffer_Get(data, message_bytes, 44);

    message_a2->sync_byte               = message_bytes[0];
    message_a2->message_id              = message_bytes[1];

    message_a2->angular_rate_x          = (double)((int16_t) (message_bytes[2] + (message_bytes[3] << 8))) * (LUT_NEG_POWERS_OF_TWO[11]);
    message_a2->angular_rate_y          = (double)((int16_t) (message_bytes[4] + (message_bytes[5] << 8))) * (LUT_NEG_POWERS_OF_TWO[11]);
    message_a2->angular_rate_z          = (double)((int16_t) (message_bytes[6] + (message_bytes[7] << 8))) * (LUT_NEG_POWERS_OF_TWO[11]);

    message_a2->linear_acceleration_x   = (double)((int16_t) (message_bytes[8] + (message_bytes[9] << 8))) * (LUT_NEG_POWERS_OF_TWO[5] * 0.3048);
    message_a2->linear_acceleration_y   = (double)((int16_t) (message_bytes[10] + (message_bytes[11] << 8))) * (LUT_NEG_POWERS_OF_TWO[5] * 0.3048);
    message_a2->linear_acceleration_z   = (double)((int16_t) (message_bytes[12] + (message_bytes[13] << 8))) * (LUT_NEG_POWERS_OF_TWO[5] * 0.3048);


    uint8_t lsb_status_word_1                           = message_bytes[14];
    uint8_t msb_status_word_1                           = message_bytes[15];

    message_a2->status_word_1.status_word_2_id          = lsb_status_word_1 & 0x0F;
    message_a2->status_word_1.control_frequency         = (lsb_status_word_1 >> 4) & 0x0F;
    message_a2->status_word_1.guidance_frequency        = msb_status_word_1 & 0x0F;
    message_a2->status_word_1.gyro_bit_summary          = (msb_status_word_1 >> 4) & 0x01;
    message_a2->status_word_1.accelerometer_bit_summary = (msb_status_word_1 >> 5) & 0x01;
    message_a2->status_word_1.magnetometer_bit_summary  = (msb_status_word_1 >> 6) & 0x01;
    message_a2->status_word_1.cbit_status               = (msb_status_word_1 >> 7) & 0x01;

    uint8_t lsb_multiplexed_status_word_2               = message_bytes[16];
    uint8_t msb_multiplexed_status_word_2               = message_bytes[17];

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

    message_a2->delta_angle_x       = (double) ((int32_t) (message_bytes[18] + (message_bytes[19] << 8) + (message_bytes[20] << 16) + (message_bytes[21] << 24))) * (LUT_NEG_POWERS_OF_TWO[33]);
    message_a2->delta_angle_y       = (double) ((int32_t) (message_bytes[22] + (message_bytes[23] << 8) + (message_bytes[24] << 16) + (message_bytes[25] << 24))) * (LUT_NEG_POWERS_OF_TWO[33]);
    message_a2->delta_angle_z       = (double) ((int32_t) (message_bytes[26] + (message_bytes[27] << 8) + (message_bytes[28] << 16) + (message_bytes[29] << 24))) * (LUT_NEG_POWERS_OF_TWO[33]);

    message_a2->delta_velocity_x    = (double) ((int32_t) (message_bytes[30] + (message_bytes[31] << 8) + (message_bytes[32] << 16) + (message_bytes[33] << 24))) * (LUT_NEG_POWERS_OF_TWO[27] * 0.3048);
    message_a2->delta_velocity_y    = (double) ((int32_t) (message_bytes[34] + (message_bytes[35] << 8) + (message_bytes[36] << 16) + (message_bytes[37] << 24))) * (LUT_NEG_POWERS_OF_TWO[27] * 0.3048);
    message_a2->delta_velocity_z    = (double) ((int32_t) (message_bytes[38] + (message_bytes[39] << 8) + (message_bytes[40] << 16) + (message_bytes[41] << 24))) * (LUT_NEG_POWERS_OF_TWO[27] * 0.3048);


    message_a2->checksum = message_bytes[42] + (message_bytes[43] << 8);

    return 0;
}

uint8_t ProcessHGuidei300(HGuidei300Imu_t *imu, RingBuffer_t *data)
{
    uint8_t trash[1];
    while (1)
    {
        if (data->count < 44)
        {
            break;
        }

        if (data->buffer[data->tail] == 0x0E && data->buffer[data->tail + 1] == 0xA1)
        {
            MessageA1_t _control_message;
            MessageA1_t *control_message = &_control_message;
            DeserializeMessageA1(data, control_message);

            imu->angular_rate_x         = control_message->angular_rate_x;
            imu->angular_rate_y         = control_message->angular_rate_y;
            imu->angular_rate_z         = control_message->angular_rate_z;
            imu->linear_acceleration_x  = control_message->linear_acceleration_x;
            imu->linear_acceleration_y  = control_message->linear_acceleration_y;
            imu->linear_acceleration_z  = control_message->linear_acceleration_z;
        }

        else if (data->buffer[data->tail] == 0x0E && data->buffer[data->tail + 1] == 0xA2)
        {
            MessageA2_t _inertial_message;
            MessageA2_t *inertial_message = &_inertial_message;
            DeserializeMessageA2(data, inertial_message);

            imu->angular_rate_x         = inertial_message->angular_rate_x;
            imu->angular_rate_y         = inertial_message->angular_rate_y;
            imu->angular_rate_z         = inertial_message->angular_rate_z;
            imu->linear_acceleration_x  = inertial_message->linear_acceleration_x;
            imu->linear_acceleration_y  = inertial_message->linear_acceleration_y;
            imu->linear_acceleration_z  = inertial_message->linear_acceleration_z;
            imu->delta_angle_x          = inertial_message->delta_angle_x;
            imu->delta_angle_y          = inertial_message->delta_angle_y;
            imu->delta_angle_z          = inertial_message->delta_angle_z;
            imu->delta_velocity_x       = inertial_message->delta_velocity_x;
            imu->delta_velocity_y       = inertial_message->delta_velocity_y;
            imu->delta_velocity_z       = inertial_message->delta_velocity_z;
        }

        else
        {
            RingBuffer_Get(data, trash, 1);
        }
    }

    return 0;
}

double  GetAngularRateX(HGuidei300Imu_t *imu)
{
    return imu->angular_rate_x;
}

double  GetAngularRateY(HGuidei300Imu_t *imu)
{
    return imu->angular_rate_y;
}

double  GetAngularRateZ(HGuidei300Imu_t *imu)
{
    return imu->angular_rate_z;
}

double  GetLinearAccelerationX(HGuidei300Imu_t *imu)
{
    return imu->linear_acceleration_x;
}

double  GetLinearAccelerationY(HGuidei300Imu_t *imu)
{
    return imu->linear_acceleration_y;
}

double  GetLinearAccelerationZ(HGuidei300Imu_t *imu)
{
    return imu->linear_acceleration_z;
}

double  GetDeltaAngleX(HGuidei300Imu_t *imu)
{
    return imu->delta_angle_x;
}

double  GetDeltaAngleY(HGuidei300Imu_t *imu)
{
    return imu->delta_angle_y;
}

double  GetDeltaAngleZ(HGuidei300Imu_t *imu)
{
    return imu->delta_angle_z;
}

double  GetDeltaVelocityX(HGuidei300Imu_t *imu)
{
    return imu->delta_velocity_x;
}

double  GetDeltaVelocityY(HGuidei300Imu_t *imu)
{
    return imu->delta_velocity_y;
}

double  GetDeltaVelocityZ(HGuidei300Imu_t *imu)
{
    return imu->delta_velocity_z;
}

