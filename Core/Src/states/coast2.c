#include "coast2.h"

#include "states.h"

double averageVelocity = 0;
double count = 0;
int sampleSize = APOGEE_CHECK_FREQ * APOGEE_CHECK_TIME_SEC;

void coast2_initialize(double currentTimestampSec) {
    printf("Entered coast2 state");
    rocket.velocity_check_time_sec = currentTimestampSec;
}

State *coast2_execute(double currentTimestampSec) {
    //move on to recovery if apogee detected   
    if (currentTimestampSec >= rocket.velocity_check_time_sec + APOGEE_CHECK_FREQ) {
        averageVelocity += get_vertical_vel_msec(&rocket.fsv) / sampleSize;
        count++;
        //if 
        if (count >= sampleSize) {
            if (averageVelocity < 0) {
                return &recovery;
            }
            averageVelocity = 0;
            count = 0;
            rocket.velocity_check_time_sec = currentTimestampSec;
        }
    }

    //move on to recovery if max chute deploy time reached
    if (rocket.fsv.time_since_launch_seconds > CHUTE_DEPLOY_TIME_SEC) {
        return &recovery;
    }

    return &coast2;
}

void coast2_finish(double currentTimestampSec) {
    //TODO: deploy parachute
}