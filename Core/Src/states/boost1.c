#include "boost1.h"

#include "states.h"

void boost1_initialize(double currentTimestampSec) {
    printf("Entered boost1 state");

    rocket.start_launch_timestamp_sec = currentTimestampSec;
}

State *boost1_execute(double currentTimestampSec) {
    if (rocket.fsv.time_since_launch_seconds >= BOOSTER_BURNOUT_TIME_SEC
        && get_vertical_accel_msec2(&rocket.fsv) < BOOSTER_BURNOUT_ACCEL_THRESH_MSEC2) {
        return &fast1;
    }

    return &boost1;
}

void boost1_finish(double currentTimestampSec) {
    rocket.booster_burnout_time_sec = currentTimestampSec;
}