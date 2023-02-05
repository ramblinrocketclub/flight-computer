#include "boost2.h"

#include "states.h"

void boost2_initialize(double currentTimestampSec) {
    printf("Entered boost2 state");
}

State *boost2_execute(double currentTimestampSec) {
    if (rocket.fsv.time_since_launch_seconds >= SUSTAINER_BURNOUT_TIME_SEC
        && get_vertical_accel_msec2(&rocket.fsv) < SUSTAINER_BURNOUT_ACCEL_THRESH_MSEC2) {
        return &fast2;
    }

    return &boost2;
}

void boost2_finish(double currentTimestampSec) {
    
}