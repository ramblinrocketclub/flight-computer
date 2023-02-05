#include "pre_stage.h"

#include "states.h"

void pre_stage_initialize(double currentTimestampSec) {
    printf("Entered pre stage state");
}

State *pre_stage_execute(double currentTimestampSec) {
    if (rocket.fsv.time_since_launch_seconds > STAGING_TIME_THRESH_SEC
        && get_vertical_accel_msec2(&rocket.fsv) < STAGING_ACCEL_THRESH_MSEC2) {
        return &post_stage;
    }

    return &pre_stage;
}

void pre_stage_finish(double currentTimestampSec) {
    // TODO: Fire staging action command here!
}