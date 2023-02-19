#include "coast1.h"

#include "states.h"

void coast1_initialize(double currentTimestampSec) {
    printf("Entered coast1 state");
}

State *coast1_execute(double currentTimestampSec) {
    //move on to boost2 when time since launch reaches staging threshold AND tilt condition is met
    if (rocket.fsv.time_since_launch_seconds >= STAGING_TIME_THRESH_SEC + BOOSTER_BURNOUT_TIME_SEC &&
        get_tilt_radians(&rocket.fsv) < MAX_SUSTAINER_IGN_TILT) {
        return &boost2;
    //move on to coast2 if time exceeds staging time limit
    } else if (rocket.fsv.time_since_launch_seconds >= STAGING_TIME_LIMIT_SEC) {
        return &coast2;
    }

    return &coast1;
}

void coast1_finish(double currentTimestampSec) {
    rocket.sustainer_ignition_time_sec = currentTimestampSec;
    //TODO: sustainer ignition
}