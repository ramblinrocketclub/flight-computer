#include "sustainer_ignition.h"

#include "states.h"

#include <math.h>

void sustainer_ignition_initialize(double currentTimestampSec) {
    printf("Entered sustainer ignition state");
}

State *sustainer_ignition_execute(double currentTimestampSec) {
    if (rocket.fsv.time_since_launch_seconds >= 
        rocket.booster_burnout_time_sec + SUSTAINER_IGNITION_TIME_OFFSET_SEC
        && fabs(get_tilt_radians(&rocket.fsv)) < PI) {
        return &boost2;
    }

    return &sustainer_ignition;
}

void sustainer_ignition_finish(double currentTimestampSec) {
    // TODO: Fire ignite sustainer action here!
}