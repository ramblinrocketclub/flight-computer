#include "boost1.h"

#include "states.h"

void boost1_initialize(double currentTimestampSec) {
    printf("Entered boost1 state");
    //start time since launch counter
    rocket.start_launch_timestamp_sec = currentTimestampSec;
}

State *boost1_execute(double currentTimestampSec) {
    //move on to coast1 when time since launch reaches booster burnout threshold
    if (rocket.fsv.time_since_launch_seconds >= BOOSTER_BURNOUT_TIME_SEC) {
        return &coast1;
    }

    return &boost1;
}

void boost1_finish(double currentTimestampSec) {
    rocket.booster_burnout_time_sec = currentTimestampSec;
    //TODO: Stage
}