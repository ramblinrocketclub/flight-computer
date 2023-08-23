#include "boost2.h"

#include "states.h"

void boost2_initialize(double currentTimestampSec) {
    printf("Entered boost2 state");
}

State *boost2_execute(double currentTimestampSec) {
    //move on to coast2 when time since launch reaches sustainer burnout threshold
    if (rocket.fsv.time_since_launch_seconds >= SUSTAINER_BURNOUT_TIME_SEC + rocket.sustainer_ignition_time_sec) {
        return &coast2;
    }

    return &boost2;
}

void boost2_finish(double currentTimestampSec) {
    
}