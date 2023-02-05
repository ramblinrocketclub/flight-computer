#include "fast1.h"

#include "states.h"

void fast1_initialize(double currentTimestampSec) {
    printf("Entered fast1 state");
}

State *fast1_execute(double currentTimestampSec) {
    if (rocket.fsv.time_since_launch_seconds > STAGING_TIME_THRESH_SEC) {
        return &pre_stage;
    }

    return &fast1;
}

void fast1_finish(double currentTimestampSec) {
    
}