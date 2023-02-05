#include "post_stage.h"

#include "states.h"

void post_stage_initialize(double currentTimestampSec) {
    printf("Entered post stage state");
}

State *post_stage_execute(double currentTimestampSec) {
    if (rocket.fsv.time_since_launch_seconds >= 
        rocket.booster_burnout_time_sec + SUSTAINER_IGNITION_TIME_OFFSET_SEC) {
        return &sustainer_ignition;
    }

    return &post_stage;
}

void post_stage_finish(double currentTimestampSec) {
    
}