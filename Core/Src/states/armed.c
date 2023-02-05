#include "armed.h"

#include "states.h"

void armed_initialize(double currentTimestampSec) {
    printf("Entered armed state");
}

State *armed_execute(double currentTimestampSec) {
    if (get_vertical_accel_msec2(&rocket.fsv) > BOOSTER_IGNITION_ACCEL_THRESH_MSEC2) {
        return &boost1;
    }

    return &armed;
}

void armed_finish(double currentTimestampSec) {
    
}