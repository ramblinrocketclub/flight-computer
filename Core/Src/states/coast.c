#include "coast.h"

#include "states.h"

void coast_initialize(double currentTimestampSec) {
    printf("Entered coast state");
}

State *coast_execute(double currentTimestampSec) {
    if (get_vertical_vel_msec(&rocket.fsv) < 0) {
        return &chute;
    }

    return &coast;
}

void coast_finish(double currentTimestampSec) {
    // TODO: Fire parachute deployment action here!
}