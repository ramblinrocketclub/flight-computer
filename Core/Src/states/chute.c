#include "chute.h"

#include "states.h"

void chute_initialize(double currentTimestampSec) {
    printf("Entered chute state");
}

State *chute_execute(double currentTimestampSec) {
    if (get_vertical_pos_m(&rocket.fsv) < LANDED_HEIGHT_THRESH_M) {
        return &landed;
    }

    return &chute;
}

void chute_finish(double currentTimestampSec) {
    
}