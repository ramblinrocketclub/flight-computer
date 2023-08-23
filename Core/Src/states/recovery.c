#include "recovery.h"

#include "states.h"

void recovery_initialize(double currentTimestampSec) {
    printf("Entered recovery state");
}

State *recovery_execute(double currentTimestampSec) {
    //move on to landed when altitude falls below landed height threshold
    if (get_vertical_pos_m(&rocket.fsv) <= LANDED_HEIGHT_THRESH_M) {
        return &landed;
    }
}

void recovery_finish(double currentTimestampSec) {
    
}