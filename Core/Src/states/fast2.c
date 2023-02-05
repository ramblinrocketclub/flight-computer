#include "fast2.h"

#include "states.h"

void fast2_initialize(double currentTimestampSec) {
    printf("Entered fast2 state");
}

State *fast2_execute(double currentTimestampSec) {
    // Detect if getting close to apogee
    if (get_vertical_pos_m(&rocket.fsv) > APOGEE_HEIGHT_THRESH_M) {
        return &apogee;
    }

    return &fast2;
}

void fast2_finish(double currentTimestampSec) {
    
}