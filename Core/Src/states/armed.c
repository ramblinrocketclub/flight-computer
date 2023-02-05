#include "armed.h"

#include "states.h"

void armed_initialize(double currentTimestampSec) {
    printf("Entered armed state");
}

State *armed_execute(double currentTimestampSec) {
    bool has_booster_ignited = false;

    if (has_booster_ignited) {
        return &boost1;
    }

    return &armed;
}

void armed_finish(double currentTimestampSec) {
    
}