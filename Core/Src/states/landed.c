#include "landed.h"

#include "states.h"

void landed_initialize(double currentTimestampSec) {
    printf("Entered landed state");
}

State *landed_execute(double currentTimestampSec) {
    return &landed;
}

void landed_finish(double currentTimestampSec) {
    
}