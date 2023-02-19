#include "coast2.h"

#include "states.h"

void coast2_initialize(double currentTimestampSec) {
    printf("Entered coast2 state");
}

State *coast2_execute(double currentTimestampSec) {
    //move on to recovery if apogee detected (average negative velocity for five seconds) OR max chute deploy time reached
    //TODO: apogee detection
    /*if ( || rocket.fsv.time_since_launch_seconds > CHUTE_DEPLOY_TIME_SEC) {
        return &recovery;
    }*/
    return &coast2;
}

void coast2_finish(double currentTimestampSec) {
    //TODO: deploy parachute
}