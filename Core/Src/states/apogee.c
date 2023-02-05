#include "apogee.h"

#include "states.h"

#include "rolling_window.h"

void apogee_initialize(double currentTimestampSec) {
    printf("Entered apogee state");
}

State *apogee_execute(double currentTimestampSec) {
    // Detect if we are falling

    if ((get_earliest_datapoint_rolling_window(&rocket.fsv.vertical_position_m_rw) 
        - get_latest_datapoint_rolling_window(&rocket.fsv.vertical_position_m_rw)
            > APOGEE_HEIGHT_DIFF_THRESH_M) 
            || get_vertical_vel_msec(&rocket.fsv) < FALLING_VEL_THRESH_MSEC) {
        return &coast;
    }

    return &apogee;
}

void apogee_finish(double currentTimestampSec) {

}