#include <stdint.h>
#include <stddef.h>

#include "flight_state_variables.h"
#include "rolling_window.h"

void init_flight_state_variables(FlightStateVariables *fsv) {
    fsv->time_since_launch_seconds = 0.0;
    fsv->last_predict_time_seconds = 0.0;

    init_rolling_window(&fsv->vertical_acceleration_msec2_rw, fsv->vertical_acceleration_msec2_arr, ROLLING_WINDOW_SIZE);
    init_rolling_window(&fsv->vertical_velocity_msec_rw, fsv->vertical_velocity_msec_arr, ROLLING_WINDOW_SIZE);
    init_rolling_window(&fsv->vertical_position_m_rw, fsv->vertical_position_m_arr, ROLLING_WINDOW_SIZE);
    init_rolling_window(&fsv->tilt_radians_rw, fsv->tilt_radians_arr, ROLLING_WINDOW_SIZE);
}