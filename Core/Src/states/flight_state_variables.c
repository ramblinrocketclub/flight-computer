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

double get_vertical_accel_msec2(FlightStateVariables *fsv) {
    return get_latest_datapoint_rolling_window(&fsv->vertical_acceleration_msec2_rw);
}

double get_vertical_vel_msec(FlightStateVariables *fsv) {
    return get_latest_datapoint_rolling_window(&fsv->vertical_velocity_msec_rw);
}

double get_vertical_pos_m(FlightStateVariables *fsv) {
    return get_latest_datapoint_rolling_window(&fsv->vertical_position_m_rw);
}

double get_tilt_radians(FlightStateVariables *fsv) {
    return get_latest_datapoint_rolling_window(&fsv->tilt_radians_rw);
}