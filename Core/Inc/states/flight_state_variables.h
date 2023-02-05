#ifndef FLIGHT_STATE_VARIABLES_H
#define FLIGHT_STATE_VARIABLES_H

#include "rolling_window.h"

#define ROLLING_WINDOW_SIZE 100

// Contains all of the information needed to make flight decisions
typedef struct FlightStateVariables {
    double time_since_launch_seconds;
    double last_predict_time_seconds;

    double vertical_acceleration_msec2_arr[ROLLING_WINDOW_SIZE];
    double vertical_velocity_msec_arr[ROLLING_WINDOW_SIZE];
    double vertical_position_m_arr[ROLLING_WINDOW_SIZE]; // This position is wrt starting launch height
    double tilt_radians_arr[ROLLING_WINDOW_SIZE];

    RollingWindow vertical_acceleration_msec2_rw;
    RollingWindow vertical_velocity_msec_rw;
    RollingWindow vertical_position_m_rw; // This position is wrt starting launch height
    RollingWindow tilt_radians_rw;
} FlightStateVariables;

void init_flight_state_variables(FlightStateVariables *fsv);
double get_vertical_accel_msec2(FlightStateVariables *fsv);
double get_vertical_vel_msec(FlightStateVariables *fsv);
double get_vertical_pos_m(FlightStateVariables *fsv);
double get_tilt_radians(FlightStateVariables *fsv);

#endif /* FLIGHT_STATE_VARIABLES_H */