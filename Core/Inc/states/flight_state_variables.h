#ifndef FLIGHT_STATE_VARIABLES_H
#define FLIGHT_STATE_VARIABLES_H

// Contains all of the information needed to make flight decisions
typedef struct FlightStateVariables {
    double time_since_launch_seconds;
    double last_predict_time_seconds;
    double vertical_acceleration_msec2;
    double vertical_velocity_msec;
    double vertical_position_m; // This position is wrt starting launch height
    double tilt_radians;
} FlightStateVariables;

#endif /* FLIGHT_STATE_VARIABLES_H */