#ifndef UPDATE_STATE_VARIABLES_H
#define UPDATE_STATE_VARIABLES_H

#include "kalman_filter.h"
#include "flight_state_variables.h"

void update_state_variables(double currentTimeS, double accelVar, KalmanFilter *kf, FlightStateVariables *sv);

#endif /* UPDATE_STATE_VARIABLES_H */