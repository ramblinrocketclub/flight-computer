#include "update_state_variables.h"

void update_state_variables(double currentTimeS, double accelVar, KalmanFilter *kf, FlightStateVariables *sv) {
    double dt = currentTimeS - sv->last_update_time_seconds;

    // Update matrices with new dt
    kf->F.pData[0] = 1.0f;
    kf->F.pData[1] = dt;
    kf->F.pData[2] = 0.0f;
    kf->F.pData[3] = 1.0f;

    kf->G.pData[0] = 0.5 * dt * dt;
    kf->G.pData[1] = dt;

    kf->Q.pData[0] = (dt * dt * dt * dt) / 4.0 * accelVar;
    kf->Q.pData[1] = (dt * dt * dt) / 2.0 * accelVar;
    kf->Q.pData[2] = (dt * dt * dt) / 2.0 * accelVar;
    kf->Q.pData[3] = dt * dt * accelVar;    
}