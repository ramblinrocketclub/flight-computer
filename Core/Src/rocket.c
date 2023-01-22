#include "rocket.h"

#include "util.h"

void init_rocket(Rocket *rkt) {
    double msec2_per_microg = 1.0 / 101971.62129779282;

    // Datasheets used for the following:
    // N61-2009-000-003_i300-IMU-bro.pdf
    // https://content.u-blox.com/sites/default/files/ZED-F9R-01B_Datasheet_UBX-19054459.pdf
    rkt->hguide_vertical_accel_std_msec2 = 1.0 * msec2_per_microg;
    rkt->gps_altitude_std_m = 0.01;

    rkt->F_f32[0] = 1.0;
    rkt->F_f32[1] = NOMINAL_DT_SECONDS;
    rkt->F_f32[2] = 0.0;
    rkt->F_f32[3] = 1.0;

    rkt->G_f32[0] = 0.5 * NOMINAL_DT_SECONDS * NOMINAL_DT_SECONDS;
    rkt->G_f32[1] = NOMINAL_DT_SECONDS;

    rkt->P_f32[0] = INITIAL_POS_VAR;
    rkt->P_f32[1] = 0.0;
    rkt->P_f32[2] = 0.0;
    rkt->P_f32[3] = INITIAL_VEL_VAR;

    rkt->Q_f32[0] = 1.0;
    rkt->Q_f32[1] = 0.0;
    rkt->Q_f32[2] = 0.0;
    rkt->Q_f32[3] = 1.0;

    rkt->xHat_f32[0] = INITIAL_HEIGHT;
    rkt->xHat_f32[1] = INITIAL_VELOCITY;

    double accelVar = rkt->hguide_vertical_accel_std_msec2 * rkt->hguide_vertical_accel_std_msec2;

    float32_t state_std_devs_f32[2] = {
        // Position std, velocity std
        NOMINAL_DT_SECONDS * NOMINAL_DT_SECONDS / 2.0 * accelVar, 
        NOMINAL_DT_SECONDS * accelVar
    };

    ARM_CHECK_STATUS(init_kalman_filter(&rkt->kf, 2, 1, 
                                &rkt->F_f32[0], &rkt->G_f32[0], 
                                &rkt->P_f32[0], &rkt->Q_f32[0], 
                                &rkt->xHat_f32[0], &state_std_devs_f32[0]));
}

void calibrate_rocket(Rocket *rkt) {

}

void update_rocket_state_variables(Rocket *rkt, double currentTimeS, HGuideIMU_t *hguideData, GPS_t *gpsData) {
    if (hguideData != NULL) {
        double dt = currentTimeS - rkt->fsv.last_predict_time_seconds;

        // Update matrices with new dt
        rkt->kf.F.pData[0] = 1.0f;
        rkt->kf.F.pData[1] = dt;
        rkt->kf.F.pData[2] = 0.0f;
        rkt->kf.F.pData[3] = 1.0f;

        rkt->kf.G.pData[0] = 0.5 * dt * dt;
        rkt->kf.G.pData[1] = dt;

        double accelVar = rkt->hguide_vertical_accel_std_msec2 * rkt->hguide_vertical_accel_std_msec2;

        rkt->kf.Q.pData[0] = (dt * dt * dt * dt) / 4.0 * accelVar;
        rkt->kf.Q.pData[1] = (dt * dt * dt) / 2.0 * accelVar;
        rkt->kf.Q.pData[2] = (dt * dt * dt) / 2.0 * accelVar;
        rkt->kf.Q.pData[3] = dt * dt * accelVar;   

        rkt->fsv.last_predict_time_seconds = currentTimeS;
    }
}