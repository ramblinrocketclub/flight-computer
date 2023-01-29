#include <stdint.h>
#include <stddef.h>
#include "rocket.h"
#include "util.h"

#include "event_constants.h"

void init_rocket(Rocket *rkt, double timestamp, GPS_t *gpsData) {
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

    rkt->fsv.time_since_launch_seconds = 0.0;
    rkt->fsv.last_predict_time_seconds = timestamp;
    rkt->fsv.vertical_acceleration_msec2 = 0.0;
    rkt->fsv.vertical_velocity_msec = 0.0;
    rkt->fsv.vertical_position_m = 0.0;
    rkt->fsv.tilt_radians = 0.0;

    rkt->starting_launch_altitude_meters = gpsData->altitude_meters;

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

    rkt->has_calibrated = false;
}

void calibrate_rocket(Rocket *rkt, HGuideIMU_t *hguideData) {
    arm_mat_init_f32(&rkt->hguide_local_to_world_3x3, 3, 3, rkt->hguide_local_to_world_3x3_f32);

    float32_t Axyz[3] = {
        GetLinearAccelerationXMsec2(hguideData),
        GetLinearAccelerationYMsec2(hguideData),
        GetLinearAccelerationZMsec2(hguideData)
    };

    calibrate_imu(Axyz, &rkt->hguide_local_to_world_3x3);

    init_quaternion_xyzw(&rkt->hguide_local_orientation, 0, 0, 0, 1);

    arm_mat_init_f32(&rkt->hguide_world_orientation_3x3, 3, 3, rkt->hguide_world_orientation_3x3_f32);

    arm_mat_init_f32(&rkt->hguide_axyz_local, 3, 1, rkt->hguide_axyz_local_f32);
    arm_mat_init_f32(&rkt->hguide_axyz_world, 3, 1, rkt->hguide_axyz_world_f32);

    rkt->has_calibrated = true;
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

        double wx = GetAngularRateXRadPerSec(hguideData);
        double wy = GetAngularRateYRadPerSec(hguideData);
        double wz = GetAngularRateZRadPerSec(hguideData);

        // Update orientation
        rkt->hguide_local_orientation = update_local_orientation(&rkt->hguide_local_orientation, wx, wy, wz, dt);

        get_world_rotation_matrix(&rkt->hguide_local_to_world_3x3, &rkt->hguide_local_orientation, &rkt->hguide_world_orientation_3x3);

        // Transform linear accelerations
        rkt->hguide_axyz_local_f32[0] = GetLinearAccelerationXMsec2(hguideData);
        rkt->hguide_axyz_local_f32[1] = GetLinearAccelerationYMsec2(hguideData);
        rkt->hguide_axyz_local_f32[2] = GetLinearAccelerationZMsec2(hguideData);

        ARM_CHECK_STATUS(arm_mat_mult_f32(&rkt->hguide_world_orientation_3x3, &rkt->hguide_axyz_local, &rkt->hguide_axyz_world));

        rkt->fsv.vertical_acceleration_msec2 = rkt->hguide_axyz_world_f32[2] - GRAVITY_CONSTANT_MSEC2;

        float32_t un_f32[1] = { (float32_t)(rkt->fsv.vertical_acceleration_msec2) };

        ARM_CHECK_STATUS(predict_kalman_filter(&rkt->kf, un_f32));

        rkt->fsv.last_predict_time_seconds = currentTimeS;
    }

    if (gpsData != NULL) {
        float32_t zn_f32[1] = { (float32_t)(gpsData->altitude_meters - rkt->starting_launch_altitude_meters) };
        float32_t H_f32[2] = { 1, 0 }; // Position factor, velocity factor

        // TODO: eventually read the actual standard deviation from the GPS statistics frame
        float32_t measurementStdDevs[1] = { (float32_t)(rkt->gps_altitude_std_m) };

        ARM_CHECK_STATUS(correct_kalman_filter(&rkt->kf, 1, zn_f32, H_f32, measurementStdDevs));
    }
}

