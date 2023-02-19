#include <stdint.h>
#include <stddef.h>
#include "rocket.h"
#include "util.h"

#include "event_constants.h"
#include "states/flight_state_variables.h"
#include "rolling_window.h"
#include "arm_math.h"

void init_rocket(Rocket *rkt, GPS_t *gpsData) {
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

    rkt->fsv.last_predict_time_seconds = -1;

    init_flight_state_variables(&rkt->fsv);

    rkt->starting_launch_altitude_meters = gpsData->altitude_meters;

    rkt->start_launch_timestamp_sec = -1;
    rkt->booster_burnout_time_sec = -1;

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

void update_rocket_state_variables(Rocket *rkt, double currentTimestampSec, HGuideIMU_t *hguideData, GPS_t *gpsData) {
    if (rkt->start_launch_timestamp_sec >= 0) {
        rkt->fsv.time_since_launch_seconds = currentTimestampSec - rkt->start_launch_timestamp_sec;
    }

    if (hguideData != NULL) {
        if (rkt->fsv.last_predict_time_seconds >= 0) {
            double dt = currentTimestampSec - rkt->fsv.last_predict_time_seconds;

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

            ARM_CHECK_STATUS(get_world_rotation_matrix(&rkt->hguide_local_to_world_3x3, &rkt->hguide_local_orientation, &rkt->hguide_world_orientation_3x3));

            // Transform linear accelerations
            rkt->hguide_axyz_local_f32[0] = GetLinearAccelerationXMsec2(hguideData);
            rkt->hguide_axyz_local_f32[1] = GetLinearAccelerationYMsec2(hguideData);
            rkt->hguide_axyz_local_f32[2] = GetLinearAccelerationZMsec2(hguideData);

            ARM_CHECK_STATUS(arm_mat_mult_f32(&rkt->hguide_world_orientation_3x3, &rkt->hguide_axyz_local, &rkt->hguide_axyz_world));

            add_data_point_rolling_window(&rkt->fsv.vertical_acceleration_msec2_rw, rkt->hguide_axyz_world_f32[2]);

            double up[3] = { 0.0, 0.0, 1.0 };
            double orientation_vector[3] = { 0.0, 0.0, 0.0 };

            rotate_vector(&rkt->hguide_local_orientation, up, orientation_vector);

            double dot_product = up[0] * orientation_vector[0] + up[1] * orientation_vector[1] + up[2] * orientation_vector[2];
            double up_magnitude = 1.0;

            float32_t orientation_magnitude;

            ARM_CHECK_STATUS(arm_sqrt_f32((float32_t)(orientation_vector[0] * orientation_vector[0]
                                            + orientation_vector[1] * orientation_vector[1]
                                            + orientation_vector[2] * orientation_vector[2]), &orientation_magnitude));

            double cos_theta = dot_product / (up_magnitude * (double)orientation_magnitude);

            // float32_t tan_theta = 1.0f;

            // ARM_CHECK_STATUS(arm_sqrt_f32((1.0 / (cos_theta * cos_theta)) - 1.0, &tan_theta));

            // float32_t theta_rad = 0.0f;

            // ARM_CHECK_STATUS(arm_atan2_f32(tan_theta, 1.0f, &theta_rad));

            double theta_rad = acos(cos_theta);

            add_data_point_rolling_window(&rkt->fsv.tilt_radians_rw, theta_rad);

            float32_t un_f32[1] = { (float32_t)(get_vertical_accel_msec2(&rkt->fsv)) };

            ARM_CHECK_STATUS(predict_kalman_filter(&rkt->kf, un_f32));

            rkt->fsv.last_predict_time_seconds = currentTimestampSec;

            if (gpsData == NULL) {
                // Add state estimates immediately as there is no GPS data to correct
                add_data_point_rolling_window(&rkt->fsv.vertical_position_m_rw, rkt->xHat_f32[0]);
                add_data_point_rolling_window(&rkt->fsv.vertical_velocity_msec_rw, rkt->xHat_f32[1]);
            }
        } else {
            rkt->fsv.last_predict_time_seconds = currentTimestampSec;
        }
    }

    if (gpsData != NULL) {
        float32_t zn_f32[1] = { (float32_t)(gpsData->altitude_meters - rkt->starting_launch_altitude_meters) };
        float32_t H_f32[2] = { 1, 0 }; // Position factor, velocity factor

        // TODO: eventually read the actual standard deviation from the GPS statistics frame
        float32_t measurementStdDevs[1] = { (float32_t)(rkt->gps_altitude_std_m) };

        ARM_CHECK_STATUS(correct_kalman_filter(&rkt->kf, 1, zn_f32, H_f32, measurementStdDevs));

        add_data_point_rolling_window(&rkt->fsv.vertical_position_m_rw, rkt->xHat_f32[0]);
        add_data_point_rolling_window(&rkt->fsv.vertical_velocity_msec_rw, rkt->xHat_f32[1]);
    }
}

