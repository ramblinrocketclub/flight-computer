#ifndef ROCKET_H
#define ROCKET_H

#include "hguide_imu.h"
#include "gps.h"
#include "state_machine.h"
#include "flight_state_variables.h"
#include "kalman_filter.h"
#include "imu_math_helper.h"

#define NOMINAL_DT_SECONDS 0.02
#define INITIAL_HEIGHT     0.00
#define INITIAL_VELOCITY   0.00
#define INITIAL_POS_VAR    0.01
#define INITIAL_VEL_VAR    0.01

// Contains properties related to the rocket
typedef struct Rocket {
    // The kalman filter is responsible for filtering sensor data
    // to produce an estimate of flight state variables
    KalmanFilter kf;

    // Kalman matrix backing arrays
    float32_t F_f32[4];
    float32_t G_f32[2];
    float32_t P_f32[4];
    float32_t Q_f32[4];
    float32_t xHat_f32[2];

    // Flight state variables are state estimates of the system
    FlightStateVariables fsv;

    // 3x3 DCM to convert from casing frame to world frame
    float32_t hguide_local_to_world_3x3_f32[9];
    // Rotation matrix representing the orientation of the IMU in the world frame
    float32_t hguide_world_orientation_3x3_f32[9];
    // Acceleration vector in the local casing frame
    float32_t hguide_axyz_local_f32[3];
    // Acceleration vector in the world frame
    float32_t hguide_axyz_world_f32[3];

    arm_matrix_instance_f32 hguide_local_to_world_3x3;
    Quaternion hguide_local_orientation;
    arm_matrix_instance_f32 hguide_world_orientation_3x3;

    arm_matrix_instance_f32 hguide_axyz_local;
    arm_matrix_instance_f32 hguide_axyz_world;

    // Standard deviation constants for sensors
    double hguide_vertical_accel_std_msec2;  // meters^2/s
    double gps_altitude_std_m;               // meters

    // The altitude at which the rocket starts at
    double startingLaunchAltitude;
} Rocket;

// Initialize rocket properties
void init_rocket(Rocket *rkt, double timestamp, GPS_t *gpsData);

// Find calibration matrices for sensors
void calibrate_rocket(Rocket *rkt, HGuideIMU_t *hguideData);

// Run a kalman predict step (and correct step if applicable)
void update_rocket_state_variables(Rocket *rkt, double currentTimeS, HGuideIMU_t *hguideData, GPS_t *gpsData);

#endif /* ROCKET_H */