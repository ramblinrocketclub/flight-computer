#ifndef IMU_MATH_HELPER_H
#define IMU_MATH_HELPER_H

#include <stdint.h>

#include "arm_math.h"

typedef struct Quaternion {
    double x, y, z, w;
} Quaternion;

void init_quaternion_xyzw(Quaternion *quat, double x, double y, double z, double w);
double get_norm_quaternion(const Quaternion *quat);
double get_length_quaternion(const Quaternion *quat);
Quaternion normalize_quaternion(const Quaternion *quat);
Quaternion mult_quaternions(const Quaternion *a, const Quaternion *b);
void quaternion_to_matrix(const Quaternion *quat, arm_matrix_instance_f32 *dst3x3);
Quaternion matrix_to_quaternion(const arm_matrix_instance_f32 *r3x3);
void rotate_vector(const Quaternion *quat, double vectorXYZ[], double dstXYZ[]);
Quaternion get_conjugate_quaternion(const Quaternion *quat);

void calibrate_imu(float32_t Axyz[], arm_matrix_instance_f32 *dst3x3);
Quaternion update_local_orientation(Quaternion *initialPose, double gXRPS, double gYRPS, double gZRPS, 
                                double dtSec);
arm_status get_world_rotation_matrix(arm_matrix_instance_f32 *R_cal, Quaternion *q_imu, 
                                arm_matrix_instance_f32 *dst3x3);

#endif /* IMU_MATH_HELPER_H */