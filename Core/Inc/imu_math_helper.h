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

Quaternion update_local_orientation(Quaternion *initialPose, double gXRPS, double gYRPS, double gZRPS, 
                                double dtSec);

#endif /* IMU_MATH_HELPER_H */