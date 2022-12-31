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

Quaternion update_local_orientation(Quaternion *initialPose, float32_t gXRPS, float32_t gYRPS, float32_t gZRPS, 
                                float32_t dtSec);

#endif /* IMU_MATH_HELPER_H */