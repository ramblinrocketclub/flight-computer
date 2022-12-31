#include "imu_math_helper.h"

void init_quaternion_xyzw(Quaternion *quat, double x, double y, double z, double w) {
    quat->x = x;
    quat->y = y;
    quat->z = z;
    quat->w = w;
}

double get_norm_quaternion(const Quaternion *quat) {
    double result;
    result = (quat->x * quat->x);
    result = (result + (quat->y * quat->y));
    result = (result + (quat->z * quat->z));
    result = (result + (quat->w * quat->w));
    return result;
}

double get_length_quaternion(const Quaternion *quat) {
    float32_t result;

    arm_sqrt_f32((float32_t)get_norm_quaternion(quat), &result);

    return (double)result;
}

Quaternion normalize_quaternion(const Quaternion *quat) {
    Quaternion result;

    init_quaternion_xyzw(&result, quat->x, quat->y, quat->z, quat->w);

    double lenSqr;
    float32_t len;

	lenSqr = get_norm_quaternion(quat);

    arm_sqrt_f32((float32_t)lenSqr, &len);

    result.x /= len;
    result.y /= len;
    result.z /= len;
    result.w /= len;

	return result;
}

Quaternion mult_quaternions(const Quaternion *a, const Quaternion *b) {
    Quaternion result;

    init_quaternion_xyzw(&result, 
            (((a->w * b->x) + (a->x * b->w)) + (a->y * b->z)) - (a->z * b->y),
			(((a->w * b->y) + (a->y * b->w)) + (a->z * b->x)) - (a->x * b->z),
			(((a->w * b->z) + (a->z * b->w)) + (a->x * b->y)) - (a->y * b->x),
			(((a->w * b->w) - (a->x * b->x)) - (a->y * b->y)) - (a->z * b->z)
    );

    return normalize_quaternion(&result);
}

Quaternion update_local_orientation(Quaternion *initialPose, float32_t gXRPS, float32_t gYRPS, float32_t gZRPS, 
                                float32_t dtSec) {
    Quaternion deltaQuat;

    init_quaternion_xyzw(&deltaQuat, 1, 0.5 * gXRPS * dtSec, 0.5 * gYRPS * dtSec, 0.5 * gZRPS * dtSec);

    return mult_quaternions(initialPose, &deltaQuat);
}