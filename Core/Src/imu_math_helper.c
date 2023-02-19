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
    Quaternion a_norm;
    Quaternion b_norm;
    Quaternion result;

    a_norm = normalize_quaternion(a);
    b_norm = normalize_quaternion(b);

    init_quaternion_xyzw(&result, 
            (((a_norm.w * b_norm.x) + (a_norm.x * b_norm.w)) + (a_norm.y * b_norm.z)) - (a_norm.z * b_norm.y),
			(((a_norm.w * b_norm.y) + (a_norm.y * b_norm.w)) + (a_norm.z * b_norm.x)) - (a_norm.x * b_norm.z),
			(((a_norm.w * b_norm.z) + (a_norm.z * b_norm.w)) + (a_norm.x * b_norm.y)) - (a_norm.y * b_norm.x),
			(((a_norm.w * b_norm.w) - (a_norm.x * b_norm.x)) - (a_norm.y * b_norm.y)) - (a_norm.z * b_norm.z)
    );

    return result;
}

void quaternion_to_matrix(const Quaternion *quat, arm_matrix_instance_f32 *dst3x3) {
    Quaternion quat_norm;

    quat_norm = normalize_quaternion(quat);

    float32_t x = quat_norm.x;
    float32_t y = quat_norm.y;
    float32_t z = quat_norm.z;
    float32_t w = quat_norm.w;

    dst3x3->pData[0] = 1.0f - (2.0f * y * y) - (2.0f * z * z);
    dst3x3->pData[1] = (2.0f * x * y) - (2.0f * w * z); 
    dst3x3->pData[2] = (2.0f * x * z) + (2.0f * w * y);
    dst3x3->pData[3] = (2.0f * x * y) + (2.0f * w * z);
    dst3x3->pData[4] = 1.0f - (2.0f * x * x) - (2.0f * z * z);
    dst3x3->pData[5] = (2.0f * y * z) - (2.0f * w * x);
    dst3x3->pData[6] = (2.0f * x * z) - (2.0f * w * y);
    dst3x3->pData[7] = (2.0f * y * z) + (2.0f * w * x);
    dst3x3->pData[8] = 1.0f - (2.0f * x * x) - (2.0f * y * y);
}

Quaternion matrix_to_quaternion(const arm_matrix_instance_f32 *r3x3) {
    double tr = r3x3->pData[0 * 3 + 0] + r3x3->pData[1 * 3 + 1] + r3x3->pData[2 * 3 + 2];
    
    float32_t sqrt_1_tr;

    arm_sqrt_f32(1.0 + tr, &sqrt_1_tr);

    double w = sqrt_1_tr / 2.0;

    double x = (r3x3->pData[2 * 3 + 1] - r3x3->pData[1 * 3 + 2]) / (4.0 * w);
    double y = (r3x3->pData[0 * 3 + 2] - r3x3->pData[2 * 3 + 0]) / (4.0 * w);
    double z = (r3x3->pData[1 * 3 + 0] - r3x3->pData[0 * 3 + 1]) / (4.0 * w);

    Quaternion result;

    init_quaternion_xyzw(&result, x, y, z, w);

    return result;
}

void rotate_vector(const Quaternion *quat, double vectorXYZ[], double dstXYZ[]) {
    Quaternion quat_norm;

    quat_norm = normalize_quaternion(quat);

    double tmpX, tmpY, tmpZ, tmpW;

    double x = vectorXYZ[0];
    double y = vectorXYZ[1];
    double z = vectorXYZ[2];

    tmpX = (((quat_norm.w * x) + (quat_norm.y * z)) - (quat_norm.z * y));
    tmpY = (((quat_norm.w * y) + (quat_norm.z * x)) - (quat_norm.x * z));
    tmpZ = (((quat_norm.w * z) + (quat_norm.x * y)) - (quat_norm.y * x));
    tmpW = (((quat_norm.x * x) + (quat_norm.y * y)) + (quat_norm.z * z));

    dstXYZ[0] = ((((tmpW * quat_norm.x) + (tmpX * quat_norm.w)) - (tmpY * quat_norm.z)) + (tmpZ * quat_norm.y));
	dstXYZ[1] =	((((tmpW * quat_norm.y) + (tmpY * quat_norm.w)) - (tmpZ * quat_norm.x)) + (tmpX * quat_norm.z));
	dstXYZ[2] = ((((tmpW * quat_norm.z) + (tmpZ * quat_norm.w)) - (tmpX * quat_norm.y)) + (tmpY * quat_norm.x));
}

Quaternion get_conjugate_quaternion(const Quaternion *quat) {
    Quaternion result;

    init_quaternion_xyzw(&result, -quat->x, -quat->y, -quat->z, quat->w);

    return result;
}

void calibrate_imu(float32_t Axyz[], arm_matrix_instance_f32 *dst3x3) {
    float32_t len;

    arm_sqrt_f32(Axyz[0] * Axyz[0] + Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2], &len);

    float32_t Nxyz[3] = {
        Axyz[0] / len, Axyz[1] / len, Axyz[2] / len
    };

    float32_t Ax = Nxyz[0];
    float32_t Ay = Nxyz[1];
    float32_t Az = Nxyz[2];

    dst3x3->pData[0] = (Ay * Ay - Ax * Ax * Az) / (Ax * Ax + Ay * Ay);
    dst3x3->pData[1] = (-Ax * Ay - Ax * Ay * Az) / (Ax * Ax + Ay * Ay);
    dst3x3->pData[2] = Ax;
    dst3x3->pData[3] = (-Ax * Ay - Ax * Ay * Az) / (Ax * Ax + Ay * Ay);
    dst3x3->pData[4] = (Ax * Ax - Ay * Ay * Az) / (Ax * Ax + Ay * Ay);
    dst3x3->pData[5] = Ay;
    dst3x3->pData[6] = -Ax;
    dst3x3->pData[7] = -Ay;
    dst3x3->pData[8] = -Az;
}

Quaternion update_local_orientation(Quaternion *initialPose, double gXRPS, double gYRPS, double gZRPS, 
                                double dtSec) {
    Quaternion deltaQuat;

    init_quaternion_xyzw(&deltaQuat, 0.5 * gXRPS * dtSec, 0.5 * gYRPS * dtSec, 0.5 * gZRPS * dtSec, 1.0);

    Quaternion result;

    result = mult_quaternions(initialPose, &deltaQuat);

    return result;
}

arm_status get_world_rotation_matrix(arm_matrix_instance_f32 *R_cal, Quaternion *q_imu, 
                                arm_matrix_instance_f32 *dst3x3) {
    arm_status result = ARM_MATH_SUCCESS;

    /*
        From ChatGPT:

        1. Convert the quaternion Q_imu (representing the current rotation of the IMU relative 
           to its startup orientation) to a rotation matrix. Let's call this rotation matrix R_imu.

        2. Compute the inverse of R_imu. Let's call the inverse of R_imu as R_inv.

        3. Compute the product of R_inv and R_cal. This will give you the combined transformation 
           matrix T = R_inv * R_cal.

        4. Multiply the new acceleration vector from the IMU by the transformation matrix T to obtain 
           the net acceleration vector in the global frame where gravity points downward.
    */

    float32_t R_imu_f32[3 * 3];
    float32_t R_inv_f32[3 * 3];

    arm_matrix_instance_f32 R_imu;
    arm_matrix_instance_f32 R_inv;

    arm_mat_init_f32(&R_imu, 3, 3, R_imu_f32);
    arm_mat_init_f32(&R_inv, 3, 3, R_inv_f32);

    quaternion_to_matrix(q_imu, &R_imu);

    result |= arm_mat_inverse_f32(&R_imu, &R_inv);

    result |= arm_mat_mult_f32(&R_inv, R_cal, dst3x3);

    return result;
}