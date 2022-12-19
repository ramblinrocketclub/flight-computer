#include "kalman_filter.h"

arm_status init_kalman_filter(KalmanFilter *kf, uint16_t numStates, uint16_t numInputs, 
                        float32_t *F_f32, float32_t *G_f32, float32_t *initialP, 
                        float32_t *Q_f32, float32_t *initialXHat, float32_t *stateStdDevs) {

    arm_status result = ARM_MATH_SUCCESS;

    float32_t wnT_f32[numStates];

    kf->numStates = numStates;
    kf->numInputs = numInputs;

    arm_mat_init_f32(&kf->F, numStates, numStates, F_f32);
    arm_mat_init_f32(&kf->G, numStates, numInputs, G_f32);
    arm_mat_init_f32(&kf->P, numStates, numStates, initialP);
    arm_mat_init_f32(&kf->Q, numStates, numStates, Q_f32);
    arm_mat_init_f32(&kf->xHat, numStates, 1, initialXHat);

    arm_matrix_instance_f32 wn;
    arm_matrix_instance_f32 wnT;

    arm_mat_init_f32(&wn, numStates, 1, stateStdDevs);
    arm_mat_init_f32(&wnT, 1, numStates, (float32_t *)wnT_f32);

    result |= arm_mat_trans_f32(&wn, &wnT);

    // Compute process noise matrix
    result |= arm_mat_mult_f32(&wn, &wnT, &kf->Q);

    return result;
}