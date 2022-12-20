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
    arm_mat_init_f32(&wnT, 1, numStates, wnT_f32);

    result |= arm_mat_trans_f32(&wn, &wnT);

    // Compute process noise matrix
    result |= arm_mat_mult_f32(&wn, &wnT, &kf->Q);

    return result;
}

arm_status predict_kalman_filter(KalmanFilter *kf, float32_t *un_f32) {
    arm_status result = ARM_MATH_SUCCESS;

    float32_t fTerm_f32[kf->numStates];
    float32_t gTerm_f32[kf->numStates];
    float32_t partialPTerm_f32[kf->numStates * kf->numStates];
    float32_t fT_f32[kf->numStates * kf->numStates];
    float32_t fullPTerm_f32[kf->numStates * kf->numStates];

    arm_matrix_instance_f32 fTerm;
    arm_matrix_instance_f32 gTerm;
    arm_matrix_instance_f32 un;
    arm_matrix_instance_f32 partialPTerm;
    arm_matrix_instance_f32 fT;
    arm_matrix_instance_f32 fullPTerm;

    arm_mat_init_f32(&fTerm, kf->numStates, 1, fTerm_f32);
    arm_mat_init_f32(&gTerm, kf->numStates, 1, gTerm_f32);
    arm_mat_init_f32(&un, kf->numInputs, 1, un_f32);
    arm_mat_init_f32(&partialPTerm, kf->numStates, kf->numStates, partialPTerm_f32);
    arm_mat_init_f32(&fT, kf->numStates, kf->numStates, fT_f32);
    arm_mat_init_f32(&fullPTerm, kf->numStates, kf->numStates, fullPTerm_f32);

    // Extrapolate state
    arm_mat_mult_f32(&kf->F, &kf->xHat, &fTerm);
    arm_mat_mult_f32(&kf->G, &un, &gTerm);

    arm_mat_add_f32(&fTerm, &gTerm, &kf->xHat);

    // Extrapolate uncertainty
    arm_mat_mult_f32(&kf->F, &kf->P, &partialPTerm);
    arm_mat_trans_f32(&kf->F, &fT);
    arm_mat_mult_f32(&partialPTerm, &fT, &fullPTerm);
    arm_mat_add_f32(&fullPTerm, &kf->Q, &kf->P);

    return result;
}