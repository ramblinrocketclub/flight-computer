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
    result |= arm_mat_mult_f32(&kf->F, &kf->xHat, &fTerm);
    result |= arm_mat_mult_f32(&kf->G, &un, &gTerm);

    result |= arm_mat_add_f32(&fTerm, &gTerm, &kf->xHat);

    // Extrapolate uncertainty
    result |= arm_mat_mult_f32(&kf->F, &kf->P, &partialPTerm);
    result |= arm_mat_trans_f32(&kf->F, &fT);
    result |= arm_mat_mult_f32(&partialPTerm, &fT, &fullPTerm);
    result |= arm_mat_add_f32(&fullPTerm, &kf->Q, &kf->P);

    return result;
}

arm_status correct_kalman_filter(KalmanFilter *kf, uint16_t numMeasuredStates, float32_t *zn_f32, 
                        float32_t *H_f32, float32_t *measurementStdDevs) {
    arm_status result = ARM_MATH_SUCCESS;
    
    float32_t HT_f32[kf->numStates * numMeasuredStates];
    float32_t vn_f32[numMeasuredStates];
    float32_t Rn_f32[numMeasuredStates * numMeasuredStates];
    float32_t K_f32[kf->numStates * numMeasuredStates];
    float32_t KT_f32[numMeasuredStates * kf->numStates];

    float32_t PHT_f32[kf->numStates * numMeasuredStates];
    float32_t HPHT_f32[numMeasuredStates * numMeasuredStates];
    float32_t HPHTRn_f32[numMeasuredStates * numMeasuredStates];

    // P: nx * nx
    arm_matrix_instance_f32 H;  // nz * nx
    arm_matrix_instance_f32 HT; // nx * nz
    arm_matrix_instance_f32 vn; // nz * 1
    arm_matrix_instance_f32 Rn; // nz * nz
    arm_matrix_instance_f32 K;  // nx * nz
    arm_matrix_instance_f32 KT; // nz * nx

    arm_matrix_instance_f32 PHT; // nx * nz
    arm_matrix_instance_f32 HPHT; // nz * nz
    arm_matrix_instance_f32 HPHTRn; // nz * nz
    arm_matrix_instance_f32 HPHTRnI; // nz * nz

    arm_mat_init_f32(&H, numMeasuredStates, kf->numStates, H_f32);
    arm_mat_init_f32(&HT, kf->numStates, numMeasuredStates, HT_f32);
    arm_mat_init_f32(&vn, numMeasuredStates, 1, vn_f32);
    arm_mat_init_f32(&Rn, numMeasuredStates, numMeasuredStates, Rn_f32);
    arm_mat_init_f32(&K, kf->numStates, numMeasuredStates, K_f32);
    arm_mat_init_f32(&KT, numMeasuredStates, kf->numStates, KT_f32);
    
    arm_mat_init_f32(&PHT, kf->numStates, numMeasuredStates, PHT_f32);
    arm_mat_init_f32(&HPHT, numMeasuredStates, numMeasuredStates, HPHT_f32);
    arm_mat_init_f32(&HPHTRn, numMeasuredStates, numMeasuredStates, HPHTRn_f32);

    arm_mat_trans_f32(&H, &HT);

    arm_mat_mult_f32(&kf->P, &HT, &PHT);
    arm_mat_mult_f32(&H, &PHT, &HPHT);

    arm_mat_add_f32(&HPHT, &Rn, &HPHTRn);

    arm_mat_inverse_f32(&HPHTRn, &HPHTRnI);

    arm_mat_mult_f32(&PHT, &HPHTRnI, &K);

    return result;
}