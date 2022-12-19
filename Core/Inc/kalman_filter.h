#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

#include "arm_math.h"

typedef struct KalmanFilter {
    uint16_t numStates;
    uint16_t numInputs;
    
    // System matrices
    arm_matrix_instance_f32 F;
    arm_matrix_instance_f32 G;
    // Estimate uncertainty matrix
    arm_matrix_instance_f32 P;
    // Process noise matrix
    arm_matrix_instance_f32 Q;

    // Current state matrix
    arm_matrix_instance_f32 xHat;
} KalmanFilter;

void init_kalman_filter(KalmanFilter *kf, uint16_t numStates, uint16_t numInputs, 
                        float32_t *F_f32, float32_t *G_f32, float32_t *initialP, 
                        float32_t *Q_f32, float32_t *initialXHat, float32_t *stateStdDevs);

#endif /* KALMAN_FILTER_H */