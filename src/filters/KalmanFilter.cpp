#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(float dt) : dt(dt) {
    for (int i = 0; i < 9; i++) {
        P[i][i] = 1;  // Initial uncertainty
    }
}

std::vector<float> KalmanFilter::filter(std::vector<float> raw_state) {
    // Extract raw IMU-based velocity, position, and orientation
    float rawPosX = raw_state[0], rawPosY = raw_state[1], rawPosZ = raw_state[2];
    float rawVelX = raw_state[3], rawVelY = raw_state[4], rawVelZ = raw_state[5];
    float rawRoll = raw_state[6], rawPitch = raw_state[7], rawYaw = raw_state[8];

    // Predict Step: Estimate new state
    state[0] += state[3] * dt;
    state[1] += state[4] * dt;
    state[2] += state[5] * dt;

    state[3] = rawVelX;
    state[4] = rawVelY;
    state[5] = rawVelZ;
    state[6] = rawRoll;
    state[7] = rawPitch;
    state[8] = rawYaw;

    // Update covariance matrix (P) with limits
    for (int i = 0; i < 9; i++) {
        P[i][i] = std::min(P[i][i] + process_noise, 10.0f);
    }

    // Compute Kalman Gain
    for (int i = 0; i < 9; i++) {
        K[i][i] = P[i][i] / (P[i][i] + measurement_noise);
    }

    // Correct estimates using Kalman Gain
    state[3] += K[3][3] * (rawVelX - state[3]);
    state[4] += K[4][4] * (rawVelY - state[4]);
    state[5] += K[5][5] * (rawVelZ - state[5]);
    state[6] += K[6][6] * (rawRoll - state[6]);
    state[7] += K[7][7] * (rawPitch - state[7]);
    state[8] += K[8][8] * (rawYaw - state[8]);

    // Apply smoothed position correction
    state[0] += K[0][0] * (rawPosX - state[0]);
    state[1] += K[1][1] * (rawPosY - state[1]);
    state[2] += K[2][2] * (rawPosZ - state[2]);

    // Update covariance matrix
    for (int i = 0; i < 9; i++) {
        P[i][i] *= (1 - K[i][i]);
    }

    return {state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7], state[8]};
}
