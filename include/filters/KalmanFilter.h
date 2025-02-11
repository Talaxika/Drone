#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <vector>

class KalmanFilter {
private:
    float dt;  // Time step (s)

    // State variables: [posX, posY, posZ, velX, velY, velZ, roll, pitch, yaw]
    std::vector<float> state = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Covariance matrix (uncertainty)
    float P[9][9] = {0};

    // Process noise
    float process_noise = 0.001;

    // Measurement noise
    float measurement_noise = 0.1;

    // Kalman Gain
    float K[9][9] = {0};

public:
    explicit KalmanFilter(float dt);

    std::vector<float> filter(std::vector<float> raw_state);
};

#endif // KALMAN_FILTER_H
