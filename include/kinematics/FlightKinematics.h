#ifndef FLIGHT_KINEMATICS_H
#define FLIGHT_KINEMATICS_H

#include "KalmanFilter.h"
#include <vector>
#include <cmath>
#include <iostream>

class FlightKinematics {
private:
    float dt;  // Time step (s)
    KalmanFilter kf;
    float maxVelocity;

    // Accelerometer recordings
    float ax, ay, az;

    // Gyroscope recordings
    float gx, gy, gz;

    // State variables: velocity, position, and orientation
    float velX, velY, velZ;
    float posX, posY, posZ;
    float roll, pitch, yaw;  // Orientation

    void calculateValues(float ax, float ay, float az, float gx, float gy, float gz);

public:
    explicit FlightKinematics(float dt, float maxVelocity);

    void updateFlightKinematics(float ax, float ay, float az, float gx, float gy, float gz);

    std::vector<float> getState();
    std::vector<float> getPositions();
    std::vector<float> getVelocities();
};

#endif // FLIGHT_KINEMATICS_H
