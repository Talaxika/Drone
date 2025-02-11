#include "KalmanFilter.h"
#include "FlightKinematics.h"

FlightKinematics::FlightKinematics(float dt, float maxVelocity) :
    dt(dt), kf(dt), maxVelocity(maxVelocity),
    ax(0), ay(0), az(9.81),
    gx(0), gy(0), gz(0),
    velX(0), velY(0), velZ(0),
    posX(0), posY(0), posZ(0),
    roll(0), pitch(0), yaw(0) {}

void FlightKinematics::calculateValues(float ax, float ay, float az, float gx, float gy, float gz) {
    // Convert gyroscope readings from degrees/s to radians/s
    float gyroX = gx * (M_PI / 180.0);
    float gyroY = gy * (M_PI / 180.0);
    float gyroZ = gz * (M_PI / 180.0);

    // Step 1: Update Orientation (Euler Integration)
    roll  += gyroX * dt;
    pitch += gyroY * dt;
    yaw   += gyroZ * dt;

    // Convert angles to radians
    float rollRad = roll * (M_PI / 180.0);
    float pitchRad = pitch * (M_PI / 180.0);
    float yawRad = yaw * (M_PI / 180.0);

    // Step 2: Rotate Acceleration Based on Orientation
    float accelX_corrected = ax * cos(yawRad) - ay * sin(yawRad);
    float accelY_corrected = ax * sin(yawRad) + ay * cos(yawRad);
    float accelZ_corrected = az - 9.81; // Gravity compensation

    // Step 3: Apply Acceleration to Velocity
    static float prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0;
    velX += 0.5 * (prevAccelX + accelX_corrected) * dt;
    velY += 0.5 * (prevAccelY + accelY_corrected) * dt;
    velZ += 0.5 * (prevAccelZ + accelZ_corrected) * dt;

    prevAccelX = accelX_corrected;
    prevAccelY = accelY_corrected;
    prevAccelZ = accelZ_corrected;

    // Step 4: Apply Damping (Friction/Air Resistance)
    const float dampingFactor = 0.99;
    velX *= dampingFactor;
    velY *= dampingFactor;
    velZ *= dampingFactor;

    // Step 5: Clamp Maximum Velocity
    if (velX > maxVelocity) velX = maxVelocity;
    if (velX < -maxVelocity) velX = -maxVelocity;
    if (velY > maxVelocity) velY = maxVelocity;
    if (velY < -maxVelocity) velY = -maxVelocity;
    if (velZ > maxVelocity) velZ = maxVelocity;
    if (velZ < -maxVelocity) velZ = -maxVelocity;

    // Step 6: Update Position (Movement Adjusted for Orientation)
    posX += velX * dt;
    posY += velY * dt;
    posZ += velZ * dt;
}



void FlightKinematics::updateFlightKinematics(float ax, float ay, float az, float gx, float gy, float gz) {
    // Step 1: Calculate velocity, position, and orientation with gravity compensation
    calculateValues(ax, ay, az, gx, gy, gz);

    // Step 2: Apply Kalman Filter
    std::vector<float> filtered_state = kf.filter({posX, posY, posZ, velX, velY, velZ, roll, pitch, yaw});

    // Step 3: Update state with filtered values
    posX = filtered_state[0];
    posY = filtered_state[1];
    posZ = filtered_state[2];
    velX = filtered_state[3];
    velY = filtered_state[4];
    velZ = filtered_state[5];
    roll = filtered_state[6];
    pitch = filtered_state[7];
    yaw = filtered_state[8];
}

std::vector<float> FlightKinematics::getState() {
    return {posX, posY, posZ, velX, velY, velZ, roll, pitch, yaw};
}

std::vector<float> FlightKinematics::getPositions() {
    return {posX, posY, posZ};
}

std::vector<float> FlightKinematics::getVelocities() {
    return {velX, velY, velZ};
}
