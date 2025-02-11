#include "IMUSensor.h"
#include <thread>
#include <chrono>
#include <iostream>

IMUSensor::IMUSensor(I2C& i2c_bus, uint8_t addr)
    : I2CDevice(i2c_bus, addr, MAX_REGISTER), EnergyConsumer("IMU", 3.3, 5) {}

void IMUSensor::writeRegister(uint8_t reg, const std::vector<uint8_t>& data) {

    if (!isOperational()) {
        std::cout << "IMU (" << static_cast<int>(address) << "): Not operational...\n";
        return;
    }

    if (reg == COMMAND_REG && !data.empty()) {
        uint8_t command = data[0];

        if (command == 0x01) {  // Start measurement
            measurement_started = true;
            std::cout << "IMU (" << static_cast<int>(address) << "): Measurement started...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simulated processing delay
        } else if (command == 0x02) {  // Stop measurement
            measurement_started = false;
            std::cout << "IMU (" << static_cast<int>(address) << "): Measurement stopped.\n";
        } else if (command == 0x03) {  // Reset sensor
            measurement_started = false;
            std::fill(registers.begin(), registers.end(), 0);
            std::cout << "IMU (" << static_cast<int>(address) << "): Sensor reset.\n";
        }
        return;
    }

    if (measurement_started) {
        I2CDevice::writeRegister(reg, data);
    } else {
        std::cout << "IMU (" << static_cast<int>(address) << "): Write failed, start measurement first!\n";
    }
}

std::vector<uint8_t> IMUSensor::readRegister(uint8_t reg, uint8_t length) {

    if (!isOperational()) {
        std::cout << "IMU (" << static_cast<int>(address) << "): Not operational...\n";
        return std::vector<uint8_t>(length, 0xFF);
    }

    if (!measurement_started) {
        std::cout << "IMU (" << static_cast<int>(address) << "): Read failed, start measurement first!\n";
        return std::vector<uint8_t>(length, 0xFF);
    }
    return I2CDevice::readRegister(reg, length);
}

// Read Acceleration & Gyro Data (Floating-Point)
float IMUSensor::getAccelX() {
    return readFloatRegister(ACCEL_X_REG);
}

float IMUSensor::getAccelY() {
    return readFloatRegister(ACCEL_Y_REG);
}

float IMUSensor::getAccelZ() {
    return readFloatRegister(ACCEL_Z_REG);
}

float IMUSensor::getGyroX() {
    return readFloatRegister(GYRO_X_REG);
}

float IMUSensor::getGyroY() {
    return readFloatRegister(GYRO_Y_REG);
}

float IMUSensor::getGyroZ() {
    return readFloatRegister(GYRO_Z_REG);
}

// Write Acceleration & Gyro Data (Floating-Point)
void IMUSensor::writeAccelX(float ax) {
    if (!measurement_started) {
        std::cout << "IMU (" << static_cast<int>(address) << "): Cannot write acceleration X, start measurement first!\n";
        return;
    }
    writeFloatRegister(ACCEL_X_REG, ax);
    std::cout << "IMU (" << static_cast<int>(address) << "): Acceleration X written: " << ax << "\n";
}

void IMUSensor::writeAccelY(float ay) {
    if (!measurement_started) {
        std::cout << "IMU (" << static_cast<int>(address) << "): Cannot write acceleration Y, start measurement first!\n";
        return;
    }
    writeFloatRegister(ACCEL_Y_REG, ay);
    std::cout << "IMU (" << static_cast<int>(address) << "): Acceleration Y written: " << ay << "\n";
}

void IMUSensor::writeAccelZ(float az) {
    if (!measurement_started) {
        std::cout << "IMU (" << static_cast<int>(address) << "): Cannot write acceleration Z, start measurement first!\n";
        return;
    }
    writeFloatRegister(ACCEL_Z_REG, az);
    std::cout << "IMU (" << static_cast<int>(address) << "): Acceleration Z written: " << az << "\n";
}

void IMUSensor::writeGyroX(float gx) {
    if (!measurement_started) {
        std::cout << "IMU (" << static_cast<int>(address) << "): Cannot write gyroscope X, start measurement first!\n";
        return;
    }
    writeFloatRegister(GYRO_X_REG, gx);
    std::cout << "IMU (" << static_cast<int>(address) << "): Gyroscope X written: " << gx << "\n";
}

void IMUSensor::writeGyroY(float gy) {
    if (!measurement_started) {
        std::cout << "IMU (" << static_cast<int>(address) << "): Cannot write gyroscope Y, start measurement first!\n";
        return;
    }
    writeFloatRegister(GYRO_Y_REG, gy);
    std::cout << "IMU (" << static_cast<int>(address) << "): Gyroscope Y written: " << gy << "\n";
}

void IMUSensor::writeGyroZ(float gz) {
    if (!measurement_started) {
        std::cout << "IMU (" << static_cast<int>(address) << "): Cannot write gyroscope Z, start measurement first!\n";
        return;
    }
    writeFloatRegister(GYRO_Z_REG, gz);
    std::cout << "IMU (" << static_cast<int>(address) << "): Gyroscope Z written: " << gz << "\n";
}

void IMUSensor::reset() {
    if (!measurement_started) {
        std::cout << "IMU (" << static_cast<int>(address) << "): Cannot reset, start measurement first!\n";
        return;
    }

    writeFloatRegister(ACCEL_X_REG, 0);
    writeFloatRegister(ACCEL_Y_REG, 0);
    writeFloatRegister(ACCEL_Z_REG, 9.81);
    writeFloatRegister(GYRO_X_REG, 0);
    writeFloatRegister(GYRO_Y_REG, 0);
    writeFloatRegister(GYRO_Z_REG, 0);
}
