#include "TempHumiditySensor.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring> // For std::memcpy

TempHumiditySensor::TempHumiditySensor(I2C& i2c_bus, uint8_t addr)
    : I2CDevice(i2c_bus, addr, MAX_REGISTER), EnergyConsumer("TempHumi", 3.3, 5) {}

float TempHumiditySensor::getTemperature() {

    if (!isOperational()) {
        std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Not operational...\n";
        return -999.9f;
    }

    if (!measurement_started) {
        std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Read failed, start measurement first!\n";
        return -999.9f;
    }
    return readFloatRegister(TEMP_REG);
}

void TempHumiditySensor::writeTemperature(float temperature) {

    if (!isOperational()) {
        std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Not operational...\n";
        return;
    }

    if (!measurement_started) {
        std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Cannot write temperature, start measurement first!\n";
        return;
    }

    writeFloatRegister(TEMP_REG, temperature);
    std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Temperature written: " << temperature << "°C\n";
}

float TempHumiditySensor::getHumidity() {

    if (!isOperational()) {
        std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Not operational...\n";
        return -1.0f;
    }

    if (!measurement_started) {
        std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Read failed, start measurement first!\n";
        return -1.0f;
    }
    return readFloatRegister(HUMIDITY_REG);
}

void TempHumiditySensor::writeHumidity(float humidity) {

    if (!isOperational()) {
        std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Not operational...\n";
        return;
    }

    if (!measurement_started) {
        std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Cannot write humidity, start measurement first!\n";
        return;
    }

    writeFloatRegister(HUMIDITY_REG, humidity);
    std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Humidity written: " << humidity << "%\n";
}

uint8_t TempHumiditySensor::getCommand() {
    auto data = readRegister(COMMAND_REG, 1);
    return data[0];
}

void TempHumiditySensor::writeCommand(uint8_t command) {
    writeRegister(COMMAND_REG, {command});
    std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Command written: " << static_cast<int>(command) << "\n";
}

void TempHumiditySensor::writeRegister(uint8_t reg, const std::vector<uint8_t>& data) {

    if (!isOperational()) {
        std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Not operational...\n";
        return;
    }

    if (reg == COMMAND_REG && !data.empty()) {
        uint8_t command = data[0];

        if (command == 0x01) {  // Start measurement
            measurement_started = true;
            std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Measurement started...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Simulated delay
        } else if (command == 0x02) {  // Stop measurement
            measurement_started = false;
            std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Measurement stopped.\n";
        } else if (command == 0x03) {  // Reset sensor
            measurement_started = false;
            std::fill(registers.begin(), registers.end(), 0);
            std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Sensor reset.\n";
        }
        return;
    }

    if (measurement_started) {
        I2CDevice::writeRegister(reg, data);
    } else {
        std::cout << "TempHumiditySensor (" << static_cast<int>(address) << "): Write failed, start measurement first!\n";
    }
}
