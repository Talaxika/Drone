#ifndef BAROMETER_SENSOR_H
#define BAROMETER_SENSOR_H

#include "I2CDevice.h"
#include <unordered_map>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

class BarometerSensor : public I2CDevice {
private:
    std::unordered_map<uint8_t, uint8_t> registers;
    bool measurement_started = false;

public:
    static constexpr uint8_t PRESSURE_REG = 0x00;
    static constexpr uint8_t COMMAND_REG = 0x02;

    BarometerSensor(uint8_t addr, uint16_t pressure = 101);

    void writeRegister(uint8_t reg, const std::vector<uint8_t>& data) override;
    std::vector<uint8_t> readRegister(uint8_t reg, uint8_t length) override;
};

#endif // BAROMETER_SENSOR_H
