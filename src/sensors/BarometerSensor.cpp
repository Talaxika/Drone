#include "BarometerSensor.h"

BarometerSensor::BarometerSensor(uint8_t addr, uint16_t pressure)
    : I2CDevice(addr) {
    registers[PRESSURE_REG] = (pressure >> 8) & 0xFF;  // MSB
    registers[PRESSURE_REG + 1] = pressure & 0xFF;      // LSB
    registers[COMMAND_REG] = 0x00;
}

void BarometerSensor::writeRegister(uint8_t reg, const std::vector<uint8_t>& data) {
    if (reg == COMMAND_REG && !data.empty() && data[0] == 0x01) {
        measurement_started = true;
        std::cout << "Barometer (" << static_cast<int>(address) << "): Measurement started...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(300));  // Simulate processing delay
    } else if (measurement_started) {
        for (size_t i = 0; i < data.size(); i++) {
            registers[reg + i] = data[i];
        }
    } else {
        std::cout << "Barometer (" << static_cast<int>(address) << "): Write failed, start measurement first!\n";
    }
}

std::vector<uint8_t> BarometerSensor::readRegister(uint8_t reg, uint8_t length) {
    if (!measurement_started) {
        std::cout << "Barometer (" << static_cast<int>(address) << "): Read failed, start measurement first!\n";
        return std::vector<uint8_t>(length, 0xFF);
    }

    std::vector<uint8_t> result;
    for (uint8_t i = 0; i < length; i++) {
        result.push_back(registers[reg + i]);
    }
    return result;
}
