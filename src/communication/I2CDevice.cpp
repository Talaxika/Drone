#include "I2CDevice.h"
#include "Util.h"

I2CDevice::I2CDevice(I2C& i2c_bus, uint8_t addr, size_t max_register)
    : i2c(i2c_bus), address(addr), registers(max_register, 0) {
    i2c.registerDevice(address, this);
}

void I2CDevice::writeRegister(uint8_t reg, const std::vector<uint8_t>& data) {
    if (reg + data.size() <= registers.size()) { // Ensure within bounds
        for (size_t i = 0; i < data.size(); ++i) {
            registers[reg + i] = data[i];
        }
    }
}

std::vector<uint8_t> I2CDevice::readRegister(uint8_t reg, uint8_t length) {
    std::vector<uint8_t> data(length, 0);
    if (reg + length <= registers.size()) { // Ensure within bounds
        for (uint8_t i = 0; i < length; ++i) {
            data[i] = registers[reg + i];
        }
    }
    return data;
}

// Write a float value into a 4-byte register
void I2CDevice::writeFloatRegister(uint8_t reg, float value) {
    if (reg + 4 > registers.size()) { // Ensure within bounds
        std::cout << "I2CDevice (" << static_cast<int>(address) << "): Float write out of bounds!\n";
        return;
    }

    uint8_t floatBytes[4];
    std::memcpy(floatBytes, &value, sizeof(float));

    writeRegister(reg, {floatBytes[0], floatBytes[1], floatBytes[2], floatBytes[3]});
    std::cout << "I2CDevice (" << static_cast<int>(address) << "): Wrote float " << value << " to register " << static_cast<int>(reg) << "\n";
}

// Read a float value from a 4-byte register
float I2CDevice::readFloatRegister(uint8_t reg) {
    if (reg + 4 > registers.size()) { // Ensure within bounds
        std::cout << "I2CDevice (" << static_cast<int>(address) << "): Float read out of bounds!\n";
        return 0.0f; // Return invalid value
    }

    auto data = readRegister(reg, 4);
    float value;
    std::memcpy(&value, data.data(), sizeof(float));

    if (ENABLE_PRINTS) {
        std::cout << "I2CDevice (" << static_cast<int>(address) << "): Read float " << value << " from register " << static_cast<int>(reg) << "\n";
    }

    return value;
}
