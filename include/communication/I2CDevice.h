#ifndef I2C_DEVICE_H
#define I2C_DEVICE_H

#include "I2C.h"
#include <vector>
#include <iostream>
#include <cstdint>
#include <cstring> // For memcpy

class I2CDevice {
protected:
    I2C& i2c;
    uint8_t address;
    std::vector<uint8_t> registers; // Fixed register space

public:
    I2CDevice(I2C& i2c_bus, uint8_t addr, size_t max_register);

    virtual void writeRegister(uint8_t reg, const std::vector<uint8_t>& data);
    virtual std::vector<uint8_t> readRegister(uint8_t reg, uint8_t length);

    // Functions to support float values
    void writeFloatRegister(uint8_t reg, float value);
    float readFloatRegister(uint8_t reg);
};

#endif // I2C_DEVICE_H
