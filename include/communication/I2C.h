#ifndef I2C_H
#define I2C_H

#include <iostream>
#include <unordered_map>
#include <vector>

class I2CDevice;

class I2C {
private:
    std::unordered_map<uint8_t, I2CDevice*> devices;

public:
    void registerDevice(uint8_t address, I2CDevice* device);

    void write(uint8_t device_address, uint8_t reg, const std::vector<uint8_t>& data);

    std::vector<uint8_t> read(uint8_t device_address, uint8_t reg, uint8_t length);
};

#endif /* I2C_H */
