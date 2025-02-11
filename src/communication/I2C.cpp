#include "I2C.h"
#include "I2CDevice.h"  // Include the full I2CDevice definition

void I2C::registerDevice(uint8_t address, I2CDevice* device) {
    devices[address] = device;
}

void I2C::write(uint8_t device_address, uint8_t reg, const std::vector<uint8_t>& data) {
    if (devices.find(device_address) != devices.end()) {
        devices[device_address]->writeRegister(reg, data);
    }
}

std::vector<uint8_t> I2C::read(uint8_t device_address, uint8_t reg, uint8_t length) {
    if (devices.find(device_address) != devices.end()) {
        return devices[device_address]->readRegister(reg, length);
    }
    return std::vector<uint8_t>(length, 0);
}
