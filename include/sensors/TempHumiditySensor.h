#ifndef TEMP_HUMIDITY_SENSOR_H
#define TEMP_HUMIDITY_SENSOR_H

#include "I2CDevice.h"
#include "EnergyConsumer.h"

class TempHumiditySensor : public I2CDevice, public EnergyConsumer {
public:
    static constexpr uint8_t MAX_REGISTER = 0x09; // 9 bytes total
    static constexpr uint8_t TEMP_REG = 0x00;     // Float (4 bytes)
    static constexpr uint8_t HUMIDITY_REG = 0x04; // Float (4 bytes)
    static constexpr uint8_t COMMAND_REG = 0x08;  // 1 byte (uint8_t)

    TempHumiditySensor(I2C& i2c_bus, uint8_t addr);

    float getTemperature();
    void writeTemperature(float temperature);

    float getHumidity();
    void writeHumidity(float humidity);

    uint8_t getCommand();
    void writeCommand(uint8_t command);

    void writeRegister(uint8_t reg, const std::vector<uint8_t>& data) override;

private:
    bool measurement_started = false;
};

#endif // TEMP_HUMIDITY_SENSOR_H
