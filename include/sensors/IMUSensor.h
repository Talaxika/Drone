#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "I2CDevice.h"
#include "EnergyConsumer.h"

class IMUSensor : public I2CDevice, public EnergyConsumer {
private:
    bool measurement_started = false;

public:
    static constexpr uint8_t MAX_REGISTER = 0x19;  // 25 bytes total
    static constexpr uint8_t ACCEL_X_REG = 0x00;   // 4 bytes (float)
    static constexpr uint8_t ACCEL_Y_REG = 0x04;   // 4 bytes (float)
    static constexpr uint8_t ACCEL_Z_REG = 0x08;   // 4 bytes (float)
    static constexpr uint8_t GYRO_X_REG  = 0x0C;   // 4 bytes (float)
    static constexpr uint8_t GYRO_Y_REG  = 0x10;   // 4 bytes (float)
    static constexpr uint8_t GYRO_Z_REG  = 0x14;   // 4 bytes (float)
    static constexpr uint8_t COMMAND_REG = 0x18;   // 1 byte (uint8_t)

    IMUSensor(I2C& i2c_bus, uint8_t addr);

    void writeRegister(uint8_t reg, const std::vector<uint8_t>& data) override;
    std::vector<uint8_t> readRegister(uint8_t reg, uint8_t length) override;

    float getAccelX();
    float getAccelY();
    float getAccelZ();
    float getGyroX();
    float getGyroY();
    float getGyroZ();

    void writeAccelX(float ax);
    void writeAccelY(float ay);
    void writeAccelZ(float az);
    void writeGyroX(float gx);
    void writeGyroY(float gy);
    void writeGyroZ(float gz);

    void reset();
};

#endif // IMU_SENSOR_H
