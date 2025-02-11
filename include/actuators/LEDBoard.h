#ifndef LED_BOARD_H
#define LED_BOARD_H

#include "I2CDevice.h"
#include "EnergyConsumer.h"

class LEDBoard : public I2CDevice, public EnergyConsumer {
public:
    static constexpr uint8_t MAX_REGISTER = 0x05; // 5 bytes total
    static constexpr uint8_t RED_REG = 0x00;      // 1 byte (0-255)
    static constexpr uint8_t GREEN_REG = 0x01;    // 1 byte (0-255)
    static constexpr uint8_t BLUE_REG = 0x02;     // 1 byte (0-255)
    static constexpr uint8_t BRIGHTNESS_REG = 0x03; // 1 byte (0-255)
    static constexpr uint8_t COMMAND_REG = 0x04;  // 1 byte (e.g., ON/OFF command)

    LEDBoard(I2C& i2c_bus, uint8_t addr);

    void setColor(uint8_t red, uint8_t green, uint8_t blue);
    void setBrightness(uint8_t brightness);
    void turnOn();
    void turnOff();
    bool isOn();

    void writeRegister(uint8_t reg, const std::vector<uint8_t>& data) override;

private:
    bool led_on = false;
};

#endif // LED_BOARD_H
