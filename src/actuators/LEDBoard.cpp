#include "LEDBoard.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring> // For std::memcpy

LEDBoard::LEDBoard(I2C& i2c_bus, uint8_t addr)
    : I2CDevice(i2c_bus, addr, MAX_REGISTER), EnergyConsumer("LEDBoard", 3.3, 0.02) {} // Assume 20mA LED draw

void LEDBoard::setColor(uint8_t red, uint8_t green, uint8_t blue) {
    if (!isOperational()) {
        std::cout << "LEDBoard (" << static_cast<int>(address) << "): Not operational...\n";
        return;
    }

    writeRegister(RED_REG, {red});
    writeRegister(GREEN_REG, {green});
    writeRegister(BLUE_REG, {blue});

    std::cout << "LEDBoard (" << static_cast<int>(address) << "): Color set to ("
              << static_cast<int>(red) << ", " << static_cast<int>(green) << ", "
              << static_cast<int>(blue) << ")\n";
}

void LEDBoard::setBrightness(uint8_t brightness) {
    if (!isOperational()) {
        std::cout << "LEDBoard (" << static_cast<int>(address) << "): Not operational...\n";
        return;
    }
    
    writeRegister(BRIGHTNESS_REG, {brightness});
    std::cout << "LEDBoard (" << static_cast<int>(address) << "): Brightness set to " 
              << static_cast<int>(brightness) << "/255\n";
}

void LEDBoard::turnOn() {
    if (!isOperational()) {
        std::cout << "LEDBoard (" << static_cast<int>(address) << "): Not operational...\n";
        return;
    }

    writeRegister(COMMAND_REG, {0x01});  // 0x01 = ON command
    led_on = true;
    std::cout << "LEDBoard (" << static_cast<int>(address) << "): LED turned ON.\n";
}

void LEDBoard::turnOff() {
    if (!isOperational()) {
        std::cout << "LEDBoard (" << static_cast<int>(address) << "): Not operational...\n";
        return;
    }

    writeRegister(COMMAND_REG, {0x00});  // 0x00 = OFF command
    led_on = false;
    std::cout << "LEDBoard (" << static_cast<int>(address) << "): LED turned OFF.\n";
}

bool LEDBoard::isOn() {
    return led_on;
}

void LEDBoard::writeRegister(uint8_t reg, const std::vector<uint8_t>& data) {
    if (!isOperational()) {
        std::cout << "LEDBoard (" << static_cast<int>(address) << "): Not operational...\n";
        return;
    }

    if (reg == COMMAND_REG && !data.empty()) {
        uint8_t command = data[0];

        if (command == 0x01) {  // Turn on
            led_on = true;
            std::cout << "LEDBoard (" << static_cast<int>(address) << "): LED turned ON.\n";
        } else if (command == 0x00) {  // Turn off
            led_on = false;
            std::cout << "LEDBoard (" << static_cast<int>(address) << "): LED turned OFF.\n";
        }
        return;
    }

    I2CDevice::writeRegister(reg, data);
}
