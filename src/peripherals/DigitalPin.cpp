#include "DigitalPin.h"

DigitalPin::DigitalPin(int32_t pin) : pinNumber(pin), pwmValue(0) {}

void DigitalPin::writePWM(int32_t value) {
    if (value < 0) value = 0;
    if (value > 255) value = 255; // Limit PWM range

    pwmValue = value;
    std::cout << "Pin " << pinNumber << " set to PWM: " << pwmValue << std::endl;

    // Activate callback
    if (pwmCallback) {
        pwmCallback(value);
    }
}

int32_t DigitalPin::readPWM() const {
    return pwmValue;
}

int32_t DigitalPin::getPinNumber() const {
    return pinNumber;
}

// Set a callback function to notify ESC when PWM changes
void DigitalPin::setPWMCallback(std::function<void(int32_t)> callback) {
    pwmCallback = callback;
}
