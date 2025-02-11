#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include <iostream>
#include <functional>  // For callback function
#include <cstdint>

class DigitalPin {
private:
    int32_t pinNumber;
    int32_t pwmValue;  // 0-255 (like real PWM)
    std::function<void(int32_t)> pwmCallback;  // Callback function to notify ESC

public:
    DigitalPin(int32_t pin);

    void writePWM(int32_t value);  // Set PWM (0-255) and notify ESC
    int32_t readPWM() const;       // Get PWM value
    int32_t getPinNumber() const;  // Get pin number

    void setPWMCallback(std::function<void(int32_t)> callback);  // Set callback for ESC
};

#endif // DIGITALPIN_H