#include "ESC.h"
#include <iostream>

ESC::ESC(float voltage, float idleCurrent, float maxCurrent)
    : EnergyConsumer("ESC", voltage, idleCurrent), throttle(0.0) {}

void ESC::setThrottle(double value) {
    if (value < 0.0) value = 0.0;
    if (value > 1.0) value = 1.0;
    throttle = value;
    motor.setThrottle(value);
    motor.update();
    updatePowerConsumption();
}

void ESC::updatePowerConsumption() {
    // Power consumption scales with throttle
    currentDraw = 5.0 + (motor.getRPM() / MAX_RPM) * 20.0; // Example: 5mA (idle) to 25mA (max)
}

double ESC::getRPM() {
    return motor.getRPM();
}
