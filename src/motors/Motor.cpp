#include "Motor.h"
#include <iostream>

// Constructor initializes max RPM and sets default values
Motor::Motor() : max_rpm(MAX_RPM), rpm(0), throttle(0), is_faulty(false){}

// Set throttle (between 0 and 1)
void Motor::setThrottle(double value) {
    if (value < 0) value = 0;
    if (value > 1) value = 1;
    throttle = value;
}

// Update motor speed based on throttle
void Motor::update() {
    if (is_faulty) {
        rpm = 0;  // If faulty, motor does not run
    } else {
        rpm = throttle * max_rpm;
    }
}

// Get current RPM
double Motor::getRPM() const {
    return rpm;
}

// Simulate a fault (e.g., motor failure)
void Motor::induceFault() {
    std::cout << "Motor failure detected!" << std::endl;
    is_faulty = true;
}

// Reset motor to working condition
void Motor::resetFault() {
    is_faulty = false;
    std::cout << "Motor repaired!" << std::endl;
}
