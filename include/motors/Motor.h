#ifndef MOTOR_H
#define MOTOR_H

#include <cstdint>
#include "EnergyConsumer.h"

constexpr int32_t MAX_RPM = 6000;

class Motor {
private:
    double rpm;         // Rotations per minute
    double throttle;    // Input power (0 to 1)
    double max_rpm;     // Max possible RPM
    bool is_faulty;     // Fault state

public:
    // Constructor
    Motor();

    // Set throttle (0-1 range)
    void setThrottle(double value);

    // Update motor speed based on throttle input
    void update();

    // Get current RPM
    double getRPM() const;

    // Simulate fault (e.g., overheating or failure)
    void induceFault();

    // Reset motor fault
    void resetFault();
};

#endif // MOTOR_H
