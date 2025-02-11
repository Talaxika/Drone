#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "ESC.h"
#include "DigitalPin.h"
#include <vector>
#include <memory>

constexpr int32_t MOTOR_COUNT = 4;
constexpr float MAX_VELOCITY = 10.0f; // Set the device speed limit

enum class FlightState {
    IDLE, HOVERING, MOVING_FWD, MOVING_BWD, MOVING_LEFT, MOVING_RIGHT, ASCENDING, DESCENDING
};

class MotorController {
private:
    FlightState state = FlightState::IDLE;
    std::vector<std::unique_ptr<ESC>> escs; // Store ESCs inside

public:
    MotorController();
    void init();

    void hover();
    void moveForward();
    void moveBackward();
    void moveLeft();
    void moveRight();
    void moveUp();
    void moveDown();

    void turnOff();

    void displayStatus();
    void displayState();

    FlightState getState();
    // Function to allow PCB to access ESCs for power tracking
    std::vector<ESC*> getESCs();
};

#endif // MOTORCONTROLLER_H
