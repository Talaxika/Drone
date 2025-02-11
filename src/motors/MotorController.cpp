#include "MotorController.h"
#include <iostream>

// Default constructor
MotorController::MotorController() {}

// Init function now creates ESCs internally
void MotorController::init() {
    escs.reserve(MOTOR_COUNT);
    for (int32_t i = 0; i < MOTOR_COUNT; i++) {
        escs.emplace_back(std::make_unique<ESC>(3.3, 5, 5));
    }
}

// Hover: Set all ESCs to 50% throttle
void MotorController::hover() {
    for (auto &esc : escs) esc->setThrottle(0.5);
    state = FlightState::HOVERING;
}

// Move forward: Reduce front motors, increase back motors
void MotorController::moveForward() {
    escs[0]->setThrottle(0.4); // M1 (Front-Left)
    escs[1]->setThrottle(0.4); // M2 (Front-Right)
    escs[2]->setThrottle(0.6); // M3 (Back-Left)
    escs[3]->setThrottle(0.6); // M4 (Back-Right)
    state = FlightState::MOVING_FWD;
}

// Move backward: Increase front motors, reduce back motors
void MotorController::moveBackward() {
    escs[0]->setThrottle(0.6);
    escs[1]->setThrottle(0.6);
    escs[2]->setThrottle(0.4);
    escs[3]->setThrottle(0.4);
    state = FlightState::MOVING_BWD;
}

// Move left: Increase right motors, reduce left motors
void MotorController::moveLeft() {
    escs[0]->setThrottle(0.4);
    escs[1]->setThrottle(0.6);
    escs[2]->setThrottle(0.4);
    escs[3]->setThrottle(0.6);
    state = FlightState::MOVING_LEFT;
}

// Move right: Increase left motors, reduce right motors
void MotorController::moveRight() {
    escs[0]->setThrottle(0.6);
    escs[1]->setThrottle(0.4);
    escs[2]->setThrottle(0.6);
    escs[3]->setThrottle(0.4);
    state = FlightState::MOVING_RIGHT;
}

// Ascend: Increase all motors
void MotorController::moveUp() {
    for (auto &esc : escs) esc->setThrottle(0.7);
    state = FlightState::ASCENDING;
}

// Descend: Reduce all motors
void MotorController::moveDown() {
    for (auto &esc : escs) esc->setThrottle(0.3);
    state = FlightState::DESCENDING;
}

// Stop all motors
void MotorController::turnOff() {
    for (auto &esc : escs) esc->setThrottle(0.0);
    state = FlightState::IDLE;
}

// Display motor RPMs
void MotorController::displayStatus() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        std::cout << "Motor " << i + 1 << " RPM: " << escs[i]->getRPM() << std::endl;
    }
    displayState();
}

void MotorController::displayState() {
    std::cout << "Current Drone Status: ";
    switch (state) {
        case FlightState::IDLE: std::cout << "Idle"; break;
        case FlightState::HOVERING: std::cout << "Hovering"; break;
        case FlightState::MOVING_FWD: std::cout << "Moving Forward"; break;
        case FlightState::MOVING_BWD: std::cout << "Moving Backward"; break;
        case FlightState::MOVING_LEFT: std::cout << "Moving Left"; break;
        case FlightState::MOVING_RIGHT: std::cout << "Moving Right"; break;
        case FlightState::ASCENDING: std::cout << "Ascending"; break;
        case FlightState::DESCENDING: std::cout << "Descending"; break;
    }
    std::cout << std::endl;
}

FlightState MotorController::getState() {
    return state;
}

// Allow PCB to access ESCs
std::vector<ESC*> MotorController::getESCs() {
    std::vector<ESC*> escPtrs;
    for (auto &esc : escs) escPtrs.push_back(esc.get());
    return escPtrs;
}
