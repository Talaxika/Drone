#include "PowerDistributionBoard.h"
#include "Util.h"
#include <iostream>

// Constructor initializes internal battery instead of receiving an external one
PowerDistributionBoard::PowerDistributionBoard(float batteryCapacity, float batteryVoltage, float motorVoltage, float sensorVoltage)
    : battery(batteryCapacity, batteryVoltage), motorVoltage(motorVoltage), sensorVoltage(sensorVoltage) {}

void PowerDistributionBoard::addMotor(EnergyConsumer* consumer) {
    motorConsumers.push_back(consumer);
    std::cout << "Added energy consumer: " + consumer->getName() << std::endl;
}

void PowerDistributionBoard::addSensor(EnergyConsumer* consumer) {
    sensorConsumers.push_back(consumer);
    std::cout << "Added energy consumer: " + consumer->getName() << std::endl;
}

void PowerDistributionBoard::distributePower() {
    // Power Motors at 11.1V
    for (auto& consumer : motorConsumers) {
        consumer->connectPower(motorVoltage);
    }

    // Power Sensors at 3.3V
    for (auto& consumer : sensorConsumers) {
        consumer->connectPower(sensorVoltage);
    }
}

void PowerDistributionBoard::updateBattery(float duration_s) {
    float totalPower = 0;

    // Motors Power Consumption
    for (const auto& consumer : motorConsumers) {
        if (ENABLE_PRINTS) {
            std::cout << consumer->getName() + " consumed " << consumer->getPowerConsumption() * 1000 << "mW." << std::endl;
        }
        totalPower += consumer->getPowerConsumption() * 1000; // Convert W to mW
    }

    // Sensors Power Consumption
    for (const auto& consumer : sensorConsumers) {
        if (ENABLE_PRINTS) {
            std::cout << consumer->getName() + " consumed " << consumer->getPowerConsumption() * 1000 << "mW." << std::endl;
        }
        totalPower += consumer->getPowerConsumption() * 1000; // Convert W to mW
    }

    battery.drain(totalPower, duration_s);
}

bool PowerDistributionBoard::isDepleted() const {
    return battery.isDepleted();
}

void PowerDistributionBoard::displayStatus() const {
    battery.displayStatus();
}

bool PowerDistributionBoard::isBatteryLow() {
    return battery.isLow();
}