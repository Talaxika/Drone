#include "EnergyConsumer.h"
#include <iostream>

EnergyConsumer::EnergyConsumer(std::string name, float voltageRequired, float currentDraw)
    : name(name), voltageRequired(voltageRequired), currentDraw(currentDraw), isPowered(false), isEnabled(false) {}

std::string EnergyConsumer::getName() {
    return name;
}

void EnergyConsumer::connectPower(float voltageSupplied) {
    if (voltageSupplied >= voltageRequired * 0.95) {
        isPowered = true;
    } else {
        isPowered = false;
        std::cout << name << ": Voltage too low! Not operational.\n";
    }
}

void EnergyConsumer::enable() {
    if (isPowered) {
        isEnabled = true;
    } else {
        std::cout << name << ": No power, cannot enable!\n";
    }
}

void EnergyConsumer::disable() {
    isEnabled = false;
}

float EnergyConsumer::getPowerConsumption() const {
    return isPowered && isEnabled ? voltageRequired * (currentDraw / 1000.0) : 0; // Power in Watts
}

bool EnergyConsumer::isOperational() const {
    return isPowered && isEnabled;
}
