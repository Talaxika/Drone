#ifndef POWER_DISTRIBUTION_BOARD_H
#define POWER_DISTRIBUTION_BOARD_H

#include "Battery.h"
#include "EnergyConsumer.h"
#include <vector>

class PowerDistributionBoard {
private:
    Battery battery;  // Internal battery
    float motorVoltage; // 11.1V for motors
    float sensorVoltage; // 3.3V for sensors
    std::vector<EnergyConsumer*> motorConsumers;
    std::vector<EnergyConsumer*> sensorConsumers;

public:
    PowerDistributionBoard(float batteryCapacity, float batteryVoltage, float motorVoltage, float sensorVoltage);

    void addMotor(EnergyConsumer* consumer);
    void addSensor(EnergyConsumer* sensor);
    void distributePower();
    void updateBattery(float duration_s);
    bool isDepleted() const;
    void displayStatus() const;
    bool isBatteryLow();
};

#endif // POWER_DISTRIBUTION_BOARD_H
