#ifndef ENERGY_CONSUMER_H
#define ENERGY_CONSUMER_H

#include <string>

class EnergyConsumer {
protected:
    std::string name;
    float voltageRequired;
    float currentDraw; // mA
    bool isPowered;
    bool isEnabled;

public:
    EnergyConsumer(std::string name, float voltageRequired, float currentDraw);

    virtual std::string getName();
    virtual void enable();
    virtual void disable();
    virtual void connectPower(float voltageSupplied);
    virtual float getPowerConsumption() const;
    virtual bool isOperational() const;

    virtual ~EnergyConsumer() = default;
};

#endif // ENERGY_CONSUMER_H
