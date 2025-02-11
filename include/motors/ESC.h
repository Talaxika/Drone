#ifndef ESC_H
#define ESC_H

#include "Motor.h"
#include "EnergyConsumer.h"

class ESC : public EnergyConsumer {
private:
    Motor motor;   // Controlled motor
    float throttle; // Current throttle value (0.0 - 1.0)

public:
    ESC(float voltage, float idleCurrent, float maxCurrent);

    void setThrottle(double value);
    void updatePowerConsumption();
    double getRPM();
};

#endif // ESC_H
