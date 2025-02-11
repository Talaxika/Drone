#include "Battery.h"
#include <iostream>

Battery::Battery(float capacity_mAh, float voltage) : capacity_mAh(capacity_mAh), voltage(voltage), remaining_mAh(capacity_mAh) {}

void Battery::drain(float power_mW, float duration_s) {
    float energyUsed_mAh = (power_mW * duration_s) / (voltage * 3600);
    remaining_mAh -= energyUsed_mAh;
    if (remaining_mAh < 0) remaining_mAh = 0;
}

float Battery::getRemainingCharge() const {
    return remaining_mAh;
}

bool Battery::isDepleted() const {
    return remaining_mAh <= 0;
}

bool Battery::isLow() {
    return remaining_mAh <= capacity_mAh * 0.2;
}


void Battery::displayStatus() const {
    std::cout << "Battery mAh remaining (" << (remaining_mAh / capacity_mAh) * 100 << "%)\n";
}
