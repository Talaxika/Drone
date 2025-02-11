#ifndef BATTERY_H
#define BATTERY_H

class Battery {
private:
    float capacity_mAh;
    float voltage;
    float remaining_mAh;

public:
    Battery(float capacity_mAh, float voltage);

    void drain(float power_mW, float duration_s);
    float getRemainingCharge() const;
    bool isDepleted() const;
    bool isLow() ;
    void displayStatus() const;
};

#endif // BATTERY_H
