#ifndef MOTOR_H
#define  MOTOR_H

#include <global.h>
#include <Arduino.h>

class Motor{

public:
    Motor(int p_pin, int p_resolution);

    void Update();
    void SetDutyCycle(int p_duty_cylce);

private:
    void setupPinOutput();

    int m_pin;
    int m_resolution;
    int m_duty_cycle = 0;
};
#endif