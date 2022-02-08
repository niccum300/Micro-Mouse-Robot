#ifndef MOTOR_H
#define  MOTOR_H

#include <global.h>
#include <MotorQueue.h>

class Motor{

public:
    Motor(int p_pin, int p_resolution, MotorQueue * p_motor_q);

    void Update();
    void SetDutyCycle(int p_duty_cylce);

private:
    void setupPinOutput();

    int m_pin;
    int m_resolution;
    float m_duty_cycle = 0.0;
    MotorQueue * m_motor_q;
};
#endif