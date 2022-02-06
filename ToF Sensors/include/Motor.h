#ifndef MOTOR_H
#define  MOTOR_H

#include <global.h>

enum MOTOR_LOCATION {FRONT_LEFT = 0, FRONT_RIGHT = 1, BACK_LEFT = 3, BACK_RIGHT = 4};

class Motor{

public:
    Motor(int p_pin, int p_resolution, MOTOR_LOCATION p_motor_loc);

    void Update();
    void SetDutyCycle(int p_duty_cylce);

private:
    void setupPinOutput();

    int m_pin;
    int m_resolution;
    int m_duty_cycle = 0;
    MOTOR_LOCATION m_motor_loc;
};
#endif