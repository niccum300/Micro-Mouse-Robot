// Author Brayton Niccum
// Date: 3/21/2022

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <global.h>

enum MotorDirection {FORWARD, REVERSE};

class MotorDriver 
{

public:
    MotorDriver();

    void SetMotorDirection(MotorDirection p_direction);

};

#endif