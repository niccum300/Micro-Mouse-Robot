// Author Brayton Niccum
// Date: 3/21/2022

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <global.h>

#define BACK_RIGHT_AIN2_PIN (33)
#define BACK_RIGHT_AIN1_PIN (34)
#define BACK_LEFT_BIN2_PIN (39)
#define BACK_LEFT_BIN1_PIN (38)
#define STANDBY_PIN (36)

enum MotorDirection {FORWARDS, REVERSE, ZERO_POINT, RIGHT_ZERO_POINT, LEFT_ZERO_POINT};

class MotorDriver 
{

public:
    MotorDriver();

    void SetMotorDirection(MotorDirection p_direction);

    void ToggleZeroDirection();

    MotorDirection m_current_direction = FORWARDS;

};

#endif