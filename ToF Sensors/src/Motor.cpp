// Authors: Brayton Niccum
// Date: 2/5/2022

#include <Motor.h>

// public 
Motor::Motor(int p_pin, int p_resolution, MOTOR_LOCATION p_motor_loc)
    :m_pin(p_pin), m_resolution(p_resolution), m_motor_loc(p_motor_loc)
{   
    setupPinOutput();
}

void Motor::Update()
{
    SENSOR_DATA_BUNDLE bundle;
    switch (m_motor_loc)
    {
    case FRONT_LEFT:
            bundle = FrontLeftMotorQ.Pop();
        break;
    case FRONT_RIGHT:
            bundle = FrontRightMotorQ.Pop();
        break;
    case BACK_LEFT:
            bundle = BackLeftMotorQ.Pop();
        break;
    case BACK_RIGHT:
            bundle = BackRightMotorQ.Pop();
        break;
    }

    Serial.printf(" \n Motor %d | Values %f | %f | %f", m_motor_loc, bundle.front.average, bundle.left.average, bundle.right.average);


    analogWrite(m_pin, m_duty_cycle);
}

void Motor::SetDutyCycle(int p_duty_cylce)
{
    m_duty_cycle = p_duty_cylce;
}

// private
void Motor::setupPinOutput()
{
    analogWriteResolution(m_resolution);
}
