// Authors: Brayton Niccum
// Date: 2/5/2022

#include <Motor.h>

// public 
Motor::Motor(int p_pin, int p_resolution, MotorQueue * p_motor_q)
    :m_pin(p_pin), m_resolution(p_resolution), m_motor_q(p_motor_q)
{   
    //setupPinOutput();
}

void Motor::Update()
{
    if (!m_motor_q->isEmpty())
    {
        SetDutyCycle(m_motor_q->Pop());
        Serial.printf("Motor %d: %f \n", m_pin, m_duty_cycle);
        analogWrite(m_pin, m_duty_cycle);
    }
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
