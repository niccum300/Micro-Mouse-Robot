// Authors: Brayton Niccum
// Date: 2/5/2022

#include <Motor.h>

// public 
Motor::Motor(int p_pin, int p_resolution)
    :m_pin(p_pin), m_resolution(p_resolution)
{   
    setupPinOutput();
}

void Motor::Update()
{
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
