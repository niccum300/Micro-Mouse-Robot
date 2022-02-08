// Authors: Brayton Niccum
// Date: 2/5/2022

#include <Motor.h>

extern SensorQueue FrontSensorQ;
extern SensorQueue LeftSensorQ;
extern SensorQueue RightSensorQ;

// public 
Motor::Motor(int p_pin, int p_resolution, MOTOR_LOCATION p_motor_loc)
    :m_pin(p_pin), m_resolution(p_resolution), m_motor_loc(p_motor_loc)
{   
    setupPinOutput();
}

void Motor::Update()
{
    SENSOR_DATA_BUNDLE bundle;
    bundle.front = FrontSensorQ.Pop();
    bundle.left = LeftSensorQ.Pop();
    bundle.right = RightSensorQ.Pop(); 


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
