#include <MotorController.h>

// public
MotorController::MotorController()
{

}

void MotorController::Init()
{
    // nicks code for driver controlls
    pinMode(BACK_RIGHT_AIN1_PIN, OUTPUT);
    pinMode(BACK_RIGHT_AIN2_PIN, OUTPUT);
    pinMode(BACK_LEFT_BIN1_PIN, OUTPUT);
    pinMode(BACK_LEFT_BIN2_PIN, OUTPUT);

    digitalWrite(BACK_RIGHT_AIN1_PIN, LOW);
    digitalWrite(BACK_RIGHT_AIN2_PIN, HIGH);
    digitalWrite(BACK_LEFT_BIN1_PIN, LOW);
    digitalWrite(BACK_LEFT_BIN2_PIN, HIGH);

    m_driving_state = DRIVING;
}

void MotorController::Update()
{
    aquireSensorData();
    computeSensorData();
    updateMotorQueues();
}

// private
void MotorController::aquireSensorData()
{
    m_sensor_data[FRONT] = FrontSensorQ.Pop();
    m_sensor_data[LEFT]  = LeftSensorQ.Pop();
    m_sensor_data[RIGHT] = RightSensorQ.Pop();


    Serial.printf("Front: %f | Left: %f | Right: %f \n", m_sensor_data[FRONT].average, m_sensor_data[LEFT].average, m_sensor_data[RIGHT].average);
}

void MotorController::computeSensorData()
{
    
    if (m_sensor_data[FRONT].average <= MIN_DISTANCE || m_sensor_data[LEFT].average <= MIN_DISTANCE 
    || m_sensor_data[RIGHT].average <= MIN_DISTANCE)
    {
        disableMotors();
        m_driving_state = STOP;
    }
    else if(m_driving_state == DRIVING){
        m_motor_data[FRONT_LEFT] = PWM_RESOULTION_32_BIT;
        m_motor_data[FRONT_RIGHT] = PWM_RESOULTION_32_BIT;
        m_motor_data[BACK_LEFT] = PWM_RESOULTION_32_BIT;
        m_motor_data[BACK_RIGHT] = PWM_RESOULTION_32_BIT;
    }   
}

void MotorController::updateMotorQueues()
{
    FrontLeftMotorQ.Push(m_motor_data[FRONT_LEFT]);
    FrontRightMotorQ.Push(m_motor_data[FRONT_RIGHT]);
    BackLeftMotorQ.Push(m_motor_data[BACK_LEFT]);
    BackRightMotorQ.Push(m_motor_data[BACK_RIGHT]);
}

void MotorController::disableMotors()
{
    m_motor_data[FRONT_LEFT] = MOTOR_OFF * PWM_RESOULTION_32_BIT;
    m_motor_data[FRONT_RIGHT] = MOTOR_OFF * PWM_RESOULTION_32_BIT;
    m_motor_data[BACK_LEFT] = MOTOR_OFF * PWM_RESOULTION_32_BIT;
    m_motor_data[BACK_RIGHT] = MOTOR_OFF * PWM_RESOULTION_32_BIT;
}