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

    digitalWrite(BACK_RIGHT_AIN1_PIN, HIGH);
    digitalWrite(BACK_RIGHT_AIN2_PIN, LOW);
    digitalWrite(BACK_LEFT_BIN1_PIN, HIGH);
    digitalWrite(BACK_LEFT_BIN2_PIN, LOW);

    m_driving_state = DRIVING;
}

void MotorController::Update()
{
    aquireSensorData();
    //computeSensorData();
    ZigZag();
    
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
        m_motor_data[FRONT_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[FRONT_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
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

void MotorController::ZigZag()
{
    if (m_sensor_data[FRONT].average <= MIN_DISTANCE_FRONT)
    {
        disableMotors();
        m_driving_state = STOP;
        return;
    }

    if (m_sensor_data[LEFT].average < m_sensor_data[RIGHT].average)
    {
        m_driving_state = SLOWRIGHT;
    }

    if (m_driving_state == SLOWRIGHT){
        m_motor_data[FRONT_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[FRONT_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT) *.95;
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT)*.95;
    }
    if (m_sensor_data[RIGHT].average < m_sensor_data[LEFT].average)
    {
        m_driving_state = SLOWLEFT;
    }

    if (m_driving_state == SLOWLEFT){
        m_motor_data[FRONT_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT) *.95;
        m_motor_data[FRONT_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT) *.95;
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
    }
}
