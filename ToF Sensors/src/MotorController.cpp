#include <MotorController.h>

// public
MotorController::MotorController()
{

}

void MotorController::Init()
{
    // nicks code for driver controlls
    pinMode(BACK_RIGHT_AIN1_PIN, OUTPUT); //
    pinMode(BACK_RIGHT_AIN2_PIN, OUTPUT);
    pinMode(BACK_LEFT_BIN1_PIN, OUTPUT);
    pinMode(BACK_LEFT_BIN2_PIN, OUTPUT);

    digitalWrite(BACK_RIGHT_AIN1_PIN, HIGH);
    digitalWrite(BACK_RIGHT_AIN2_PIN, LOW);
    digitalWrite(BACK_LEFT_BIN1_PIN, HIGH);
    digitalWrite(BACK_LEFT_BIN2_PIN, LOW);

    m_driving_state = STOP;
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
    m_gyro_data = GyroQ.Pop();


    Serial.printf("Front: %f | Left: %f | Right: %f \n", m_sensor_data[FRONT].average, m_sensor_data[LEFT].average, m_sensor_data[RIGHT].average);
    Serial.printf("Gyro: %f \n", m_gyro_data);
}

void MotorController::computeSensorData()
{
    
    // if (m_sensor_data[FRONT].average <= MIN_DISTANCE || m_sensor_data[LEFT].average <= MIN_DISTANCE 
    // || m_sensor_data[RIGHT].average <= MIN_DISTANCE)
    // {
    //     disableMotors();
    //     m_driving_state = STOP;
    // }
    // else if(m_driving_state == DRIVING){
    //     m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
    //     m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
    // }   
}

void MotorController::updateMotorQueues()
{
    BackLeftMotorQ.Push(m_motor_data[BACK_LEFT]);
    BackRightMotorQ.Push(m_motor_data[BACK_RIGHT]);
}

void MotorController::disableMotors()
{
    m_motor_data[BACK_LEFT] = MOTOR_OFF * PWM_RESOULTION_32_BIT;
    m_motor_data[BACK_RIGHT] = MOTOR_OFF * PWM_RESOULTION_32_BIT;
}

void MotorController::ZigZag()
{
    if (m_driving_state == STOP)
    {
        if(m_sensor_data[RIGHT].average <= MIN_DISTANCE)
        {
            m_driving_state = DRIVING;
        }

        return;
    }
    if (m_driving_state == TURNLEFT) {turnLeft(); return;}
    if (m_driving_state == TURNRIGHT) {turnRight(); return;}
    if (m_sensor_data[FRONT].average <= MIN_DISTANCE_FRONT)
    {
        disableMotors();
        //m_driving_state = STOP;
        turnLeft();
        turnRight();
        return;
    }

    if (m_sensor_data[LEFT].average < m_sensor_data[RIGHT].average)
    {
        m_driving_state = SLOWRIGHT;
    }

    if (m_driving_state == SLOWRIGHT){
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT)*.95;
    }
    if (m_sensor_data[RIGHT].average < m_sensor_data[LEFT].average)
    {
        m_driving_state = SLOWLEFT;
    }

    if (m_driving_state == SLOWLEFT){
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT) *.95;
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
    }
}

void MotorController::turnLeft()
{
    if (m_sensor_data[LEFT].average >= 5.00 && m_driving_state != TURNRIGHT && m_driving_state != TURNLEFT)
    {
        m_initial = m_gyro_data;
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT) *.75;
        m_motor_data[BACK_LEFT] = MOTOR_OFF;

        m_driving_state = TURNLEFT;
    }else if (m_gyro_data >= m_initial + 90 && m_driving_state == TURNLEFT)
    {
        m_driving_state = DRIVING;
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_gyro_data = 0;
    }
}

void MotorController::turnRight()
{
    if (m_sensor_data[RIGHT].average >= 5.00 && m_driving_state != TURNRIGHT && m_driving_state != TURNLEFT)
    {
        m_initial = m_gyro_data;
        m_motor_data[BACK_RIGHT] = MOTOR_OFF;
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT) *.75;

        m_driving_state = TURNRIGHT;
    }else if (m_gyro_data <= m_initial - 90 && m_driving_state == TURNRIGHT)
    {
        m_driving_state = DRIVING;
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
    }
}