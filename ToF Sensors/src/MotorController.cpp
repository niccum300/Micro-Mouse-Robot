#include <MotorController.h>

// public
MotorController::MotorController()
{

}

void MotorController::Init()
{
    // nicks code for driver controlls
    m_motor_driver = MotorDriver();

    m_driving_state = STOP;
}

void MotorController::Update()
{
    aquireSensorData();
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

        m_driving_state = DRIVING;
        m_bearing = m_gyro_data;
        m_motor_driver.SetMotorDirection(FORWARD);
        

        return;
    }
    if (m_driving_state == TURNLEFT) {turnLeft(); return;}
    if (m_driving_state == TURNRIGHT) {turnRight(); return;}
    if (m_sensor_data[FRONT].average <= MIN_DISTANCE_FRONT && m_driving_state != BACKWARDS)
    {
        disableMotors();

        if (m_sensor_data[LEFT].average < 5.00 && m_sensor_data[RIGHT].average < 5.00)
        {
            reverse();
            return;
        }

        if (m_sensor_data[LEFT].average >= 5.00 && m_driving_state != TURNRIGHT)
        {
            turnLeft();
        }
        
        if (m_sensor_data[RIGHT].average >= 5.00 && m_driving_state != TURNRIGHT && m_driving_state != TURNLEFT){
            turnRight();
        }
        return;
    }

    if (m_sensor_data[LEFT].average < MIN_DISTANCE) 
    {
        m_l_adjust_factor += 1.0;
    }else{
        m_l_adjust_factor = 0.0;
    }
    if(m_sensor_data[RIGHT].average <  MIN_DISTANCE){
        m_r_adjust_factor += 1.0;
    }else{
        m_r_adjust_factor = 0.0;
    }

    useGyro();

}

void MotorController::useGyro()
{
    float left_factor = ((LEFT_MOTOR_ADJUST) + (m_gyro_data - m_bearing));
    float right_factor = ((RIGHT_MOTOR_ADJUST) - (m_gyro_data - m_bearing));
    

    if(left_factor > PWM_RESOULTION_32_BIT)
    {
        left_factor = PWM_RESOULTION_32_BIT;
    }else if (left_factor < 0.00)
    {
        left_factor = 0.00;
    }

    if(right_factor > PWM_RESOULTION_32_BIT)
    {
        right_factor = PWM_RESOULTION_32_BIT;
    }else if (right_factor < 0.00)
    {
        right_factor = 0.00;
    }

    m_motor_data[BACK_LEFT] = left_factor + m_l_adjust_factor;
    m_motor_data[BACK_RIGHT] = right_factor + m_r_adjust_factor;
}

void MotorController::reverse(){
    m_motor_driver.SetMotorDirection(REVERSE);
    m_driving_state = BACKWARDS;
}

void MotorController::turnLeft()
{
    if (m_sensor_data[LEFT].average >= 5.00 && m_driving_state != TURNRIGHT && m_driving_state != TURNLEFT)
    {
        m_initial = m_gyro_data;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST *.75;
        m_motor_data[BACK_LEFT] = MOTOR_OFF;

        m_driving_state = TURNLEFT;
    }else if (m_gyro_data >= m_initial + 82 && m_driving_state == TURNLEFT)
    {
        m_driving_state = DRIVING;
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        m_bearing = m_gyro_data;
    }
}

void MotorController::turnRight()
{
    if (m_sensor_data[RIGHT].average >= 5.00 && m_driving_state != TURNRIGHT && m_driving_state != TURNLEFT)
    {
        m_initial = m_gyro_data;
        m_motor_data[BACK_RIGHT] = MOTOR_OFF;
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;

        m_driving_state = TURNRIGHT;
    }else if (m_gyro_data <= m_initial - 82 && m_driving_state == TURNRIGHT)
    {
        m_driving_state = DRIVING;
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        
        m_bearing = m_gyro_data;
    }
}