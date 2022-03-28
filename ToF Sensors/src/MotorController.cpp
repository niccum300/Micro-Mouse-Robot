#include <MotorController.h>

// public
MotorController::MotorController()
{

}

void MotorController::Init()
{
    // nicks code for driver controlls
    m_motor_driver = MotorDriver();

    m_driving_state = START;
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

    //Serial.printf("\nLEFT: %d || RIGHT: %d\n", LeftEncoderCount, RightEncoderCount);
    //Serial.printf("Front: %f | Left: %f | Right: %f \n", m_sensor_data[FRONT].average, m_sensor_data[LEFT].average, m_sensor_data[RIGHT].average);
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
<<<<<<< HEAD
        return;
    }

    if (m_driving_state == STRAIGHT)
    {
        if (m_sensor_data[FRONT].average <= MIN_DISTANCE_FRONT)
        {

            if(m_sensor_data[RIGHT].average >= 5.00)
            {
                m_driving_state = TURNRIGHT;
                m_initial = m_gyro_data;
                m_motor_data[BACK_RIGHT] = MOTOR_OFF;
                m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;

            }else if(m_sensor_data[LEFT].average >= 5.00)
            {
                m_driving_state = TURNLEFT;
                m_initial = m_gyro_data;
                m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST *.75;
                m_motor_data[BACK_LEFT] = MOTOR_OFF;
            }else{
                m_driving_state = STOP;
            }  
        }
        else if(m_sensor_data[RIGHT].average >= 5.00)
        {
            m_driving_state = TURNRIGHT;
            m_initial = m_gyro_data;
            m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;
            m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;

        }else if(m_sensor_data[LEFT].average >= 5.00)
        {
            m_driving_state = TURNLEFT;
            m_initial = m_gyro_data;
            m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST *.75;
            m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
=======

        m_driving_state = DRIVING;
        m_bearing = m_gyro_data;
        m_motor_driver.SetMotorDirection(FORWARD);
        

        return;
    }
    if (m_driving_state == TURNLEFT) {turnLeft(); return;}
    if (m_driving_state == TURNRIGHT) {turnRight(); return;}
    if (m_driving_state == BACKWARDS) {turn180(); return;}
    if (m_sensor_data[FRONT].average <= MIN_DISTANCE_FRONT && m_driving_state != BACKWARDS)
    {
        disableMotors();

        if (m_sensor_data[LEFT].average < 5.00 && m_sensor_data[RIGHT].average < 5.00)
        {
            turn180();
            return;
        }

        if (m_sensor_data[LEFT].average >= 5.00 && m_driving_state != TURNRIGHT)
        {
            turnLeft();
        }
        
        if (m_sensor_data[RIGHT].average >= 5.00 && m_driving_state != TURNRIGHT && m_driving_state != TURNLEFT){
            turnRight();
>>>>>>> addZeroPoint
        }
    }

    switch (m_driving_state)
    {
    case START:
        m_driving_state = STRAIGHT;
        break;

    case STRAIGHT:
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
        break;
    
    case STOP:
        disableMotors();
        break;

    case TURNLEFT:
        turnLeft();
        break;
    
    case TURNRIGHT:
        turnRight();
        break;
    }

    // if (m_driving_state == TURNLEFT) {turnLeft(); return;}
    // if (m_driving_state == TURNRIGHT) {turnRight(); return;}
    // if (m_sensor_data[FRONT].average <= MIN_DISTANCE_FRONT)
    // {
    //     disableMotors();
    //     //m_driving_state = STOP;
    //     turnLeft();
    //     turnRight();
    //     return;
    // }

}

void MotorController::useGyro()
{
    float left_factor = 0.0;
    float right_factor = 0.0;

    if (m_driving_state == BACKWARDS)
    {
        left_factor = ((LEFT_MOTOR_ADJUST) - (m_gyro_data - m_bearing));
        right_factor = ((RIGHT_MOTOR_ADJUST) + (m_gyro_data - m_bearing));
    }else {
        left_factor = ((LEFT_MOTOR_ADJUST) + (m_gyro_data - m_bearing));
        right_factor = ((RIGHT_MOTOR_ADJUST) - (m_gyro_data - m_bearing));
    }
    
    

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

<<<<<<< HEAD
=======
void MotorController::reverse(){
    m_motor_driver.SetMotorDirection(REVERSE);
    m_driving_state = BACKWARDS;
}
>>>>>>> addZeroPoint

void MotorController::turnLeft()
{
    if (m_gyro_data >= m_initial + 84)
    {
        m_driving_state = STRAIGHT;
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        m_bearing = m_gyro_data;
    }
}

void MotorController::turnRight()
{
    if (m_gyro_data <= m_initial - 84)
    {
        Serial.print("here");
        m_driving_state = STRAIGHT;
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        
        m_bearing = m_gyro_data;
    }
}

void MotorController::turn180()
{
    if (m_driving_state != BACKWARDS)
    {
        m_driving_state = BACKWARDS;
        m_delay = true;
        return;
    }else if(m_delay_count == 30)
    {
        m_delay_count = 0;
        m_delay = false;
        m_initial = m_gyro_data;
        m_motor_driver.SetMotorDirection(ZERO_POINT);
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;

        return;
    }else if(m_delay){
        m_delay_count++;
        return;
    }
    
    if (m_gyro_data >= m_initial + 172 && m_driving_state == BACKWARDS)
    {
        m_driving_state = DRIVING;
        m_motor_driver.SetMotorDirection(FORWARD);
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        
        m_bearing = m_gyro_data;
    }
}