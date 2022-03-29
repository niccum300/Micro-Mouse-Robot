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
        return;
    }

    if (m_driving_state == STRAIGHT)
    {
        if (m_sensor_data[FRONT].average <= MIN_DISTANCE_FRONT)
        {
            disableMotors();
            m_driving_state = BACKWARDS;
            m_delay = true;
            
        }
        else if(m_sensor_data[RIGHT].average >= 5.00)
        {
            m_detected_edge = RIGHTEDGE;
            m_driving_state = SLOWFORWARDS;
            disableMotors();
            m_ecnoder_count = ((LeftEncoderCount + RightEncoderCount) / 2);
            m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .5;
            m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST *.5;
        
        }else if(m_sensor_data[LEFT].average >= 5.00)
        {
            m_driving_state = SLOWFORWARDS;
            m_detected_edge = LEFTEDGE;
            disableMotors();
            m_ecnoder_count = ((LeftEncoderCount + RightEncoderCount) / 2);
            m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .5;
            m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST *.5;
        }
        
    }

    switch (m_driving_state)
    {
    case START:
        m_driving_state = STRAIGHT;
        m_motor_driver.SetMotorDirection(FORWARDS);
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
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
    case BACKWARDS:
        turn180();
        break;

    case SLOWFORWARDS:
        Serial.print("Slow");
        int ecndoerCount = ((LeftEncoderCount + RightEncoderCount) / 2) - m_ecnoder_count;
        Serial.print("Encoders");
        Serial.println(ecndoerCount);
        if (ecndoerCount >= 80){
            if (m_detected_edge == RIGHTEDGE)
            {
                m_driving_state = TURNRIGHT;
                m_initial = m_gyro_data;
                m_motor_driver.SetMotorDirection(RIGHT_ZERO_POINT);
                m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
                m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;
            }else{
                m_driving_state = TURNLEFT;
                m_initial = m_gyro_data;
                m_motor_driver.SetMotorDirection(LEFT_ZERO_POINT);
                m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
                m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;
            }
        }
        break;
    }

}

void MotorController::useGyro()
{
    float left_factor = 0.0;
    float right_factor = 0.0;

    left_factor = ((LEFT_MOTOR_ADJUST) + (m_gyro_data - m_bearing));
    right_factor = ((RIGHT_MOTOR_ADJUST) - (m_gyro_data - m_bearing));
    
    

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

void MotorController::turnLeft()
{
    if (m_gyro_data >= m_initial + 84 && m_turn_delay != TURNLEFT)
    {
        m_turn_delay = TURNLEFT;
        m_motor_driver.SetMotorDirection(FORWARDS);
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        m_ecnoder_count = ((LeftEncoderCount + RightEncoderCount) / 2);
    }else if(m_turn_delay == TURNLEFT)
    {
        int encoder = ((LeftEncoderCount + RightEncoderCount) / 2) - m_ecnoder_count; 
        if (encoder >= 25)
        {
            m_driving_state = STRAIGHT;
            m_turn_delay = STRAIGHT;
            m_ecnoder_count = 0;
            m_bearing = m_gyro_data;
        }
    }
}

void MotorController::turnRight()
{
    if (m_gyro_data <= m_initial - 84 && m_turn_delay != TURNRIGHT)
    {
        m_turn_delay = TURNRIGHT;
        m_motor_driver.SetMotorDirection(FORWARDS);
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        m_ecnoder_count = ((LeftEncoderCount + RightEncoderCount) / 2);
    }else if(m_turn_delay == TURNRIGHT)
    {
        int encoder = ((LeftEncoderCount + RightEncoderCount) / 2) - m_ecnoder_count; 
        if (encoder >= 25)
        {
            m_driving_state = STRAIGHT;
            m_turn_delay = STRAIGHT;
            m_ecnoder_count = 0;
            m_bearing = m_gyro_data;
        }
    }
}

void MotorController::turn180()
{
    Serial.println(m_delay_count);
    if(m_delay_count == 30)
    {
        Serial.println("HERE 30");
        m_delay_count = 0;
        m_delay = false;
        m_initial = m_gyro_data;
        m_motor_driver.SetMotorDirection(LEFT_ZERO_POINT);
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;

        return;
    }else if(m_delay){
        Serial.println("HERE True");
        m_delay_count++;
        return;
    }
    
    if (m_gyro_data >= m_initial + 172 && m_driving_state == BACKWARDS)
    {
        Serial.println("HERE");
        m_driving_state = STRAIGHT;
        m_motor_driver.SetMotorDirection(FORWARDS);
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        
        m_bearing = m_gyro_data;
    }
}