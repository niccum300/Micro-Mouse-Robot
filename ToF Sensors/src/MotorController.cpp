#include <MotorController.h>

// public
MotorController::MotorController()
{

}

void MotorController::Init()
{
    // nicks code for driver controlls
    pinMode(STANDBY_PIN, OUTPUT);
    pinMode(BACK_RIGHT_AIN1_PIN, OUTPUT); //
    pinMode(BACK_RIGHT_AIN2_PIN, OUTPUT);
    pinMode(BACK_LEFT_BIN1_PIN, OUTPUT);
    pinMode(BACK_LEFT_BIN2_PIN, OUTPUT);

    digitalWrite(STANDBY_PIN, HIGH);
    digitalWrite(BACK_RIGHT_AIN1_PIN, LOW);
    digitalWrite(BACK_RIGHT_AIN2_PIN, HIGH);
    digitalWrite(BACK_LEFT_BIN1_PIN, HIGH);
    digitalWrite(BACK_LEFT_BIN2_PIN, LOW);

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