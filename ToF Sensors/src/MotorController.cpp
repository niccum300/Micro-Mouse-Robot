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

}

void MotorController::Update()
{
    aquireSensorData();
    updateDrivingState();
    updateMotorQueues();
}

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

void MotorController::updateDrivingState()
{
    /* 
    State machine checks current state then calls proper function
    */
    switch (m_driving_state)
    {
    case START:
        Serial.print("start");
        disableMotors();
        m_driving_state = STRAIGHT;
        m_gyro_bearing = m_gyro_data;
        Init();
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        break;

    case STOP:
        break;
    
    case STRAIGHT:
        updateSensorPriority();

        if (m_sensor_priority == GYRO)
        {
            updateBearing();
        }else{
            determineNextAction();
        }
        break;
    
    case TURNRIGHT:
        turnRight();
        break;

    case TURNLEFT:
        turnLeft();
        break;    

    case SLOWLEFT:
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT) *.95;
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_driving_state = STRAIGHT;
        m_gyro_bearing = m_gyro_data;
        break;

    case SLOWRIGHT:
        m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
        m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT)*.95;
        m_driving_state = STRAIGHT;
        m_gyro_bearing = m_gyro_data;
        break;
    
    }
}

bool MotorController::checkStartSignal()
{
    Serial.println(checkForFrontCollision());
    return checkForFrontCollision();
}

void MotorController::disableMotors()
{
    if (m_driving_state != START)
    {
        m_driving_state = STOP;
    }
    m_motor_data[BACK_LEFT] = MOTOR_OFF * PWM_RESOULTION_32_BIT;
    m_motor_data[BACK_RIGHT] = MOTOR_OFF * PWM_RESOULTION_32_BIT;
}

bool MotorController::checkForFrontCollision()
{
    if (m_sensor_data[FRONT].average < MIN_DISTANCE_FRONT) {return true;}
    
    return false;
}

bool MotorController::checkForLeftCollision()
{
    if (m_sensor_data[LEFT].average < MIN_DISTANCE) {return true;}
    
    return false;
}

bool MotorController::checkForRightCollision()
{
    if (m_sensor_data[RIGHT].average < MIN_DISTANCE) {return true;}
    
    return false;
}

void MotorController::updateSensorPriority()
{
    switch (m_sensor_priority)
    {
    case TOF_SENSOR:
            if (!checkForFrontCollision() || !checkForLeftCollision() || !checkForRightCollision())
            {
                m_sensor_priority = GYRO;
                m_gyro_bearing = m_gyro_data;
            }
        break;
    
    case GYRO:
            if (checkForFrontCollision() || checkForLeftCollision() || checkForRightCollision())
            {
                m_sensor_priority = TOF_SENSOR;
            }
        break;

    default:
        break;
    }

}

void MotorController::determineNextAction()
{
    if (checkForFrontCollision())
    {
        if (checkForLeftCollision() && checkForRightCollision())
        {
            disableMotors();
            return;
        }
        else if (checkForRightCollision() && !checkForLeftCollision()) 
        {
            turnLeft();
            return;
        }
        else if (checkForLeftCollision() && !checkForRightCollision())
        {
            turnRight();
            return;
        }else{
                m_motor_data[BACK_LEFT] = (0.1 * PWM_RESOULTION_32_BIT);
                m_motor_data[BACK_RIGHT] = (0.1 * PWM_RESOULTION_32_BIT);
        }
    }    
    else
    {
        centerInCorridor();
        return;
    }
    
}

void MotorController::updateBearing()
{
    Serial.printf("\ndiff %f \n", m_gyro_data  - m_gyro_bearing);
    Serial.printf("\ndiff %f \n", m_gyro_bearing);
    if (m_gyro_bearing - m_gyro_data < 0)
    {
        m_motor_data[BACK_LEFT] = ((MOTOR_HALF * PWM_RESOULTION_32_BIT) +  ((MOTOR_HALF * PWM_RESOULTION_32_BIT)*0.1));
        m_motor_data[BACK_RIGHT] = ((MOTOR_HALF * PWM_RESOULTION_32_BIT) -  ((MOTOR_HALF * PWM_RESOULTION_32_BIT)*0.1));
        return;
    }

    if (m_gyro_bearing - m_gyro_data > 0)
    {
        Serial.print("negative gyro");
        m_motor_data[BACK_LEFT] = ((MOTOR_HALF * PWM_RESOULTION_32_BIT) -  ((MOTOR_HALF * PWM_RESOULTION_32_BIT)*0.1));
        m_motor_data[BACK_RIGHT] = ((MOTOR_HALF * PWM_RESOULTION_32_BIT) +  ((MOTOR_HALF * PWM_RESOULTION_32_BIT)*0.1));
        return;
    }


    m_motor_data[BACK_LEFT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
    m_motor_data[BACK_RIGHT] = (MOTOR_HALF * PWM_RESOULTION_32_BIT);
}

void MotorController::centerInCorridor()
{

    if (m_sensor_data[LEFT].average < m_sensor_data[RIGHT].average)
    {
        m_driving_state = SLOWRIGHT;
    }

    if (m_sensor_data[RIGHT].average < m_sensor_data[LEFT].average)
    {
        m_driving_state = SLOWLEFT;
    }
}

void MotorController::turnLeft()
{
    if (m_driving_state != TURNLEFT)
    {
        m_driving_state = TURNLEFT;
        m_initial = m_gyro_data;
        m_motor_data[BACK_RIGHT] = (PWM_RESOULTION_32_BIT * MOTOR_HALF);
        m_motor_data[BACK_LEFT] = MOTOR_OFF;
    }

    if (m_gyro_data >= m_initial + 90)
    {
        m_driving_state = STRAIGHT;
        m_gyro_bearing = m_gyro_data;
    }
}

void MotorController::turnRight()
{
    if (m_driving_state != TURNRIGHT)
    {
        m_driving_state = TURNRIGHT;
        m_initial = m_gyro_data;
        m_motor_data[BACK_RIGHT] = MOTOR_OFF;
        m_motor_data[BACK_LEFT] = (PWM_RESOULTION_32_BIT * MOTOR_HALF);
    }

    if (m_gyro_data <= m_initial - 90)
    {
        m_driving_state = STRAIGHT;
        m_gyro_bearing = m_gyro_data;
    }
}