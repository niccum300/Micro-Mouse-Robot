#include <MotorController.h>

// public
MotorController::MotorController()
{
    Entropy.Initialize();
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
    runStateMachine();
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
    //Serial.printf("Gyro: %f \n", m_gyro_data);
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

void MotorController::generate_numbers()
{
    int num = Entropy.random(12);
    Serial.println(num);
    if (num >= 6){
        Serial.println("straight");
        left_turns = false;
        right_turns = false;
        straight = true;
    }else if (num % 3 == 1){
        Serial.println("left turns");
        right_turns = false;
        left_turns = true;
        straight = false;
    }else{
         right_turns = true;;
         left_turns = false;
         straight = false;
    }
}

void MotorController::checkSurroundings()
{
    INTERSECTION intersection;

    if (m_sensor_data[FRONT].average <= MIN_DISTANCE_FRONT && m_sensor_data[RIGHT].average <= MIN_DISTANCE_FRONT && m_sensor_data[LEFT].average <= MIN_DISTANCE_FRONT)
    {
        disableMotors();
        intersection.REAR = REAR_OPENING;
        m_driving_state = BACKWARDS;
        m_delay = true;
        return;
        
    }else{

        if (m_delay)
        {
            int ecndoerCount = RightEncoderCount;
            if (ecndoerCount >= 260){
                m_delay = false;
            
            }else{
                return;
            }
        }

        if (m_sensor_data[LEFT].average >= 5.00)
        {
            intersection.LEFT = LEFT_OPENING;
        }

        if (m_sensor_data[RIGHT].average >= 5.00)
        {
            intersection.RIGHT = RIGHT_OPENING;
        }

        if (m_sensor_data[FRONT].average > 10.00)
        {
            intersection.FORNT = FRONT_OPENING;
        }

        if (intersection.RIGHT != NO_OPENING && intersection.LEFT != NO_OPENING && intersection.FORNT != NO_OPENING)
        {
            generate_numbers();

            if (right_turns == true)
            {
                m_detected_edge = RIGHTEDGE;
                m_driving_state = SLOWFORWARDS;
                RightEncoderCount = 0;
                m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
                m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;

                return;
            }else if (left_turns == true)
            {
                m_detected_edge = LEFTEDGE;
                m_driving_state = SLOWFORWARDS;
                RightEncoderCount = 0;
                m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
                m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;

                return;
            }else if (straight == true)
            {
               RightEncoderCount = 0;
                m_delay = true;
                return;
            }

        }else if (intersection.RIGHT != NO_OPENING && intersection.LEFT != NO_OPENING && intersection.FORNT == NO_OPENING)
        {
            int num = Entropy.random(12);

            if (num % 2 == 0)
            {
                m_detected_edge = RIGHTEDGE;
                m_driving_state = SLOWFORWARDS;
                RightEncoderCount = 0;
                m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
                m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;
                return;

            }else{
                m_detected_edge = LEFTEDGE;
                m_driving_state = SLOWFORWARDS;
                RightEncoderCount = 0;
                m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
                m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;
                return;
            }

            

        }else if (intersection.RIGHT != NO_OPENING && intersection.LEFT == NO_OPENING && intersection.FORNT == NO_OPENING)
        {

            m_detected_edge = RIGHTEDGE;
            m_driving_state = SLOWFORWARDS;
            RightEncoderCount = 0;
            m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
            m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;
            return;


        }else if (intersection.RIGHT == NO_OPENING && intersection.LEFT != NO_OPENING && intersection.FORNT == NO_OPENING)
        {
            m_detected_edge = LEFTEDGE;
            m_driving_state = SLOWFORWARDS;
            RightEncoderCount = 0;
            m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
            m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;
            return;

        }
        else if (intersection.RIGHT != NO_OPENING && intersection.LEFT == NO_OPENING && intersection.FORNT != NO_OPENING)
        {
            int num = Entropy.random(12);

            if (num % 2 == 0)
            {
                m_detected_edge = RIGHTEDGE;
                m_driving_state = SLOWFORWARDS;
                RightEncoderCount = 0;
                m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
                m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;
                return;

            }else{
                RightEncoderCount = 0;
                m_delay = true;
                return;
            }

        }else if (intersection.RIGHT == NO_OPENING && intersection.LEFT != NO_OPENING && intersection.FORNT != NO_OPENING)
        {
            int num = Entropy.random(12);

            if (num % 2 == 0)
            {
                m_detected_edge = LEFTEDGE;
                m_driving_state = SLOWFORWARDS;
                RightEncoderCount = 0;
                m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
                m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;
                return;

            }else{
                RightEncoderCount = 0;
                m_delay = true;
                return;
            }
        }
    }

}

void MotorController::runStateMachine()
{
    switch (m_driving_state)
    {
    case START:
        m_driving_state = STRAIGHT;
        m_motor_driver.SetMotorDirection(FORWARDS);
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        break;

    case STRAIGHT:
        ZigZag();


        if (m_turn_delay != NONE){
            int encoder = RightEncoderCount; 
            if (encoder >= 50)
            {
                m_turn_delay = NONE;
            }
        }else{
            checkSurroundings();
        }

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
        int ecndoerCount = RightEncoderCount;
        if (ecndoerCount >= 190){
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

void MotorController::ZigZag()
{

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
    if (m_gyro_data >= m_initial + 86 && m_turn_delay != TURNLEFT)
    {
        m_turn_delay = TURNLEFT;
        m_driving_state = STRAIGHT;
        m_bearing = m_gyro_data;
        m_motor_driver.SetMotorDirection(FORWARDS);
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        RightEncoderCount = 0;
        turn_made = true;
    }
}

void MotorController::turnRight()
{
    if (m_gyro_data <= m_initial - 85 && m_turn_delay != TURNRIGHT)
    {
        m_turn_delay = TURNRIGHT;
        m_driving_state = STRAIGHT;
        m_bearing = m_gyro_data;
        m_motor_driver.SetMotorDirection(FORWARDS);
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        RightEncoderCount = 0;
        turn_made = true;
    }
}

void MotorController::turn180()
{
    Serial.println(m_delay_count);
    if(m_delay_count == 30)
    {
        m_delay_count = 0;
        m_delay = false;
        m_initial = m_gyro_data;
        m_motor_driver.SetMotorDirection(LEFT_ZERO_POINT);
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST * .3;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST * .3;

        return;
    }else if(m_delay){
        m_delay_count++;
        return;
    }
    
    if (m_gyro_data >= m_initial + 172 && m_driving_state == BACKWARDS)
    {
        m_driving_state = STRAIGHT;
        m_motor_driver.SetMotorDirection(FORWARDS);
        m_motor_data[BACK_LEFT] = LEFT_MOTOR_ADJUST;
        m_motor_data[BACK_RIGHT] = RIGHT_MOTOR_ADJUST;
        
        m_bearing = m_gyro_data;
    }
}