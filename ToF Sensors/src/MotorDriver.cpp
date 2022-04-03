#include <MotorDirver.h>

MotorDriver::MotorDriver()
{
    pinMode(STANDBY_PIN, OUTPUT);
    pinMode(BACK_RIGHT_AIN1_PIN, OUTPUT); //
    pinMode(BACK_RIGHT_AIN2_PIN, OUTPUT);
    pinMode(BACK_LEFT_BIN1_PIN, OUTPUT);
    pinMode(BACK_LEFT_BIN2_PIN, OUTPUT);

}

void MotorDriver::SetMotorDirection(MotorDirection p_direction)
{
    switch (p_direction)
    {
    case FORWARDS:
        digitalWrite(STANDBY_PIN, HIGH);
        digitalWrite(BACK_RIGHT_AIN1_PIN, LOW);
        digitalWrite(BACK_RIGHT_AIN2_PIN, HIGH);
        digitalWrite(BACK_LEFT_BIN1_PIN, HIGH);
        digitalWrite(BACK_LEFT_BIN2_PIN, LOW);
        m_current_direction = FORWARDS;
        break;
    
    case REVERSE:
        digitalWrite(STANDBY_PIN, LOW);
        digitalWrite(BACK_RIGHT_AIN1_PIN, HIGH);
        digitalWrite(BACK_RIGHT_AIN2_PIN, LOW);
        digitalWrite(BACK_LEFT_BIN1_PIN, LOW);
        digitalWrite(BACK_LEFT_BIN2_PIN, HIGH);
        digitalWrite(STANDBY_PIN, HIGH);
        m_current_direction = REVERSE;
        break;

    case LEFT_ZERO_POINT:
        digitalWrite(STANDBY_PIN, LOW);
        digitalWrite(BACK_RIGHT_AIN1_PIN, LOW);
        digitalWrite(BACK_RIGHT_AIN2_PIN, HIGH);
        digitalWrite(BACK_LEFT_BIN1_PIN, LOW);
        digitalWrite(BACK_LEFT_BIN2_PIN, HIGH);
        digitalWrite(STANDBY_PIN, HIGH);
        m_current_direction = LEFT_ZERO_POINT;
        break;

    case RIGHT_ZERO_POINT:
        digitalWrite(STANDBY_PIN, LOW);
        digitalWrite(BACK_RIGHT_AIN1_PIN, HIGH);
        digitalWrite(BACK_RIGHT_AIN2_PIN, LOW);
        digitalWrite(BACK_LEFT_BIN1_PIN, HIGH);
        digitalWrite(BACK_LEFT_BIN2_PIN, LOW);
        digitalWrite(STANDBY_PIN, HIGH);
        m_current_direction = RIGHT_ZERO_POINT;
        break;
    }

}

void MotorDriver::ToggleZeroDirection()
{
    if (m_current_direction == RIGHT_ZERO_POINT)
    {
        this->SetMotorDirection(LEFT_ZERO_POINT);
    }
    else if (m_current_direction == LEFT_ZERO_POINT)
    {
        this->SetMotorDirection(RIGHT_ZERO_POINT);
    }
}