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
    case FORWARD:
        digitalWrite(STANDBY_PIN, HIGH);
        digitalWrite(BACK_RIGHT_AIN1_PIN, LOW);
        digitalWrite(BACK_RIGHT_AIN2_PIN, HIGH);
        digitalWrite(BACK_LEFT_BIN1_PIN, HIGH);
        digitalWrite(BACK_LEFT_BIN2_PIN, LOW);
        break;
    
    case REVERSE:
        digitalWrite(STANDBY_PIN, LOW);
        digitalWrite(BACK_RIGHT_AIN1_PIN, HIGH);
        digitalWrite(BACK_RIGHT_AIN2_PIN, LOW);
        digitalWrite(BACK_LEFT_BIN1_PIN, LOW);
        digitalWrite(BACK_LEFT_BIN2_PIN, HIGH);
        digitalWrite(STANDBY_PIN, HIGH);
        break;
    case ZERO_POINT:
        digitalWrite(STANDBY_PIN, LOW);
        digitalWrite(BACK_RIGHT_AIN1_PIN, LOW);
        digitalWrite(BACK_RIGHT_AIN2_PIN, HIGH);
        digitalWrite(BACK_LEFT_BIN1_PIN, LOW);
        digitalWrite(BACK_LEFT_BIN2_PIN, HIGH);
        digitalWrite(STANDBY_PIN, HIGH);
    }

}