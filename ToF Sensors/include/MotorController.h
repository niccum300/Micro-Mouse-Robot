// Authors: Brayton Niccum
// Date: 2/7/2022

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#define SENSOR_COUNT (3)
#define MOTOR_COUNT (2)

#define MIN_DISTANCE_FRONT (6.00)
#define MIN_DISTANCE (1.50)
#define MOTOR_OFF (0.00)
#define MOTOR_HALF (0.45)

#define LEFT_MOTOR_ADJUST ((PWM_RESOULTION_32_BIT * MOTOR_HALF) + ((PWM_RESOULTION_32_BIT * MOTOR_HALF) * 0.06))
#define RIGHT_MOTOR_ADJUST ((PWM_RESOULTION_32_BIT * MOTOR_HALF))

#define BACK_RIGHT_AIN2_PIN (33)
#define BACK_RIGHT_AIN1_PIN (34)
#define BACK_LEFT_BIN2_PIN (10)
#define BACK_LEFT_BIN1_PIN (38)
#define STANDBY_PIN (36)

enum MOTOR_ID {BACK_LEFT = 0, BACK_RIGHT = 1};

enum DRIVING_STATE {STRAIGHT, START, STOP, TURNLEFT, TURNRIGHT};

#include <global.h>
#include <SensorQueue.h>
#include <MotorQueue.h>
#include <GyroQueue.h>
#include <SENSOR_DATA.h>

// Senosor Data Queues
extern SensorQueue FrontSensorQ;
extern SensorQueue LeftSensorQ;
extern SensorQueue RightSensorQ;

// Motor Data Queues
extern MotorQueue BackLeftMotorQ;
extern MotorQueue BackRightMotorQ;

//Gyro Queue
extern GyroQueue GyroQ;

extern int LeftEncoderCount;
extern int RightEncoderCount;


class MotorController
{
public:
    MotorController();
    void Update();
    void Init();

private:
    void ZigZag();
    void aquireSensorData();
    void updateMotorQueues();
    void disableMotors();
    void turnLeft();
    void turnRight();
    void useGyro();

    SENSOR_DATA m_sensor_data[SENSOR_COUNT];
    float m_motor_data[MOTOR_COUNT];
    float m_gyro_data;
    float m_initial = 0.00;
    float m_r_adjust_factor = 0.00;
    float m_l_adjust_factor = 0.00;
    float m_bearing = 0.00;
    DRIVING_STATE m_driving_state;

};

#endif
