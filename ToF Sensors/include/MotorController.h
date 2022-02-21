// Authors: Brayton Niccum
// Date: 2/7/2022

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#define SENSOR_COUNT (3)
#define MOTOR_COUNT (2)

#define MIN_DISTANCE (6.50)
#define MOTOR_OFF (0.00)
#define MOTOR_HALF (0.45)

#define BACK_RIGHT_AIN2_PIN (7)
#define BACK_RIGHT_AIN1_PIN (8)
#define BACK_LEFT_BIN2_PIN (10)
#define BACK_LEFT_BIN1_PIN (11)

enum MOTOR_ID {BACK_LEFT = 0, BACK_RIGHT = 1};

enum DRIVING_STATE {DRIVING, START, STOP, SLOWRIGHT, SLOWLEFT, TURN90};

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


class MotorController
{
public:
    MotorController();
    void Update();
    void Init();

private:
    void ZigZag();
    void aquireSensorData();
    void computeSensorData();
    void updateMotorQueues();
    void disableMotors();
    void setDrivingState(DRIVING_STATE state);
    void turn90();

    SENSOR_DATA m_sensor_data[SENSOR_COUNT];
    float m_motor_data[MOTOR_COUNT];
    float m_gyro_data;
    DRIVING_STATE m_driving_state;

};

#endif
