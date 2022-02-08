// Authors: Brayton Niccum
// Date: 2/7/2022

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#define SENSOR_COUNT (3)
#define MOTOR_COUNT (4)

#define MIN_DISTANCE (0.9)
#define MOTOR_OFF (0.00)
#define MOTOR_HALF (0.50)

enum MOTOR_ID {FRONT_LEFT = 0, FRONT_RIGHT = 1, BACK_LEFT = 2, BACK_RIGHT = 3};

enum DRIVING_STATE {DRIVING, STOP};

#include <global.h>
#include <SensorQueue.h>
#include <MotorQueue.h>
#include <SENSOR_DATA.h>

// Senosor Data Queues
extern SensorQueue FrontSensorQ;
extern SensorQueue LeftSensorQ;
extern SensorQueue RightSensorQ;

// Motor Data Queues
extern MotorQueue FrontLeftMotorQ;
extern MotorQueue FrontRightMotorQ;
extern MotorQueue BackLeftMotorQ;
extern MotorQueue BackRightMotorQ;

class MotorController
{
public:
    MotorController();
    void Update();

private:
    void aquireSensorData();
    void computeSensorData();
    void updateMotorQueues();
    void disableMotors();
    void setDrivingState(DRIVING_STATE state);

    SENSOR_DATA m_sensor_data[SENSOR_COUNT];
    float m_motor_data[MOTOR_COUNT];
    DRIVING_STATE m_driving_state;

};

#endif