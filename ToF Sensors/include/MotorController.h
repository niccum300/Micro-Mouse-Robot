// Authors: Brayton Niccum
// Date: 2/7/2022

#include <global.h>
#include <SensorQueue.h>
#include <MotorQueue.h>
#include <GyroQueue.h>
#include <SENSOR_DATA.h>

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#define SENSOR_COUNT (3)
#define MOTOR_COUNT (2)

#define MIN_DISTANCE_FRONT (7.00)
#define MIN_DISTANCE (7.00)
#define MOTOR_OFF (0.00)
#define MOTOR_HALF (0.45)

#define BACK_RIGHT_AIN2_PIN (7)
#define BACK_RIGHT_AIN1_PIN (8)
#define BACK_LEFT_BIN2_PIN (10)
#define BACK_LEFT_BIN1_PIN (11)

enum MOTOR_ID {BACK_LEFT = 0, BACK_RIGHT = 1};
enum DRIVING_STATE {START, STOP, SLOWRIGHT, SLOWLEFT, TURNLEFT, TURNRIGHT, STRAIGHT};
enum SENSOR_TYPE {TOF_SENSOR, GYRO};

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
    void aquireSensorData();
    void updateMotorQueues();

    void updateDrivingState();
    void updateSensorPriority();

    // driving actions
    bool checkStartSignal(); // look for signal to tell bot to start moving
    void disableMotors();
    void centerInCorridor();

    bool checkForFrontCollision();
    bool checkForLeftCollision();
    bool checkForRightCollision();

    void turnLeft();
    void turnRight();

    void determineNextAction(); // tof's determine actions gyro just maintains bearing

    void updateBearing();
    

    SENSOR_DATA m_sensor_data[SENSOR_COUNT];
    float m_motor_data[MOTOR_COUNT];
    float m_gyro_data;
    float m_gyro_bearing = 0;
    float m_initial = 0;

    DRIVING_STATE m_driving_state = START;
    SENSOR_TYPE m_sensor_priority = GYRO;

};

#endif
