// Authors: Brayton Niccum
// Date: 2/5/2022

#ifndef SENSORQUEUE_H
#define SENSORQUEUE_H

#include <SENSOR_DATA.h>

#define MAX_SIZE (20)
#define SENSOR_QUEUE_TYPE SENSOR_DATA

class SensorQueue{
public:
    SensorQueue();
    bool Push(SENSOR_QUEUE_TYPE p_in);
    SENSOR_QUEUE_TYPE Pop();
    SENSOR_QUEUE_TYPE Peak();
    void Reset();

private:
    bool isFull();
    bool isEmpty();

    SENSOR_QUEUE_TYPE m_queue[MAX_SIZE];
    int m_front = -1;
    int m_rear = -1;
};

#endif