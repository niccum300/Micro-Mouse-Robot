// Authors: Brayton Niccum
// Date: 2/5/2022

#ifndef SENSORQUEUE_H
#define SENSORQUEUE_H

#include <SENSOR_DATA.h>

#define MAX_SIZE (20)
#define QUEUE_TYPE SENSOR_DATA_BUNDLE

class SensorQueue{
public:
    SensorQueue();
    bool Push(QUEUE_TYPE p_in);
    QUEUE_TYPE Pop();
    QUEUE_TYPE Peak();
    void Reset();

private:
    bool isFull();
    bool isEmpty();

    QUEUE_TYPE m_queue[MAX_SIZE];
    int m_front = -1;
    int m_rear = -1;
};

#endif