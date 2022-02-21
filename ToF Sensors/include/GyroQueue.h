// Authors: Brayton Niccum
// Date: 2/7/2022

#ifndef GYRO_QUEUE_H
#define GYRO_QUEUE_H

#define MAX_SIZE (20)
#define GYRO_QUEUE_TYPE float

class GyroQueue{
public:
    GyroQueue();
    bool Push(GYRO_QUEUE_TYPE p_in);
    GYRO_QUEUE_TYPE Pop();
    GYRO_QUEUE_TYPE Peak();
    void Reset();

private:
    bool isFull();
    bool isEmpty();

    GYRO_QUEUE_TYPE m_queue[MAX_SIZE];
    int m_front = -1;
    int m_rear = -1;
};

#endif