// Authors: Brayton Niccum
// Date: 2/7/2022

#ifndef MOTOR_QUEUE_H
#define MOTOR_QUEUE_H

#define MAX_SIZE (20)
#define MOTOR_QUEUE_TYPE float

class MotorQueue{
public:
    MotorQueue();
    bool Push(MOTOR_QUEUE_TYPE p_in);
    MOTOR_QUEUE_TYPE Pop();
    MOTOR_QUEUE_TYPE Peak();
    void Reset();


    bool isFull();
    bool isEmpty();

    MOTOR_QUEUE_TYPE m_queue[MAX_SIZE];
    int m_front = -1;
    int m_rear = -1;
};

#endif