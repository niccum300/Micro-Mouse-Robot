#include <MotorQueue.h>

// public
MotorQueue::MotorQueue()
{
}

bool MotorQueue::Push(MOTOR_QUEUE_TYPE p_in)
{
    if(isFull()) return false;

    if(m_front == -1) m_front = 0;
    m_rear++;

    m_queue[m_rear] = p_in;

    return true;
}

MOTOR_QUEUE_TYPE MotorQueue::Pop()
{
    if(!isEmpty())
    {
        MOTOR_QUEUE_TYPE value = m_queue[m_front];
        m_front++;

        if (m_front > m_rear){
            m_front = -1;
            m_rear = -1;
        }

        return value;
    }
}

MOTOR_QUEUE_TYPE MotorQueue::Peak()
{
    if(!isEmpty())
    {
        return m_queue[m_front];
    }
}

void MotorQueue::Reset()
{
    m_front = -1;
    m_rear = -1;
}

// private
bool MotorQueue::isFull()
{
    if(m_front == 0 && m_rear == MAX_SIZE - 1)
    {
        return true;
    }

    return false;
}

bool MotorQueue::isEmpty()
{
    if(m_front == -1) return true;

    return false;
}