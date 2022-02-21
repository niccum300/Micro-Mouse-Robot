#include <GyroQueue.h>

// public
GyroQueue::GyroQueue()
{
}

bool GyroQueue::Push(GYRO_QUEUE_TYPE p_in)
{
    if(isFull()) return false;

    if(m_front == -1) m_front = 0;
    m_rear++;

    m_queue[m_rear] = p_in;

    return true;
}

GYRO_QUEUE_TYPE GyroQueue::Pop()
{
    if(!isEmpty())
    {
        GYRO_QUEUE_TYPE value = m_queue[m_front];
        m_front++;

        if (m_front > m_rear){
            m_front = -1;
            m_rear = -1;
        }

        return value;
    }
}

GYRO_QUEUE_TYPE GyroQueue::Peak()
{
    if(!isEmpty())
    {
        return m_queue[m_front];
    }
}

void GyroQueue::Reset()
{
    m_front = -1;
    m_rear = -1;
}

// private
bool GyroQueue::isFull()
{
    if(m_front == 0 && m_rear == MAX_SIZE - 1)
    {
        return true;
    }

    return false;
}

bool GyroQueue::isEmpty()
{
    if(m_front == -1) return true;

    return false;
}