#include <SensorQueue.h>

// public
SensorQueue::SensorQueue()
{
}

bool SensorQueue::Push(QUEUE_TYPE p_in)
{
    if(isFull()) return false;

    if(m_front == -1) m_front = 0;
    m_rear++;

    m_queue[m_rear] = p_in;

    return true;
}

QUEUE_TYPE SensorQueue::Pop()
{
    if(!isEmpty())
    {
        QUEUE_TYPE value = m_queue[m_front];
        m_front++;

        if (m_front > m_rear){
            m_front = -1;
            m_rear = -1;
        }

        return value;
    }
}

QUEUE_TYPE SensorQueue::Peak()
{
    if(!isEmpty())
    {
        return m_queue[m_front];
    }
}

void SensorQueue::Reset()
{
    m_front = -1;
    m_rear = -1;
}

// private
bool SensorQueue::isFull()
{
    if(m_front == 0 && m_rear == MAX_SIZE - 1)
    {
        return true;
    }

    return false;
}

bool SensorQueue::isEmpty()
{
    if(m_front == -1) return true;

    return false;
}