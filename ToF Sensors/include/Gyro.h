#ifndef GYRO_H
#define GYRO_H

#include <MPU6050_light.h>
#include <GyroQueue.h>

class Gyro : MPU6050
{
public:
    Gyro(TwoWire &w, GyroQueue * p_gyro_queue);
    ~Gyro();

    void Update();
    void Init();
private:
    void pushData();

    GyroQueue * m_queue;
};

#endif