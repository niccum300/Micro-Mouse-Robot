#include <Gyro.h>

Gyro::Gyro(TwoWire &w, GyroQueue * p_gyro_queue): MPU6050(w), m_queue(p_gyro_queue)
{
}

Gyro::~Gyro()
{
}

void Gyro::Init()
{
    byte status = this->begin();
    this->setGyroConfig(0);
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
  
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    this->calcOffsets(true,true); // gyro and accelero
    Serial.println("Done!\n");
}

void Gyro::Update()
{
    this->update();
    pushData();
}

void Gyro::pushData()
{
    m_queue->Push(this->getAngleZ());
}