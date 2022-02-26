// Authors: Brayton Niccum
#include <IntervalTimer.h>

#include <global.h>
#include <TOF.h>
#include <Motor.h>
#include <Gyro.h>
#include <SensorQueue.h>
#include <MotorQueue.h>
#include <GyroQueue.h>
#include <MotorController.h>

//Motor
#define FRONT_RIGHT_MOTOR_PIN (29)
#define FRONT_LEFT_MOTOR_PIN (30)
#define BACK_RIGHT_MOTOR_PIN (6)
#define BACK_LEFT_MOTOR_PIN (9)
// 
#define FRONT_XSHUT_PIN_MASK (0x2)
#define FRONT_SENSOR_ADDRESS (0x10)

#define LEFT_XSHUT_PIN_MASK (0x1)
#define LEFT_SENSOR_ADDRESS (0x11)

#define RIGHT_XSHUT_PIN_MASK (0x6)
#define RIGHT_SENSOR_ADDRESS (0x12)

// Senosor Data Queues
SensorQueue FrontSensorQ;
SensorQueue LeftSensorQ;
SensorQueue RightSensorQ;

// Motor Data Queues
MotorQueue BackLeftMotorQ;
MotorQueue BackRightMotorQ;

//Gyro Queue
GyroQueue GyroQ;

TOF FrontSensor(&PORTC_PCR2, &GPIOC_PDDR, &GPIOC_PDOR, MASK(FRONT_XSHUT_PIN_MASK), FRONT_SENSOR_ADDRESS, &FrontSensorQ);
TOF LeftSensor(&PORTC_PCR1, &GPIOC_PDDR, &GPIOC_PDOR, MASK(LEFT_XSHUT_PIN_MASK), LEFT_SENSOR_ADDRESS, &LeftSensorQ);
TOF RightSensor(&PORTD_PCR6, &GPIOD_PDDR, &GPIOD_PCOR, MASK(RIGHT_XSHUT_PIN_MASK), RIGHT_SENSOR_ADDRESS, &RightSensorQ);

Motor BackRightMotor(BACK_RIGHT_MOTOR_PIN, PWM_RESOULTION_32_BIT, &BackRightMotorQ);
Motor BackLeftMotor(BACK_LEFT_MOTOR_PIN, PWM_RESOULTION_32_BIT, &BackLeftMotorQ);

Gyro GyroMpu(Wire, &GyroQ);

MotorController motorController;

IntervalTimer sensorTimer;
bool SensorStatus = false;
bool Driving = false;

// configure pins to output for shutting down each ToF
// the xshut pin is active low
void configure_tof_xshut_pins();
void startSensors();
void ReadSensors();
void UpdateMotors();
void SetFlag()
{
  SensorStatus = true;

  if (Driving == false)
  {
      Driving = true;
  }
}

void setup() {
  
  PORTC_PCR5 = PORT_PCR_MUX(0x1);

  // configure portc pin 5 to be an output
  GPIOC_PDDR |= MASK(5);
    

  Serial.begin(9600);
  configure_tof_xshut_pins();
  startSensors();
  GyroMpu.Init();
  BackLeftMotor.SetDutyCycle(0);
  BackRightMotor.SetDutyCycle(0);
  //motorController.Init();

  sensorTimer.begin(SetFlag, HIGH_SPEED_MODE);
}

void loop() {
 
    if (SensorStatus)
    {
      noInterrupts();
      SensorStatus = false;
      interrupts();
      ReadSensors();
      motorController.Update();
      UpdateMotors();
    }

    GPIOC_PDOR ^= MASK(5);
   // Serial.println("\n blink \n");
}

void ReadSensors()
{
  FrontSensor.Update();
  LeftSensor.Update();
  RightSensor.Update();
  GyroMpu.Update();
}

void UpdateMotors()
{
  BackRightMotor.Update();
  BackLeftMotor.Update();
}

void startSensors()
{
  configure_tof_xshut_pins();
  Wire.setSCL(SCL0);
  Wire.setSDA(SDA0);
  Wire.begin();
  
  GPIOC_PDOR |= MASK(FRONT_XSHUT_PIN_MASK);
  FrontSensor.Init();
  
  GPIOC_PDOR |= MASK(LEFT_XSHUT_PIN_MASK);
  LeftSensor.Init();
  
  GPIOD_PDOR |= MASK(RIGHT_XSHUT_PIN_MASK);
  RightSensor.Init();
}

void configure_tof_xshut_pins()
{
  // set port c pin 2 (23) to an I/O pin
  PORTC_PCR2 = PORT_PCR_MUX(0x1);
  PORTC_PCR1 = PORT_PCR_MUX(0x1);
  PORTD_PCR6 = PORT_PCR_MUX(0x1);
  // set port c pin 2 as output
  GPIOC_PDDR |= MASK(FRONT_XSHUT_PIN_MASK);
  GPIOC_PDDR |= MASK(LEFT_XSHUT_PIN_MASK);
  GPIOD_PDDR |= MASK(RIGHT_XSHUT_PIN_MASK);
  // set to low "AGAIN THE XSHUTDOWN PIN IS ACTIVE LOW"
  GPIOC_PDOR |= ~(MASK(FRONT_XSHUT_PIN_MASK) | MASK(LEFT_XSHUT_PIN_MASK));
  GPIOD_PCOR |= ~MASK(RIGHT_XSHUT_PIN_MASK);
}
