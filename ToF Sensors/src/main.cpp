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
#define BACK_RIGHT_MOTOR_PIN (35)
#define BACK_LEFT_MOTOR_PIN (37)
// 
#define FRONT_XSHUT_PIN (23)
#define FRONT_SENSOR_ADDRESS (0x10)

#define LEFT_XSHUT_PIN (22)
#define LEFT_SENSOR_ADDRESS (0x11)

#define RIGHT_XSHUT_PIN (21)
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

TOF FrontSensor(&PORTC_PCR2, &GPIOC_PDDR, &GPIOC_PDOR, FRONT_XSHUT_PIN, FRONT_SENSOR_ADDRESS, &FrontSensorQ);
TOF LeftSensor(&PORTC_PCR1, &GPIOC_PDDR, &GPIOC_PDOR, LEFT_XSHUT_PIN, LEFT_SENSOR_ADDRESS, &LeftSensorQ);
TOF RightSensor(&PORTD_PCR6, &GPIOD_PDDR, &GPIOD_PCOR, RIGHT_XSHUT_PIN, RIGHT_SENSOR_ADDRESS, &RightSensorQ);

Motor BackRightMotor(BACK_RIGHT_MOTOR_PIN, PWM_RESOULTION_32_BIT, &BackRightMotorQ);
Motor BackLeftMotor(BACK_LEFT_MOTOR_PIN, PWM_RESOULTION_32_BIT, &BackLeftMotorQ);

Gyro GyroMpu(Wire, &GyroQ);

MotorController motorController;

IntervalTimer sensorTimer;

int LeftEncoderCount = 0;
int RightEncoderCount = 0;

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

void LeftEncoder()
{
  LeftEncoderCount++;
  //Serial.printf("\nLeft ENCODER: %d\n", LeftEncoderCount);
}

void RightEncoder()
{
  RightEncoderCount++;
}

void setup() {
  PORTC_PCR5 = PORT_PCR_MUX(0x1);

  // configure portc pin 5 to be an output
  GPIOC_PDDR |= MASK(5);

  attachInterrupt(digitalPinToInterrupt(31), LeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(32), RightEncoder, RISING);


  Serial.begin(9600);
  configure_tof_xshut_pins();
  startSensors();
  GyroMpu.Init();
  BackLeftMotor.SetDutyCycle(0);
  BackRightMotor.SetDutyCycle(0);
  motorController.Init();

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

  Serial.print("test");
  
  digitalWrite(FRONT_XSHUT_PIN, HIGH);
  FrontSensor.Init();
  Serial.print("test0");
  
  digitalWrite(LEFT_XSHUT_PIN, HIGH);
  LeftSensor.Init();
  Serial.print("test1");

  digitalWrite(RIGHT_XSHUT_PIN, HIGH);
  RightSensor.Init();
  Serial.print("test2");
}

void configure_tof_xshut_pins()
{
  pinMode(FRONT_XSHUT_PIN, OUTPUT);
  pinMode(LEFT_XSHUT_PIN, OUTPUT);
  pinMode(RIGHT_XSHUT_PIN, OUTPUT);

  digitalWrite(FRONT_XSHUT_PIN, LOW);
  digitalWrite(LEFT_XSHUT_PIN, LOW);
  digitalWrite(RIGHT_XSHUT_PIN, LOW);
}
