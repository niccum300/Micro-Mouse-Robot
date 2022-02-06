// Authors: Brayton Niccum
#include <Arduino.h>
#include <IntervalTimer.h>

#include <global.h>
#include <SENSOR_DATA.h>
#include <TOF.h>

#define FRONT_XSHUT_PIN_MASK (0x2)
#define FRONT_SENSOR_ADDRESS (0x10)

#define LEFT_XSHUT_PIN_MASK (0x1)
#define LEFT_SENSOR_ADDRESS (0x11)

#define RIGHT_XSHUT_PIN_MASK (0x6)
#define RIGHT_SENSOR_ADDRESS (0x12)

TOF FrontSensor(&PORTC_PCR2, &GPIOC_PDDR, &GPIOC_PDOR, MASK(FRONT_XSHUT_PIN_MASK), FRONT_SENSOR_ADDRESS, FRONT);
TOF LeftSensor(&PORTC_PCR1, &GPIOC_PDDR, &GPIOC_PDOR, MASK(LEFT_XSHUT_PIN_MASK), LEFT_SENSOR_ADDRESS, LEFT);
TOF RightSensor(&PORTD_PCR6, &GPIOD_PDDR, &GPIOD_PCOR, MASK(RIGHT_XSHUT_PIN_MASK), RIGHT_SENSOR_ADDRESS, RIGHT);
SENSOR_DATA sensor_data[3];

IntervalTimer sensorTimer;
bool SensorStatus = false;

// configure pins to output for shutting down each ToF
// the xshut pin is active low
void configure_tof_xshut_pins();

void startSensors();

void ReadSensors();

void SetFlag()
{
  SensorStatus = true;
}

void setup() {
  PORTC_PCR5 = PORT_PCR_MUX(0x1);

  // configure portc pin 5 to be an output
  GPIOC_PDDR |= MASK(5);

  Serial.begin(9600);
  configure_tof_xshut_pins();
  
  Wire.setSCL(SCL0);
  Wire.setSDA(SDA0);
  Wire.begin();
  
  startSensors();

  sensorTimer.begin(SetFlag, DEFAULT_MODE);
}

void loop() {
 
    if (SensorStatus)
    {
      noInterrupts();
      SensorStatus = false;
      interrupts();
      ReadSensors();
    }
 
    GPIOC_PDOR ^= MASK(5);
    //Serial.println("\n blink \n");
}

void ReadSensors()
{
  sensor_data[FRONT] = FrontSensor.GetData();
  sensor_data[LEFT] = LeftSensor.GetData();
  sensor_data[RIGHT] = RightSensor.GetData();
  
  Serial.printf("FRONT: %f LEFT: %f RIGHT: %f \n", sensor_data[FRONT].average, sensor_data[LEFT].average, sensor_data[RIGHT].average);
  Serial.printf("Data Points %d", sensor_data[FRONT].total_data_points);
}

void startSensors()
{
  configure_tof_xshut_pins();
  
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