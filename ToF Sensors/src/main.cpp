// Authors: Brayton Niccum


#include <Arduino.h>
#include <global.h>
#include <VL53L0X.h>
#include <IntervalTimer.h>


// ToF

#define NORTH_XSHUT_PIN_MASK (2)
#define NORTH_SENSOR_ADDRESS (0x10)

#define EAST_XSHUT_PIN_MASK (1)
#define EAST_SENSOR_ADDRESS (0x11)

#define WEST_XSHUT_PIN_MASK (6)
#define WEST_SENSOR_ADDRESS (0x12)

#define SCL0 (19)
#define SDA0 (20)

#define HIGH_ACCURACY_MODE (200000) //value in microseconds
#define DEFAULT_MODE (33000)        //value in microseconds
#define HIGH_SPEED_MODE (20000)     //value in microseconds
// End ToF

VL53L0X NorthSensor;
VL53L0X EastSensor;
VL53L0X WestSensor;

IntervalTimer sensorTimer;
bool SensorStatus = false;

// set correct I2C bus and enable it 
void configure_I2C(){
  Wire.setSCL(SCL0);
  Wire.setSDA(SDA0);
  Wire.begin();
}

// configure pins to output for shutting down each ToF
// the xshut pin is active low
void configure_tof_xshut_pins()
{
  // set port c pin 2 (23) to an I/O pin
  PORTC_PCR2 = PORT_PCR_MUX(0x1);
  PORTC_PCR1 = PORT_PCR_MUX(0x1);
  PORTD_PCR6 = PORT_PCR_MUX(0x1);
  // set port c pin 2 as output
  GPIOC_PDDR |= MASK(NORTH_XSHUT_PIN_MASK);
  GPIOC_PDDR |= MASK(EAST_XSHUT_PIN_MASK);
  GPIOD_PDDR |= MASK(WEST_XSHUT_PIN_MASK);
  // set to low "AGAIN THE XSHUTDOWN PIN IS ACTIVE LOW"
  GPIOC_PDOR |= ~(MASK(NORTH_XSHUT_PIN_MASK) | MASK(EAST_XSHUT_PIN_MASK));
  GPIOD_PCOR |= ~MASK(WEST_XSHUT_PIN_MASK);
}

void SetFlag()
{
  SensorStatus = true;
}



void ReadSensors()
{
  float north_reading = NorthSensor.readRangeSingleMillimeters()/25.4001;
  float east_reading = EastSensor.readRangeSingleMillimeters()/25.4001;
  float west_reading = WestSensor.readRangeSingleMillimeters()/25.4001;
  
  Serial.printf("North: %f East: %f West: %f \n", north_reading, east_reading, west_reading);
}

void setup() {
  PORTC_PCR5 = PORT_PCR_MUX(0x1);

  // configure portc pin 5 to be an output
  GPIOC_PDDR |= MASK(5);
  Serial.begin(9600);
  configure_tof_xshut_pins();
  configure_I2C();


  // Turn on first sensor 
  GPIOC_PDOR |= MASK(NORTH_XSHUT_PIN_MASK);
  if (!NorthSensor.init())
  {
    Serial.println("Failed to detect and initalze north sensor!");
    while (1){}
  }
  // set address custom I2C address for first sensor
   NorthSensor.setAddress(NORTH_SENSOR_ADDRESS);
   Serial.println("North sensor configured");

  GPIOC_PDOR |= MASK(EAST_XSHUT_PIN_MASK);
  if (!EastSensor.init())
  {
    Serial.println("Failed to detect and initalze east sensor!");
    while (1){}
  }

  EastSensor.setAddress(EAST_SENSOR_ADDRESS);
  Serial.println("East sensor configured");

  GPIOD_PDOR |= MASK(WEST_XSHUT_PIN_MASK);
  if (!WestSensor.init())
  {
    Serial.println("Failed to detect and initalze west sensor!");
    while (1){}
  }

  WestSensor.setAddress(WEST_SENSOR_ADDRESS);
  Serial.println("West sensor configured");


  // This sets the timing budget to determine how long a reading will take and how accurate
  NorthSensor.setMeasurementTimingBudget(DEFAULT_MODE);
  EastSensor.setMeasurementTimingBudget(DEFAULT_MODE);
  WestSensor.setMeasurementTimingBudget(DEFAULT_MODE);
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
    Serial.println("\n blink \n");
}