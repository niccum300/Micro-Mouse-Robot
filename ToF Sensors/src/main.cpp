// Authors: Brayton Niccum


#include <Arduino.h>
#include <global.h>
#include <VL53L0X.h>
#include <IntervalTimer.h>


// ToF

#define FRONT_XSHUT_PIN_MASK (2)
#define FRONT_SENSOR_ADDRESS (0x10)

#define LEFT_XSHUT_PIN_MASK (1)
#define LEFT_SENSOR_ADDRESS (0x11)

#define RIGHT_XSHUT_PIN_MASK (6)
#define RIGHT_SENSOR_ADDRESS (0x12)

#define SCL0 (19)
#define SDA0 (20)

#define HIGH_ACCURACY_MODE (200000) //value in microseconds
#define DEFAULT_MODE (33000)        //value in microseconds
#define HIGH_SPEED_MODE (20000)     //value in microseconds
// End ToF

// strcuture for sensor error calcualtions 
struct SENSOR_DATA{
  float average = 0.0000;
  unsigned int total_data_points = 0;
  float new_data_point = 0.0000;
};

enum SENSOR_LOCATION {LEFT = 0, RIGHT = 1, FRONT = 2};

// error constant for sensor
float SENSOR_DEVIATION = 0.07;

VL53L0X FrontSensor;
VL53L0X LeftSensor;
VL53L0X RightSensor;
SENSOR_DATA sensor_data[3];

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
  GPIOC_PDDR |= MASK(FRONT_XSHUT_PIN_MASK);
  GPIOC_PDDR |= MASK(LEFT_XSHUT_PIN_MASK);
  GPIOD_PDDR |= MASK(RIGHT_XSHUT_PIN_MASK);
  // set to low "AGAIN THE XSHUTDOWN PIN IS ACTIVE LOW"
  GPIOC_PDOR |= ~(MASK(FRONT_XSHUT_PIN_MASK) | MASK(LEFT_XSHUT_PIN_MASK));
  GPIOD_PCOR |= ~MASK(RIGHT_XSHUT_PIN_MASK);
}

void SetFlag()
{
  SensorStatus = true;
}

void CheckDeviation(int sensor)
{

  if(sensor_data[sensor].average == 0.00)
  {
    sensor_data[sensor].average = sensor_data[sensor].new_data_point;
    sensor_data[sensor].total_data_points = 1;
  }
  // if its outside devation reset sensor in terms of average
   else if(abs(((sensor_data[sensor].new_data_point - sensor_data[sensor].average) / sensor_data[sensor].average)) > SENSOR_DEVIATION)
   {
     sensor_data[sensor].average = sensor_data[sensor].new_data_point;
     sensor_data[sensor].total_data_points = 1;
   }
   else{
     sensor_data[sensor].total_data_points += 1;
     sensor_data[sensor].average = sensor_data[sensor].average + 
        ((sensor_data[sensor].new_data_point - sensor_data[sensor].average)/sensor_data[sensor].total_data_points);
   }
}

void ReadSensors()
{
  sensor_data[FRONT].new_data_point = FrontSensor.readRangeSingleMillimeters()/25.4001;
  CheckDeviation(FRONT);
  sensor_data[LEFT].new_data_point = LeftSensor.readRangeSingleMillimeters()/25.4001;
  CheckDeviation(LEFT);
  sensor_data[RIGHT].new_data_point = RightSensor.readRangeSingleMillimeters()/25.4001;
  CheckDeviation(RIGHT);
  
  Serial.printf("FRONT: %f LEFT: %f RIGHT: %f \n", sensor_data[FRONT].average, sensor_data[LEFT].average, sensor_data[RIGHT].average);
  //Serial.printf("Data Points %d \n", sensor_data[FRONT].total_data_points);
}


void setup() {
  PORTC_PCR5 = PORT_PCR_MUX(0x1);

  // configure portc pin 5 to be an output
  GPIOC_PDDR |= MASK(5);
  Serial.begin(9600);
  configure_tof_xshut_pins();
  configure_I2C();


  // Turn on first sensor 
  GPIOC_PDOR |= MASK(FRONT_XSHUT_PIN_MASK);
  if (!FrontSensor.init())
  {
    Serial.println("Failed to detect and initalze FRONT sensor!");
    while (1){}
  }
  // set address custom I2C address for first sensor
   FrontSensor.setAddress(FRONT_SENSOR_ADDRESS);
   Serial.println("FRONT sensor configured");

  GPIOC_PDOR |= MASK(LEFT_XSHUT_PIN_MASK);
  if (!LeftSensor.init())
  {
    Serial.println("Failed to detect and initalze LEFT sensor!");
    while (1){}
  }

  LeftSensor.setAddress(LEFT_SENSOR_ADDRESS);
  Serial.println("LEFT sensor configured");

  GPIOD_PDOR |= MASK(RIGHT_XSHUT_PIN_MASK);
  if (!RightSensor.init())
  {
    Serial.println("Failed to detect and initalze RIGHT sensor!");
    while (1){}
  }

  RightSensor.setAddress(RIGHT_SENSOR_ADDRESS);
  Serial.println("RIGHT sensor configured");


  // This sets the timing budget to determine how long a reading will take and how accurate
  FrontSensor.setMeasurementTimingBudget(DEFAULT_MODE);
  LeftSensor.setMeasurementTimingBudget(DEFAULT_MODE);
  RightSensor.setMeasurementTimingBudget(DEFAULT_MODE);
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