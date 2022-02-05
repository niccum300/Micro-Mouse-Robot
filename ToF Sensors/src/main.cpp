// Authors: Brayton Niccum


#include <Arduino.h>
#include <IntervalTimer.h>

#include <global.h>
#include <SENSOR_DATA.h>
#include <TOF.h>


// ToF

#define FRONT_XSHUT_PIN_MASK (2)
#define FRONT_SENSOR_ADDRESS (0x10)

#define LEFT_XSHUT_PIN_MASK (1)
#define LEFT_SENSOR_ADDRESS (0x11)

#define RIGHT_XSHUT_PIN_MASK (6)
#define RIGHT_SENSOR_ADDRESS (0x12)

TOF FrontSensor(&PORTC_PCR2, &GPIOC_PDDR, &GPIOC_PDOR, MASK(FRONT_XSHUT_PIN_MASK), FRONT_SENSOR_ADDRESS, FRONT);
TOF LeftSensor(&PORTC_PCR1, &GPIOC_PDDR, &GPIOC_PDOR, MASK(LEFT_XSHUT_PIN_MASK), LEFT_SENSOR_ADDRESS, LEFT);
TOF RightSensor(&PORTD_PCR6, &GPIOD_PDDR, &GPIOD_PCOR, MASK(RIGHT_XSHUT_PIN_MASK), RIGHT_SENSOR_ADDRESS, RIGHT);
SENSOR_DATA sensor_data[3];

IntervalTimer sensorTimer;
bool SensorStatus = false;

// set correct I2C bus and enable it 
// void configure_I2C(){
//   Wire.setSCL(SCL0);
//   Wire.setSDA(SDA0);
//   Wire.begin();
// }

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

// void CheckDeviation(int sensor)
// {

//   if(sensor_data[sensor].average == 0.00)
//   {
//     sensor_data[sensor].average = sensor_data[sensor].new_data_point;
//     sensor_data[sensor].total_data_points = 1;
//   }
//   // if its outside devation reset sensor in terms of average
//    else if(abs(((sensor_data[sensor].new_data_point - sensor_data[sensor].average) / sensor_data[sensor].average)) > SENSOR_DEVIATION)
//    {
//      sensor_data[sensor].average = sensor_data[sensor].new_data_point;
//      sensor_data[sensor].total_data_points = 1;
//    }
//    else{
//      sensor_data[sensor].total_data_points += 1;
//      sensor_data[sensor].average = sensor_data[sensor].average + 
//         ((sensor_data[sensor].new_data_point - sensor_data[sensor].average)/sensor_data[sensor].total_data_points);
//    }
// }

void ReadSensors()
{
  FrontSensor.Update();
  sensor_data[FRONT] = FrontSensor.GetData();

  LeftSensor.Update();
  sensor_data[LEFT] = LeftSensor.GetData();
 
  RightSensor.Update();
  sensor_data[RIGHT] = RightSensor.GetData();

  
  
  Serial.printf("FRONT: %f LEFT: %f RIGHT: %f \n", sensor_data[FRONT].average, sensor_data[LEFT].average, sensor_data[RIGHT].average);
  Serial.printf("Data Points %d \n", sensor_data[FRONT].total_data_points);
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


  GPIOC_PDOR |= MASK(FRONT_XSHUT_PIN_MASK);
  FrontSensor.Init();
  
  GPIOC_PDOR |= MASK(LEFT_XSHUT_PIN_MASK);
  LeftSensor.Init();
  
  GPIOD_PDOR |= MASK(RIGHT_XSHUT_PIN_MASK);
  RightSensor.Init();

  
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