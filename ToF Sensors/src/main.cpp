#include <Arduino.h>
#include <VL53L0X.h>

#define SCL0 (19)
#define SDA0 (20)

#define HIGH_ACCURACY_MODE (200000)
#define DEFAULT_MODE (33000)
#define HIGH_SPEED_MODE (20000) 


VL53L0X sensor;

// set correct I2C bus and enable it 
void configure_I2C(){
  Wire.setSCL(SCL0);
  Wire.setSDA(SDA0);
  Wire.begin();
}

void setup() {
  Serial.begin(9600);
  configure_I2C();

  if (!sensor.init())
  {
    Serial.println("Failed to detect and initalze sensor!");
    while (1){}
  }

  // This sets the timing budget to determine how long a reading will take and how accurate
  sensor.setMeasurementTimingBudget(HIGH_ACCURACY_MODE);

}

void loop() {
  Serial.println(sensor.readRangeSingleMillimeters()/25.4001);

  delay(HIGH_ACCURACY_MODE/1000);
}