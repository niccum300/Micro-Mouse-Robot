// Authors: Brayton Niccum
#ifndef GLOBAL_h
#define GLOBAL_h

#include <Arduino.h>
#include <SENSOR_DATA.h>
#include <SensorQueue.h>

//static SENSOR_DATA_BUNDLE sensor_bundle;

static SensorQueue FrontRightMotorQ;
static SensorQueue FrontLeftMotorQ;
static SensorQueue BackRightMotorQ;
static SensorQueue BackLeftMotorQ;

#define MASK(x) (1UL << (x))
#define LED_SHIFT_MASK (5)
#define IO_MUX_MASK (1)



// error constant for sensor

#endif