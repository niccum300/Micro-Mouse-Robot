// Authors: Brayton Niccum
#ifndef GLOBAL_h
#define GLOBAL_h

#include <Arduino.h>

#define MASK(x) (1UL << (x))
#define LED_SHIFT_MASK (5)
#define IO_MUX_MASK (1)

#define PWM_RESOULTION_32_BIT (65535)
//#define PWM_RESOULTION_32_BIT (255)

enum SENSOR_LOCATION {LEFT = 0, FRONT = 1, RIGHT = 2};



// error constant for sensor

#endif