#include <Arduino.h>
#include "I2C.h"

#define LED_SHIFT (5)
#define MASK(x) (1UL << (x))


void setup() {

  // confiugre port c pin 5 to be digital output pin
  PORTC_PCR5 = PORT_PCR_MUX(0x1);

  // configure portc pin 5 to be an output
  GPIOC_PDDR |= MASK(LED_SHIFT);
}

void loop() {
  // toggle LED
  GPIOC_PDOR ^= MASK(LED_SHIFT);

  // used delay from arduino for now 
  delay(100);                  
}

