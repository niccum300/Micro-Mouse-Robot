#include <pins_arduino.h>

void SetI2CAddress(uint8_t address){
  I2C0_A1 |= address;
}
