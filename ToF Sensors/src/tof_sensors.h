// Authors: Brayton Niccum
// Api for controlling all ToF Sensors as one unit.

#include <VL53L0X.h>

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
