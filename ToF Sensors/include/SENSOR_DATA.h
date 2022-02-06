#ifndef SENSOR_DATA_h
#define SENSOR_DATA_h

// strcuture for sensor error calcualtions 
struct SENSOR_DATA{
  float average = 0.0000;
  unsigned int total_data_points = 0;
  float new_data_point = 0.0000;
};

enum SENSOR_LOCATION {LEFT = 0, RIGHT = 1, FRONT = 2};

struct SENSOR_DATA_BUNDLE{
    SENSOR_DATA front;
    SENSOR_DATA left;
    SENSOR_DATA right;
};

#endif