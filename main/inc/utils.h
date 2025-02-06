#include <math.h>
#include "freertos/FreeRTOS.h"

typedef struct{
    float w, x, y, z;
  } Quaternion;

typedef struct{
    float roll, pitch, yaw;
  } EulerAngle; 

float convertGToMS2(float value);
float convertDegreesToRadians(float value);
float angle_wrap(float degrees);
float angle_wrap_radians(float radians);
Quaternion euler_to_quaternion(EulerAngle e);