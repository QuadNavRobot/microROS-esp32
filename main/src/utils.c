#include "../inc/utils.h"

/**
 * @brief Convert the values of g to m/s2
 * @param value value of acceleration in g
 * @return float value of acceleration in m/s
 */
float convertGToMS2(float value) {
    // 1 g = 9.80665 m/s²
    return value * 9.80665;
}

/**
 * @brief Convert the values of °/s to rad/s
 */
float convertDegreesToRadians(float value) { 
  // 1° = pi/180 rad
    return value * M_PI /180;
}

/**
 * @brief Wrap the angle between -180 and 180
 */
float angle_wrap(float degrees){
	if (degrees > 180){
		return degrees - 360;
	}else if (degrees < -180){
		return degrees + 360;
	}
	return degrees;
}

/**
 * @brief Wrap the angle between -pi and pi
 */
float angle_wrap_radians(float radians){
  if (radians > M_PI){
    return radians - 2*M_PI;
  }else if (radians < -M_PI){
    return radians + 2*M_PI;
  }
  return radians;
}

/**
 * @brief Convert euler angles to quaternion
 */
Quaternion euler_to_quaternion(EulerAngle e){
  Quaternion q;
  float cy = cos(e.yaw * 0.5);
  float sy = sin(e.yaw * 0.5);
  float cp = cos(e.pitch * 0.5);
  float sp = sin(e.pitch * 0.5);
  float cr = cos(e.roll * 0.5);
  float sr = sin(e.roll * 0.5);
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

/**
 * @brief Calculate the current time
 */
void get_current_time(uint32_t *seconds, uint32_t *nanoseconds){
	TickType_t ticks = xTaskGetTickCount();
	*seconds = ticks / configTICK_RATE_HZ;
	*nanoseconds = (ticks % configTICK_RATE_HZ) * 1000000000 / configTICK_RATE_HZ;
}