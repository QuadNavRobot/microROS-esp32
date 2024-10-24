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
 * @param value 
 * @return float 
 */
float convertDegreesToRadians(float value) { 
  // 1° = pi/180 rad
    return value * M_PI /180;
}

float angle_wrap(float degrees){
	if (degrees > 180)
	{
		return degrees - 360;
	}else if (degrees < -180)
	{
		return degrees + 360;
	}
	
	return degrees;
}