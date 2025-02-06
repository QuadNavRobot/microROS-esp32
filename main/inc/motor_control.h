#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "math.h"
#include "sensors.h"

#define GPIO_IN1_DM1 2
#define GPIO_IN2_DM1 15
#define GPIO_IN1_DM2 27
#define GPIO_IN2_DM2 13

#define GPIO_ENA_M1 4 //PWM
#define GPIO_ENA_M2 16
#define GPIO_ENA_M3 25
#define GPIO_ENA_M4 26

#define CHANNEL_RR (LEDC_CHANNEL_0)
#define CHANNEL_FR (LEDC_CHANNEL_1)
#define CHANNEL_FL (LEDC_CHANNEL_2)
#define CHANNEL_RL (LEDC_CHANNEL_3)

#define RADIUS_WHEEL 0.0325
#define WHEEL_SEPARATION 0.1 // 10 cm
#define SLOTS_ENCODER 20 // Ranuras de cada encoder
#define SAMPLING_TIME 0.1 // In seconds

typedef struct{
	float PID_n;
	float Kp;
	float Ki;
	float Kd;
	float error_n;
	float error_n_1;
	float P_term;
	float I_term;
	float D_term;
} PID;

typedef struct{
	__uint8_t w_FL, w_FR, w_RR, w_RL;
}DirectionOfRotation;

typedef struct{
  float w_l, w_r;
}AngularVelocityWheels;

typedef struct{
  float w_FL, w_FR, w_RR, w_RL;
}CurrentAngularVelocityWheels;

extern CurrentAngularVelocityWheels current_angular_velocity_wheels;
extern float linear_velocity_x;
extern float angular_velocity_z;

void PID_Init();
void PWM_config();
void motor_forward(ledc_channel_t channel, uint32_t dutty_percentage);
void motor_backward(ledc_channel_t channel, uint32_t dutty_percentage);
uint32_t set_dutty(uint32_t dutty_percentage, ledc_channel_t channel);
void set_angular_velocity(AngularVelocityWheels angular_velocity_wheels);
void calculate_PID(float sensed_value, float set_point, PID *pid);
void calculate_current_angular_velocity();