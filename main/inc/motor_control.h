#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
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

extern float current_velocity_FL;
extern float current_velocity_FR;
extern float current_velocity_RR;
extern float current_velocity_RL;

typedef struct{
	float PID_n;
	float PID_n_1;
	float Kp;
	float Ki;
	float Kd;
	float error_n;
	float error_n_1;
	float error_n_2;
} PID;

void PID_Init();
void PWM_config();
void motor_forward(ledc_channel_t channel, uint32_t dutty_percentage);
void motor_backward(ledc_channel_t channel, uint32_t dutty_percentage);
uint32_t set_dutty(uint32_t dutty_percentage, ledc_channel_t channel);
void set_velocity(ledc_channel_t channel, float velocity);
void calculate_PID(ledc_channel_t channel, float velocity, PID *pid);
float calculate_current_velocity(ledc_channel_t channel);