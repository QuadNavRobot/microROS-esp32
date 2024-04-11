#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"

#define GPIO_IN1_DM1 2
#define GPIO_IN2_DM1 15
#define GPIO_IN1_DM2 27
#define GPIO_IN2_DM2 13

#define GPIO_ENA_M1 4 //PWM
#define GPIO_ENA_M2 16
#define GPIO_ENA_M3 25
#define GPIO_ENA_M4 26

void PWM_config();
void motor_forward(ledc_channel_t channel, uint32_t dutty);
void motor_backward(ledc_channel_t channel, uint32_t dutty);
