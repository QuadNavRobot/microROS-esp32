#include "../inc/motor_control.h"

/**
 * PWM settings for motors
*/
void PWM_config(){
	ledc_channel_config_t ledc_channel[4];
	
	ledc_timer_config_t ledc_timer = {
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.duty_resolution = LEDC_TIMER_7_BIT,
	.timer_num = LEDC_TIMER_0,
	.freq_hz = 100000,
	.clk_cfg = LEDC_AUTO_CLK
	};

	gpio_set_direction(GPIO_IN1_DM1, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_IN2_DM1, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_IN1_DM2, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_IN2_DM2, GPIO_MODE_OUTPUT);

	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	ledc_channel[0].gpio_num = GPIO_ENA_M1;
	ledc_channel[0].channel = LEDC_CHANNEL_0;

	ledc_channel[1].gpio_num = GPIO_ENA_M2;
	ledc_channel[1].channel = LEDC_CHANNEL_1;

	ledc_channel[2].gpio_num = GPIO_ENA_M3;
	ledc_channel[2].channel = LEDC_CHANNEL_2;

	ledc_channel[3].gpio_num = GPIO_ENA_M4;
	ledc_channel[3].channel = LEDC_CHANNEL_3;

	for(int i = 0; i < 4; i++){
		ledc_channel[i].speed_mode = LEDC_LOW_SPEED_MODE;
		ledc_channel[i].intr_type = LEDC_INTR_DISABLE;
		ledc_channel[i].timer_sel = LEDC_TIMER_0;
		ledc_channel[i].duty = 0;
		ledc_channel[i].hpoint = 0;
		ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
	}
}

/**
 * Turns the motors forwards.
 * calculo del dutty -> 2**(duty_resolution)*dutty_percentage%. Ej: 2**(7)*50% dutty 50%
*/
void motor_forward(int driver_motor, ledc_channel_t channel, uint32_t dutty){

	if(driver_motor == 1){
		gpio_set_level(GPIO_IN1_DM1, 1);
		gpio_set_level(GPIO_IN2_DM1, 0);
	}else{
		gpio_set_level(GPIO_IN1_DM2, 1);
		gpio_set_level(GPIO_IN2_DM2, 0);	
	}

	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, dutty));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
	
}

/**
 * Turns the motors backwards.
*/
void motor_backward(int driver_motor, ledc_channel_t channel, uint32_t dutty){
	if(driver_motor == 1){
		gpio_set_level(GPIO_IN1_DM1, 0);
		gpio_set_level(GPIO_IN2_DM1, 1);
	}else{
		gpio_set_level(GPIO_IN1_DM2, 0);
		gpio_set_level(GPIO_IN2_DM2, 1);	
	}

	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, dutty));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}


