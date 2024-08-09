#include "../inc/motor_control.h"

float radio = 0.0315;
uint32_t slots = 20; // ranuras de las ruedas
//			 PID_n, PID_n_1, Kp, Ki, Kd, error_n, error_n_1, error_n_2
PID pid_RR;
PID pid_FR;
PID pid_FL;
PID pid_RL;
float Ts = 0.1; //100 ms

/**
 * PWM settings for motors
*/
void PWM_config(){
	ledc_channel_config_t ledc_channel[4];
	
	ledc_timer_config_t ledc_timer = {
	.speed_mode = LEDC_HIGH_SPEED_MODE,
	.duty_resolution = LEDC_TIMER_7_BIT,
	.timer_num = LEDC_TIMER_0,
	.freq_hz = 20000,
	.clk_cfg = LEDC_AUTO_CLK
	};

	gpio_config_t gpio_struct;
	gpio_struct.intr_type = GPIO_INTR_DISABLE;
	gpio_struct.mode = GPIO_MODE_OUTPUT;
	gpio_struct.pin_bit_mask = ((1ULL << GPIO_IN1_DM2) | (1ULL << GPIO_IN2_DM2) | (1ULL << GPIO_IN1_DM1) | (1ULL << GPIO_IN2_DM1));
	gpio_struct.pull_down_en = 0;
	gpio_struct.pull_up_en = 0;
	gpio_config(&gpio_struct);

	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	ledc_channel[0].gpio_num = GPIO_ENA_M1;
	ledc_channel[0].channel = CHANNEL_RR; 

	ledc_channel[1].gpio_num = GPIO_ENA_M2;
	ledc_channel[1].channel = CHANNEL_FR; 

	ledc_channel[2].gpio_num = GPIO_ENA_M3;
	ledc_channel[2].channel = CHANNEL_FL; 

	ledc_channel[3].gpio_num = GPIO_ENA_M4;
	ledc_channel[3].channel = CHANNEL_RL;

	for(int i = 0; i < 4; i++){
		ledc_channel[i].speed_mode = LEDC_HIGH_SPEED_MODE;
		ledc_channel[i].intr_type = LEDC_INTR_DISABLE;
		ledc_channel[i].timer_sel = LEDC_TIMER_0;
		ledc_channel[i].duty = 0;
		ledc_channel[i].hpoint = 0;
		ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
	}
}

void PID_Init(){
	pid_FL.PID_n = 0;
	pid_FL.PID_n_1 = 0;
	pid_FL.Kp = 100; //30
	pid_FL.Ki = 60;  //5
	pid_FL.Kd = 1;  //0
	pid_FL.error_n = 0;
	pid_FL.error_n_1 = 0;
	pid_FL.error_n_2 = 0;

	pid_FR.PID_n = 0;
	pid_FR.PID_n_1 = 0;
	pid_FR.Kp = 100; //180
	pid_FR.Ki = 60; //100
	pid_FR.Kd = 1;   //5
	pid_FR.error_n = 0;
	pid_FR.error_n_1 = 0;
	pid_FR.error_n_2 = 0;

	pid_RR.PID_n = 0;
	pid_RR.PID_n_1 = 0;
	pid_RR.Kp = 100; //80
	pid_RR.Ki = 60;  //4
	pid_RR.Kd = 1;  //2
	pid_RR.error_n = 0;
	pid_RR.error_n_1 = 0;
	pid_RR.error_n_2 = 0;

	pid_RL.PID_n = 0;
	pid_RL.PID_n_1 = 0;
	pid_RL.Kp = 100; //50
	pid_RL.Ki = 60; //10
	pid_RL.Kd = 1;
	pid_RL.error_n = 0;
	pid_RL.error_n_1 = 0;
	pid_RL.error_n_2 = 0;
}

/**
 * Turns the motors forwards.
 * calculo del dutty -> 2**(duty_resolution)*dutty_percentage%. Ej: 2**(7)*50% dutty 50%
*/
void motor_forward(ledc_channel_t channel, uint32_t dutty_percentage){
	uint32_t dutty;
	if(channel == CHANNEL_RR || channel == CHANNEL_FR){
		gpio_set_level(GPIO_IN1_DM1, 1);
		gpio_set_level(GPIO_IN2_DM1, 0);
	}else{
		gpio_set_level(GPIO_IN1_DM2, 1);
		gpio_set_level(GPIO_IN2_DM2, 0);	
	}
	dutty = set_dutty(dutty_percentage, channel);
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, dutty));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel));
	
}

/**
 * Turns the motors backwards.
*/
void motor_backward(ledc_channel_t channel, uint32_t dutty_percentage){
	uint32_t dutty;
	if(channel == CHANNEL_RR || channel == CHANNEL_FR){
		gpio_set_level(GPIO_IN1_DM1, 0);
		gpio_set_level(GPIO_IN2_DM1, 1);
	}else{
		gpio_set_level(GPIO_IN1_DM2, 0);
		gpio_set_level(GPIO_IN2_DM2, 1);	
	}
	dutty = set_dutty(dutty_percentage, channel);
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, dutty));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel));
}

uint32_t set_dutty(uint32_t dutty_percentage, ledc_channel_t channel){
	if(channel == CHANNEL_RR || channel == CHANNEL_FR){
		return (uint32_t) pow(2, LEDC_TIMER_7_BIT) * (100 - dutty_percentage)/100;
	}else{
		return (uint32_t) pow(2, LEDC_TIMER_7_BIT) * dutty_percentage/100;
	}
}

/**
 * 
*/
void set_velocity(ledc_channel_t channel, float velocity){
	switch (channel){
		case CHANNEL_RR:
			calculate_PID(channel, velocity, &pid_RR);
			if(velocity < 0){ 
				motor_backward(channel, pid_RR.PID_n);
			}else{
				motor_forward(channel, pid_RR.PID_n);
			}
			break;
		case CHANNEL_FR:
			calculate_PID(channel, velocity, &pid_FR);
			if(velocity < 0){ 
				motor_backward(channel, pid_FR.PID_n);
			}else{
				motor_forward(channel, pid_FR.PID_n);
			}
			break;
		case CHANNEL_FL:
			calculate_PID(channel, velocity, &pid_FL);
			if(velocity < 0){ 
				motor_backward(channel, pid_FL.PID_n);
			}else{
				motor_forward(channel, pid_FL.PID_n);
			}
			break;
		case CHANNEL_RL:
			calculate_PID(channel, velocity, &pid_RL);
			if(velocity < 0){ 
				motor_backward(channel, pid_RL.PID_n);
			}else{
				motor_forward(channel, pid_RL.PID_n);
			}
			break;
		default:
			break;
	}
}

void calculate_PID(ledc_channel_t channel, float velocity, PID *pid){
	pid->error_n = velocity - calculate_current_velocity(channel);
	pid->PID_n = pid->PID_n_1 + (pid->Kp+(pid->Kd/Ts))*pid->error_n + (-pid->Kp+pid->Ki*Ts-(2*pid->Kd)/Ts)*pid->error_n_1 + (pid->Kd/Ts)*pid->error_n_2;
	if(pid->PID_n < 0){
		pid->PID_n = 0;
	}else if(pid->PID_n > 100){
		pid->PID_n = 100;
	}
	printf("PID: %f\n",pid->PID_n);
	pid->PID_n_1 = pid->PID_n;
	pid->error_n_2 = pid->error_n_1;
	pid->error_n_1 = pid->error_n;
}

float calculate_current_velocity(ledc_channel_t channel){
	if(channel == CHANNEL_RR){
		current_velocity_RR = (2*M_PI*radio*encoder_pulses_RR)/(slots*Ts);
		printf("velocidad RR: %f\n",current_velocity_RR);
		//printf("Pulsos RR: %d\n",encoder_pulses_RR);
		encoder_pulses_RR = 0;
		return current_velocity_RR;
	}else if(channel == CHANNEL_FR){
		current_velocity_FR = (2*M_PI*radio*encoder_pulses_FR)/(slots*Ts);
		printf("velocidad FR: %f\n",current_velocity_FR);
		//printf("Pulsos FR: %d\n",encoder_pulses_FR);
		encoder_pulses_FR = 0;
		return current_velocity_FR;
	}else if(channel == CHANNEL_FL){
		current_velocity_FL = (2*M_PI*radio*encoder_pulses_FL)/(slots*Ts);
		printf("velocidad FL: %f\n",current_velocity_FL);
		//printf("Pulsos FL: %d\n",encoder_pulses_FL);
		encoder_pulses_FL = 0;
		return current_velocity_FL;
	}else{ // CHANNEL_RL
		current_velocity_RL = (2*M_PI*radio*encoder_pulses_RL)/(slots*Ts);
		printf("velocidad RL: %f\n",current_velocity_RL);
		//printf("Pulsos RL: %d\n",encoder_pulses_RL);
		encoder_pulses_RL = 0;
		return current_velocity_RL;
	}
}