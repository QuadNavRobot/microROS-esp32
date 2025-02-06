#include "../inc/motor_control.h"

PID pid_RR;
PID pid_FR;
PID pid_FL;
PID pid_RL;

float linear_velocity_x = 0;
float angular_velocity_z = 0;
DirectionOfRotation directionOfRotation;
CurrentAngularVelocityWheels current_angular_velocity_wheels;

/**
 * @brief Configures the PWM for the motors
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

/**
 * @brief Initializes the PID controllers and sets the 
 * directions of rotation of the wheels forward
 */
void PID_Init(){
	pid_FL.PID_n = 0;
	pid_FL.Kp = 4;  //2
	pid_FL.Ki = 10;
	pid_FL.Kd = 0.2;
	pid_FL.error_n = 0;
	pid_FL.error_n_1 = 0;
	pid_FL.P_term = 0;
	pid_FL.I_term = 0;
	pid_FL.D_term = 0;

	pid_FR.PID_n = 0;
	pid_FR.Kp = 4;
	pid_FR.Ki = 8;
	pid_FR.Kd = 0.2;
	pid_FR.error_n = 0;
	pid_FR.error_n_1 = 0;
	pid_FR.P_term = 0;
	pid_FR.I_term = 0;
	pid_FR.D_term = 0;

	pid_RR.PID_n = 0;
	pid_RR.Kp = 4;
	pid_RR.Ki = 8;
	pid_RR.Kd = 0.2;
	pid_RR.error_n = 0;
	pid_RR.error_n_1 = 0;
	pid_RR.P_term = 0;
	pid_RR.I_term = 0;
	pid_RR.D_term = 0;

	pid_RL.PID_n = 0;
	pid_RL.Kp = 6;
	pid_RL.Ki = 8;
	pid_RL.Kd = 0.2;
	pid_RL.error_n = 0;
	pid_RL.error_n_1 = 0;
	pid_RL.P_term = 0;
	pid_RL.I_term = 0;
	pid_RL.D_term = 0;

	directionOfRotation.w_FL = 0;
	directionOfRotation.w_FR = 0;
	directionOfRotation.w_RL = 0;
	directionOfRotation.w_RR = 0;
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
	if(channel == CHANNEL_RR || channel == CHANNEL_FR || channel == CHANNEL_FL){
		return (uint32_t) pow(2, LEDC_TIMER_7_BIT) * (100 - dutty_percentage)/100;
	}else{
		return (uint32_t) pow(2, LEDC_TIMER_7_BIT) * dutty_percentage/100;
	}
}

/**
 * @brief Sets the angular velocity of each wheel
 */
void set_angular_velocity(AngularVelocityWheels angular_velocity_wheels){
	calculate_current_angular_velocity();
	calculate_PID(current_angular_velocity_wheels.w_FL, angular_velocity_wheels.w_l, &pid_FL);
	calculate_PID(current_angular_velocity_wheels.w_RL, angular_velocity_wheels.w_l, &pid_RL);
	calculate_PID(current_angular_velocity_wheels.w_FR, angular_velocity_wheels.w_r, &pid_FR);
	calculate_PID(current_angular_velocity_wheels.w_RR, angular_velocity_wheels.w_r, &pid_RR);
	if(angular_velocity_wheels.w_l < 0){	
		motor_backward(CHANNEL_FL, pid_FL.PID_n);
		motor_backward(CHANNEL_RL, pid_RL.PID_n);
		directionOfRotation.w_FL = 1;
		directionOfRotation.w_RL = 1;
	}else{
		motor_forward(CHANNEL_FL, pid_FL.PID_n);
		motor_forward(CHANNEL_RL, pid_RL.PID_n);
		directionOfRotation.w_FL = 0;
		directionOfRotation.w_RL = 0;
	}
	if(angular_velocity_wheels.w_r < 0){
		motor_backward(CHANNEL_FR, pid_FR.PID_n);
		motor_backward(CHANNEL_RR, pid_RR.PID_n);
		directionOfRotation.w_FR = 1;
		directionOfRotation.w_RR = 1;
	}else{
		motor_forward(CHANNEL_FR, pid_FR.PID_n);
		motor_forward(CHANNEL_RR, pid_RR.PID_n);
		directionOfRotation.w_FR = 0;
		directionOfRotation.w_RR = 0;
	}
}

/**
 * @brief Calculate the PID control
 */
void calculate_PID(float sensed_value, float set_point, PID *pid){
	if(set_point < 0){
		set_point = -set_point; // La rueda debe girar hacia atras
	}
	if(sensed_value < 0){
		sensed_value = -sensed_value;
	}
	pid->error_n = set_point - sensed_value;
	pid->P_term = pid->Kp * pid->error_n;
	pid->I_term += pid->Ki * pid->error_n * SAMPLING_TIME;
	pid->D_term = pid->Kd * (pid->error_n - pid->error_n_1) / SAMPLING_TIME;
	pid->PID_n = pid->P_term + pid->I_term + pid->D_term;
	pid->error_n_1 = pid->error_n;
	if(pid->PID_n < 0){
		pid->PID_n = 0;
	}else if(pid->PID_n > 100){
		pid->PID_n = 100;
	}
}

/**
 * @brief Calculates the angular velocity of each wheel and the linear velocity of the robot
 */
void calculate_current_angular_velocity(){
	current_angular_velocity_wheels.w_RR = (directionOfRotation.w_RR == 0) ? ((2*M_PI*encoder_pulses_RR)/(SLOTS_ENCODER*SAMPLING_TIME)) : (-(2*M_PI*encoder_pulses_RR)/(SLOTS_ENCODER*SAMPLING_TIME));
	current_angular_velocity_wheels.w_FR = (directionOfRotation.w_FR == 0) ? ((2*M_PI*encoder_pulses_FR)/(SLOTS_ENCODER*SAMPLING_TIME)) : (-(2*M_PI*encoder_pulses_FR)/(SLOTS_ENCODER*SAMPLING_TIME));
	current_angular_velocity_wheels.w_FL = (directionOfRotation.w_FL == 0) ? ((2*M_PI*encoder_pulses_FL)/(SLOTS_ENCODER*SAMPLING_TIME)) : (-(2*M_PI*encoder_pulses_FL)/(SLOTS_ENCODER*SAMPLING_TIME));
	current_angular_velocity_wheels.w_RL = (directionOfRotation.w_RL == 0) ? ((2*M_PI*encoder_pulses_RL)/(SLOTS_ENCODER*SAMPLING_TIME)) : (-(2*M_PI*encoder_pulses_RL)/(SLOTS_ENCODER*SAMPLING_TIME));
	
	float v_RR = RADIUS_WHEEL * current_angular_velocity_wheels.w_RR;
	float v_FR = RADIUS_WHEEL * current_angular_velocity_wheels.w_FR;
	float v_FL = RADIUS_WHEEL * current_angular_velocity_wheels.w_FL;
	float v_RL = RADIUS_WHEEL * current_angular_velocity_wheels.w_RL;

	float v_R = (v_RR + v_FR) / 2;
	float v_L = (v_FL + v_RL) / 2;

	linear_velocity_x = (v_R + v_L) / 2;
	angular_velocity_z = (v_R - v_L) / WHEEL_SEPARATION;

	encoder_pulses_FR = 0;
	encoder_pulses_RR = 0;
	encoder_pulses_FL = 0;
	encoder_pulses_RL = 0;
}