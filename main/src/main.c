#include "../inc/main.h"

void app_main(void){

	init_microROS();

	init_encoder();

	FreeRTOS_Init();

}

/**
 * @brief Configure micro-ROS. Create the nodes and publishers.
 */
void init_microROS(){

	#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    	ESP_ERROR_CHECK(uros_network_interface_initialize());
	#endif
	allocator = rcl_get_default_allocator();
	
	init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

	#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
	#endif

	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	RCCHECK(rclc_node_init_default(&node_encoder, "encoder_node", "", &support));

	RCCHECK(rclc_publisher_init_default(
		&publisher_encoder,
		&node_encoder,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"encoder"));

	RCCHECK(rclc_node_init_default(&node_IMU, "IMU_node", "", &support));

	RCCHECK(rclc_publisher_init_default(
		&publisher_IMU,
		&node_IMU,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"IMU"));

	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	
}

/**
 * @brief Configure GPIO 27 to receive encoder interrupts.
 * @return esp_err_t 
 */
esp_err_t init_encoder(){

	gpio_config_t gpio_config_struct = {
		.pin_bit_mask = (1ULL << 27),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type = GPIO_INTR_POSEDGE
	};

	gpio_config(&gpio_config_struct);
	
	gpio_install_isr_service(0);
	
	gpio_isr_handler_add(27,isr_handler, NULL);

	return ESP_OK;
}

/**
 * @brief Create the tasks and queue.
 */
void FreeRTOS_Init(){

	IMUQueue = xQueueCreate(10, sizeof(float[6]));
	if(IMUQueue == NULL){ // Check if queue creation failed
		printf("Error xQueueCreate function\n");
	}

	xTaskCreate(TaskEncoder,
        "Task encoder",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);	
	
	xTaskCreate(TaskReadDataIMU,
		"Task to read data IMU",
		1024,
		NULL,
		tskIDLE_PRIORITY+1,
		NULL);

	xTaskCreate(TaskPublishDataIMU,
		"Task to publish data IMU",
		CONFIG_MICRO_ROS_APP_STACK,
		NULL,
		CONFIG_MICRO_ROS_APP_TASK_PRIO,
		NULL);

	xTaskCreate(TaskPWM,
		"Task PWM",
		CONFIG_MICRO_ROS_APP_STACK,
		NULL,
		CONFIG_MICRO_ROS_APP_TASK_PRIO,
		NULL);
}

/**
 * @brief ISR of encoder interrupts
 */
void isr_handler(){encoder_pulses++;}

/**
 * @brief Task that calculates the angular velocity and publishes it in the topic.
 * @param argument 
 */
void TaskEncoder(void *argument){
	std_msgs__msg__Float32 angular_velocity;
	
    for(;;){
		
		angular_velocity.data = ((float)encoder_pulses/20)*360;
		printf("Publishing angular velocity: %f\n", angular_velocity.data);
		
		RCSOFTCHECK(rcl_publish(&publisher_encoder, &angular_velocity, NULL));

		vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Task that reads the accelerometer and gyroscope data from the MPU6050 and saves it in a float array
 * @param argument Pointer to the task arguments (not used)
 */
void TaskReadDataIMU(void *argument){
	esp_err_t ret;
	float values[6];
	uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    i2c_sensor_mpu6050_init();

    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
	
	while(1){
		ret = mpu6050_get_acce(mpu6050, &acce);
		TEST_ASSERT_EQUAL(ESP_OK, ret);

		values[0] = acce.acce_x;
		values[1] = acce.acce_y;
		values[2] = acce.acce_z;

		ret = mpu6050_get_gyro(mpu6050, &gyro);
    	TEST_ASSERT_EQUAL(ESP_OK, ret);
		
		values[3] = gyro.gyro_x;
		values[4] = gyro.gyro_y;
		values[5] = gyro.gyro_z;

    	if (xQueueSend(IMUQueue, &values, portMAX_DELAY) != pdPASS) {
        	printf("ERROR: full queue.\n");
    	}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

/**
 * @brief Task that receives the IMU data from the queue, converts it to a IMU msg 
 * and publishes it to a topic using micro-ROS
 * @param argument Pointer to the task arguments (not used)
 */
void TaskPublishDataIMU(void *argument){

	float values[6];
	sensor_msgs__msg__Imu *dataIMU = sensor_msgs__msg__Imu__create();
    
	while(1){


    if(xQueueReceive(IMUQueue, values, portMAX_DELAY) == pdTRUE){ //Get the data from the queue
		dataIMU->linear_acceleration.x = convertGToMS2(values[0]);
		dataIMU->linear_acceleration.y = convertGToMS2(values[1]);
		dataIMU->linear_acceleration.z = convertGToMS2(values[2]);

		dataIMU->angular_velocity.x = convertDegreesToRadians(values[3]);
		dataIMU->angular_velocity.y = convertDegreesToRadians(values[4]);
		dataIMU->angular_velocity.z = convertDegreesToRadians(values[5]);

		RCSOFTCHECK(rcl_publish(&publisher_IMU, dataIMU, NULL));
		printf("Datos IMU publicados.\n");
	}


		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

/**
 * @brief Initialize i2c and MPU6050.
 */
static void i2c_sensor_mpu6050_init(void){
    esp_err_t ret;

    i2c_Init();
    mpu6050 = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

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

/**
 * @brief Configure PWM for motor driver L298 (two motors)
 * 
 */
void PWM_config(){
	
	// MOTOR 1
	// gpio initialization
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 16); //IN1
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 17); //IN2

	// pwm config
	mcpwm_config_t pwm_config = {
		.frequency = 1000,
		.cmpr_a = 0,
		.cmpr_b = 0,
		.duty_mode = MCPWM_DUTY_MODE_0,
		.counter_mode = MCPWM_UP_COUNTER
	};

	mcpwm_init(MCPWM_UNIT_0,MCPWM_TIMER_0, &pwm_config);

	// MOTOR 2
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, 4); // IN3
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, 5); // IN4

	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
}

/**
 * @brief Make the motor rotate forward.
 * 
 * @param mcpwm_num 
 * @param timer_num 
 * @param duty_cycle 
 * @param gen_low 
 * @param gen_pwm 
 */
void motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle, mcpwm_generator_t gen_low, mcpwm_generator_t gen_pwm){
	mcpwm_set_signal_low(mcpwm_num, timer_num, gen_low);
    mcpwm_set_duty(mcpwm_num, timer_num, gen_pwm, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, gen_pwm, MCPWM_DUTY_MODE_0);	
}

/**
 * @brief Make the motor rotate backward.
 * 
 * @param mcpwm_num 
 * @param timer_num 
 * @param duty_cycle 
 * @param gen_low 
 * @param gen_pwm 
 */
void motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle, mcpwm_generator_t gen_low, mcpwm_generator_t gen_pwm){
	mcpwm_set_signal_low(mcpwm_num, timer_num, gen_low);
    mcpwm_set_duty(mcpwm_num, timer_num, gen_pwm, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, gen_pwm, MCPWM_DUTY_MODE_0);
}

/**
 * @brief Make the motor stop.
 * 
 * @param mcpwm_num 
 * @param timer_num 
 * @param gen_in1 
 * @param gen_in2 
 */
void motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, mcpwm_generator_t gen_in1, mcpwm_generator_t gen_in2){
    mcpwm_set_signal_low(mcpwm_num, timer_num, gen_in1);
    mcpwm_set_signal_low(mcpwm_num, timer_num, gen_in2);
}

/**
 * @brief Task that handler the pwm of motors. 
 * 
 * @param argument 
 */
void TaskPWM(void *argument){

	PWM_config();

	while(1){

		// Giro en un sentido
		printf("Sentido 1\n");
		motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50, MCPWM_OPR_B, MCPWM_OPR_A);
		motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, 20, MCPWM_OPR_B, MCPWM_OPR_A);
		vTaskDelay(pdMS_TO_TICKS(5000));

		// Giro en otro sentido
		printf("Sentido 2\n");
		motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50, MCPWM_OPR_A, MCPWM_OPR_B);
		motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, 20, MCPWM_OPR_A, MCPWM_OPR_B);
		vTaskDelay(pdMS_TO_TICKS(5000));

		// Stop
		//printf("Sentido 3\n");
		//motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_OPR_B);
		//motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_OPR_B);
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}



