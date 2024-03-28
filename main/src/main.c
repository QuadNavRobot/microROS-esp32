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
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
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
 * @brief Configure GPIO pins to receive encoder interrupts.
 * @return esp_err_t 
 */
esp_err_t init_encoder(){

	gpio_config_t gpio_config_struct = {
		.pin_bit_mask = ((1ULL << GPIO_ENCODER_FR) | (1ULL << GPIO_ENCODER_FL) | (1ULL << GPIO_ENCODER_RR) | (1ULL << GPIO_ENCODER_RL)),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type = GPIO_INTR_POSEDGE
	};

	gpio_config(&gpio_config_struct);
	
	gpio_install_isr_service(0);

	gpio_isr_handler_add(GPIO_ENCODER_FR, isr_handler, (void*)GPIO_ENCODER_FR);
	gpio_isr_handler_add(GPIO_ENCODER_FL, isr_handler, (void*)GPIO_ENCODER_FL);
	gpio_isr_handler_add(GPIO_ENCODER_RR, isr_handler, (void*)GPIO_ENCODER_RR);
	gpio_isr_handler_add(GPIO_ENCODER_RL, isr_handler, (void*)GPIO_ENCODER_RL);

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
        "Encoder",
        2000,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);	
	
	xTaskCreate(TaskReadDataIMU,
		"Read IMU",
		2000,
		NULL,
		tskIDLE_PRIORITY+1,
		NULL);

	xTaskCreate(TaskPublishDataIMU,
		"Publish IMU",
		2000,
		NULL,
		CONFIG_MICRO_ROS_APP_TASK_PRIO,
		NULL);

	xTaskCreate(TaskPWM,
		"Task PWM",
		2000,
		NULL,
		CONFIG_MICRO_ROS_APP_TASK_PRIO,
		NULL);
	
	/*xTaskCreate(TaskSPI,
		"Task SPI Slave",
		2048,
		NULL,
		tskIDLE_PRIORITY+1,
		NULL);*/

	
}

/**
 * @brief ISR of encoder interrupts
 */
static void IRAM_ATTR isr_handler(void* arg){
	uint32_t gpio_num = (uint32_t) arg;
	
	if(gpio_num == GPIO_ENCODER_FL){
		encoder_pulses_FL++;
	}else if(gpio_num == GPIO_ENCODER_FR){
		encoder_pulses_FR++;
	}else if(gpio_num == GPIO_ENCODER_RR){
		encoder_pulses_RR++;
	}else{
		encoder_pulses_RL++;
	}
}

/**
 * @brief Task that calculates the angular velocity and publishes it in the topic.
 * @param argument 
 */
void TaskEncoder(void *argument){

	std_msgs__msg__Float32MultiArray angular_velocity;
	std_msgs__msg__Float32MultiArray__init (&angular_velocity);
	angular_velocity.data.data = malloc(sizeof(float)*4);
	angular_velocity.data.capacity = 4;
	angular_velocity.data.size = 4;
    for(;;){
		
		printf("Stack free - ENCODER: %d \n",uxTaskGetStackHighWaterMark(NULL)); // Devuelve el stack free - Aprox 360 words free
		angular_velocity.data.data[0] = ((float)encoder_pulses_FL/20)*360;
		angular_velocity.data.data[1] = ((float)encoder_pulses_FR/20)*360;
		angular_velocity.data.data[2] = ((float)encoder_pulses_RR/20)*360;
		angular_velocity.data.data[3] = ((float)encoder_pulses_RL/20)*360;
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
		printf("Stack free - READ: %d \n",uxTaskGetStackHighWaterMark(NULL));
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
		printf("Stack free - PUBLISH: %d \n",uxTaskGetStackHighWaterMark(NULL));
		RCSOFTCHECK(rcl_publish(&publisher_IMU, dataIMU, NULL));
		//printf("Datos IMU publicados.\n");
	}


		//vTaskDelay(pdMS_TO_TICKS(1000));
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


void PWM_config(){

	ledc_timer_config_t ledc_timer;

	ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_timer.duty_resolution = LEDC_TIMER_14_BIT;
	ledc_timer.timer_num = LEDC_TIMER_0;
	ledc_timer.freq_hz = 1000;
	ledc_timer.clk_cfg = LEDC_AUTO_CLK;

	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	ledc_channel_config_t ledc_channel[4];

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
	
	gpio_set_direction(GPIO_IN1_DM1, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_IN2_DM1, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_IN1_DM2, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_IN2_DM2, GPIO_MODE_OUTPUT);
}

void TaskPWM(void *argument){
	
	PWM_config();

	for(;;){

		motor_forward(1,LEDC_CHANNEL_0, 8192);
		motor_forward(1,LEDC_CHANNEL_1, 12288);

		vTaskDelay(100);
	}
}

/***
 * calculo del dutty -> 2**(duty_resolution)*dutty_percentage%. Ej: 2**(14)*50% dutty 50%
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
}

void motor_backward(int driver_motor, ledc_channel_t channel, uint32_t dutty){
	if(driver_motor == 1){
		gpio_set_level(GPIO_IN1_DM1, 0);
		gpio_set_level(GPIO_IN2_DM1, 1);
	}else{
		gpio_set_level(GPIO_IN1_DM2, 0);
		gpio_set_level(GPIO_IN2_DM2, 1);	
	}

	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, dutty));
}