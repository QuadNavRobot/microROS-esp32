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

	RCCHECK(rclc_publisher_init_default(
		&publisher_pose,
		&node_IMU,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"pose_estimation"));		

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

	IMUPublishValuesQueue = xQueueCreate(10, sizeof(float[6]));
	if(IMUPublishValuesQueue == NULL){ // Check if queue creation failed
		printf("Error xQueueCreate function\n");
	}

	IMUValuesQueue = xQueueCreate(10, sizeof(float[6]));
	if(IMUValuesQueue == NULL){ // Check if queue creation failed
		printf("Error xQueueCreate function\n");
	}
	
	EstimedQueue = xQueueCreate(10, sizeof(float[3]));
	if(EstimedQueue == NULL){ // Check if queue creation failed

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
		tskIDLE_PRIORITY+6,
		NULL);

	xTaskCreate(TaskPublishDataIMU,
		"Publish IMU",
		2000,
		NULL,
		CONFIG_MICRO_ROS_APP_TASK_PRIO,
		NULL);

	xTaskCreate(TaskPWM,
		"Task PWM",
		2100,
		NULL,
		CONFIG_MICRO_ROS_APP_TASK_PRIO -1,
		NULL);
	
	/*xTaskCreate(TaskSPI,
		"Task SPI Slave",
		2048,
		NULL,
		tskIDLE_PRIORITY+1,
		NULL);

	xTaskCreate(TaskFusion,
		"Task fusion sensorial",
		2048,
		NULL,
		tskIDLE_PRIORITY+1,
		NULL);

	xTaskCreate(TaskPublishFusion,
		"Task etimationp",
		2048,
		NULL,
		tskIDLE_PRIORITY+1,
		NULL);
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
		
		angular_velocity.data.data[0] = ((float)encoder_pulses_FL/20)*3.14;
		angular_velocity.data.data[1] = ((float)encoder_pulses_FR/20)*3.14;
		angular_velocity.data.data[2] = ((float)encoder_pulses_RR/20)*3.14;
		angular_velocity.data.data[3] = ((float)encoder_pulses_RL/20)*3.14;
		// printf("Stack free - ENCODER: %d \n",uxTaskGetStackHighWaterMark(NULL));
		RCSOFTCHECK(rcl_publish(&publisher_encoder, &angular_velocity, NULL));

		vTaskDelay(pdMS_TO_TICKS(100));
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
		
    	if (xQueueSend(IMUPublishValuesQueue, &values, portMAX_DELAY) != pdPASS) {
        	printf("ERROR: full queue.\n");
    	}
		if (xQueueSend(IMUValuesQueue, &values, portMAX_DELAY) != pdPASS) {
        	printf("ERROR: full queue.\n");
    	}
		
		vTaskDelay(pdMS_TO_TICKS(100));
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


    if(xQueueReceive(IMUPublishValuesQueue, values, portMAX_DELAY) == pdTRUE){ //Get the data from the queue
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


		vTaskDelay(pdMS_TO_TICKS(100));
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
 * PWM settings for engines
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
 * Task to manage the operation of the engines
*/
void TaskPWM(void *argument){
	
	PWM_config();
	
	for(;;){
	
		motor_forward(1,LEDC_CHANNEL_0, 64);
		motor_forward(1,LEDC_CHANNEL_1, 64);
		motor_forward(2,LEDC_CHANNEL_2, 64);
		motor_forward(2,LEDC_CHANNEL_3, 64);
		printf("Stack free - PWM: %d \n",uxTaskGetStackHighWaterMark(NULL));
		vTaskDelay(100);
	}
}

/**
 * 
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

void TaskFusion(void *argument){
	std_msgs__msg__Float32MultiArray msg;
  	std_msgs__msg__Float32MultiArray__init(&msg);
  	msg.data.data = malloc(sizeof(float) * 3);
  	msg.data.capacity = 3;
  	msg.data.size = 3;

  	float values[6]; // Array to store the values read from the queue

  	float v_yaw_fusion = 0.0;
  	Position v_position_fusion = {0.0, 0.0};
  	DataIMU prev_velocity_imu = {0.0, 0.0, 0.0};

  	uint32_t current_time;
  	uint32_t prev_time = (uint32_t) xTaskGetTickCount();

	static float d_t_imu = 1.0;

	while (1){
		if(xQueueReceive(IMUValuesQueue, values, portMAX_DELAY) == pdTRUE){ //Get the data from the queue
			current_time = (uint32_t) xTaskGetTickCount();
        	float ticks_time = (float)(current_time - prev_time);//Cada tick es 1ms
        	float dt = ticks_time / 1000;

			Wheels despl_linear = { 
					(encoder_pulses_RR/20) * 6.5,
					(encoder_pulses_FR/20) * 6.5,
					(encoder_pulses_RL/20) * 6.5,
					(encoder_pulses_FL/20) * 6.5};
			
			//Reinicio el contador de las ruedas
			encoder_pulses_FR = 0;
			encoder_pulses_FL = 0;
			encoder_pulses_RL = 0;
			encoder_pulses_RR = 0;

			VelocityWheels vel_wheels = calculate_velocities_wheels(despl_linear, dt);

			DataIMU linear_aceleration = {values[0],
										  values[1],
										  values[2]};

			DataIMU angular_velocity = {convertDegreesToRadians(values[3]),
										convertDegreesToRadians(values[4]),
										convertDegreesToRadians(values[5])};

			//Estimaciones de yaw.
			float wheels_yaw_estimed = calculate_yaw_wheels(vel_wheels, v_yaw_fusion, 0.13, dt);
			float imu_yaw_estimed = calculate_yaw_imu(angular_velocity, v_yaw_fusion, d_t_imu);

			v_yaw_fusion = get_orientation_f_c(imu_yaw_estimed, wheels_yaw_estimed, alpha);

			//Estimaciones de las posiciones.
			Position position_wheels_estime = calculate_position_wheels(vel_wheels, v_position_fusion, dt, v_yaw_fusion);
			Position position_imu_estime = calculate_position_imu(linear_aceleration, &prev_velocity_imu, v_position_fusion, d_t_imu, v_yaw_fusion);

			Position position_fusion = get_position_f_c(position_imu_estime, position_wheels_estime, gamma, beta);
			v_position_fusion = position_fusion;

			prev_time = current_time;

			values[0] = v_position_fusion.x;
			values[1] = v_position_fusion.y;
			values[2] = v_yaw_fusion;

	    	if (xQueueSend(EstimedQueue, &values, portMAX_DELAY) != pdPASS) {
	    		printf("ERROR: full queue.\n");
	    	}
		}
	}
}

void TaskPublishFusion(void *argument){
	float values[3];

	std_msgs__msg__Float32MultiArray msg;
	std_msgs__msg__Float32MultiArray__init(&msg);
	msg.data.data = malloc(sizeof(float) * 3);
	msg.data.capacity = 3;
	msg.data.size = 3;

	while(1){
    	if(xQueueReceive(EstimedQueue, values, portMAX_DELAY) == pdTRUE){

			msg.data.data[0] = values[0];
			msg.data.data[1] =  values[1];
			msg.data.data[2] =  values[2];
		    RCSOFTCHECK(rcl_publish(&publisher_pose, &msg, NULL));
  		}
	}
}