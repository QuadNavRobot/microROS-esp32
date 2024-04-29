#include "../inc/main.h"

void app_main(void){
	if (!DEBUG_MODE)
	{
		init_microROS();
	}

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

	RCCHECK(rclc_node_init_default(&esp32_node, "esp32_node", "", &support));

	RCCHECK(rclc_publisher_init_default(
		&publisher_encoder,
		&esp32_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"encoders"));

	//RCCHECK(rclc_node_init_default(&node_IMU, "IMU_node", "", &support));

	RCCHECK(rclc_publisher_init_default(
		&publisher_IMU,
		&esp32_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"IMU"));

	RCCHECK(rclc_publisher_init_default(
		&publisher_pose,
		&esp32_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"pose_estimation"));		

	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	
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
		printf("Error xQueueCreate function\n");
	}
	
	xTaskCreate(TaskReadDataIMU,
		"Read IMU",
		2000,
		NULL,
		2,
		NULL);

	xTaskCreate(TaskPublishDataSensors,
		"Publish IMU",
		2000,
		NULL,
		1,
		NULL);

	// xTaskCreate(TaskPWM,
	// 	"Task PWM",
	// 	2100,
	// 	NULL,
	// 	1,
	// 	NULL);
	
	/*xTaskCreate(TaskSPI,
		"Task SPI Slave",
		2048,
		NULL,
		tskIDLE_PRIORITY+1,
		NULL);*/

	xTaskCreate(TaskFusion,
		"Task fusion sensorial",
		2500,
		NULL,
		1,
		NULL);
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

    mpu6050 = i2c_sensor_mpu6050_init();

    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
	ret = mpu6050_calibrate(mpu6050);
	TEST_ASSERT_EQUAL(ESP_OK, ret);
	
	while(1){
		//printf("Stack free - READ: %d \n",uxTaskGetStackHighWaterMark(NULL));
		ret = mpu6050_get_acce_cal(mpu6050, &acce);
		TEST_ASSERT_EQUAL(ESP_OK, ret);

		values[0] = acce.acce_x;
		values[1] = acce.acce_y;
		values[2] = acce.acce_z;

		ret = mpu6050_get_gyro_cal(mpu6050, &gyro);
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
void TaskPublishDataSensors(void *argument){

	float raw_imu_values[6];
	float filter_estimates[3];
	sensor_msgs__msg__Imu *dataIMU = sensor_msgs__msg__Imu__create();
    std_msgs__msg__Float32MultiArray angular_velocity;
	std_msgs__msg__Float32MultiArray__init (&angular_velocity);
	angular_velocity.data.data = malloc(sizeof(float)*4);
	angular_velocity.data.capacity = 4;
	angular_velocity.data.size = 4; 
	std_msgs__msg__Float32MultiArray pose_estimate;
	std_msgs__msg__Float32MultiArray__init(&pose_estimate);
	pose_estimate.data.data = malloc(sizeof(float) * 3);
	pose_estimate.data.capacity = 3;
	pose_estimate.data.size = 3;

	while(1){
	//IMU
    if(xQueueReceive(IMUPublishValuesQueue, raw_imu_values, portMAX_DELAY) == pdTRUE){ //Get the data from the queue
		dataIMU->linear_acceleration.x = convertGToMS2(raw_imu_values[0]);
		dataIMU->linear_acceleration.y = convertGToMS2(raw_imu_values[1]);
		dataIMU->linear_acceleration.z = convertGToMS2(raw_imu_values[2]);

		dataIMU->angular_velocity.x = convertDegreesToRadians(raw_imu_values[3]);
		dataIMU->angular_velocity.y = convertDegreesToRadians(raw_imu_values[4]);
		dataIMU->angular_velocity.z = convertDegreesToRadians(raw_imu_values[5]);

		if (DEBUG_MODE)
		{
			if (PRINT_IMU_DEBUG)
			{
				printf("DataIMU Accelerometer - x: %f, y: %f, z:%f.\n", dataIMU->linear_acceleration.x, dataIMU->linear_acceleration.y, dataIMU->linear_acceleration.z);
				printf("DataIMU Gyroscope - x: %f, y: %f, z:%f.\n", dataIMU->angular_velocity.x, dataIMU->angular_velocity.y, dataIMU->angular_velocity.z);
			}
		}else{
			RCSOFTCHECK(rcl_publish(&publisher_IMU, dataIMU, NULL));
		}

	}
	//Encoders
	angular_velocity.data.data[0] = ((float)encoder_pulses_FL/20)*360;
	angular_velocity.data.data[1] = ((float)encoder_pulses_FR/20)*360;
	angular_velocity.data.data[2] = ((float)encoder_pulses_RR/20)*360;
	angular_velocity.data.data[3] = ((float)encoder_pulses_RL/20)*360;

	if(DEBUG_MODE){
		if(PRINT_ENCODERS_DEBUG)
		{
			printf("DataEncoders - Left front: %f, Left rear: %f, Right front: %f, Right rear: %f.\n", angular_velocity.data.data[0], angular_velocity.data.data[1], angular_velocity.data.data[2], angular_velocity.data.data[3]);
		}
	}else {
		RCSOFTCHECK(rcl_publish(&publisher_encoder, &angular_velocity, NULL));
	}

	//Filter
	if(xQueueReceive(EstimedQueue, filter_estimates, portMAX_DELAY) == pdTRUE){

		pose_estimate.data.data[0] = filter_estimates[0];
		pose_estimate.data.data[1] =  filter_estimates[1];
		pose_estimate.data.data[2] =  filter_estimates[2];

		if (DEBUG_MODE)
		{
			if (PRINT_FILTER_DEBUG)
			{
				printf("FilterComplementary - pos_x: %f, pos_y: %f, or_yaw: %f.\n", pose_estimate.data.data[1], pose_estimate.data.data[2], pose_estimate.data.data[0]);
			}
		}else{
			RCSOFTCHECK(rcl_publish(&publisher_pose, &pose_estimate, NULL));
		}
  	}
	}
}

/**
 * Task to manage the operation of the engines
*/
void TaskPWM(void *argument){
	
	 PWM_config();
	
	//APAGADOS
	motor_forward(CHANNEL_RR, 0);
	motor_forward(CHANNEL_FR, 0);
	motor_forward(CHANNEL_FL, 0);
	motor_forward(CHANNEL_RL, 0);
	
	for(;;){
		//AL 50%
		motor_forward(CHANNEL_RR, 50);
		motor_forward(CHANNEL_FR, 50);
		motor_forward(CHANNEL_FL, 50);
		motor_forward(CHANNEL_RL, 50);

		vTaskDelay(500);
		motor_backward(CHANNEL_RR, 50);
		motor_backward(CHANNEL_FR, 50);
		motor_backward(CHANNEL_FL, 50);
		motor_backward(CHANNEL_RL, 50);
		printf("Stack free - PWM: %d \n",uxTaskGetStackHighWaterMark(NULL));
		vTaskDelay(500);
	}
}

/*void TaskSPI(void *argument){
	//Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=-1,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=GPIO_CS,
        .queue_size=3,
        .flags=0
    };

	//gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLDOWN_ONLY);
	//gpio_set_pull_mode(GPIO_MISO, GPIO_PULLDOWN_ONLY);
    //gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLDOWN_ONLY);
    //gpio_set_pull_mode(GPIO_CS, GPIO_PULLDOWN_ONLY);

	spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, dutty));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}*/

void TaskFusion(void *argument){
	std_msgs__msg__Float32MultiArray msg;
  	std_msgs__msg__Float32MultiArray__init(&msg);
  	msg.data.data = malloc(sizeof(float) * 3);
  	msg.data.capacity = 3;
  	msg.data.size = 3;

  	float values_output[3]; 
	float values[6]; // Array to store the values read from the queue

  	float yaw = 0.0;
	float prev_yaw = 0.0;
  	Position position = {0.0, 0.0};
	Position prev_position = {0.0, 0.0};

	DataIMU prev_velocities_imu = {0, 0, 0};

	static float dt = 0.1;

	// int encoder_pulses_FR_count = 0;
	// int encoder_pulses_FL_count = 0;
	// int encoder_pulses_RL_count = 0;
	// int encoder_pulses_RR_count = 0;

	while (1){
		if(xQueueReceive(IMUValuesQueue, values, portMAX_DELAY) == pdTRUE){ //Get the data from the queue

			// CALCULO CON LAS RUEDAS ------------------------------------------

			// Wheels despl_linear;
			// despl_linear.right_rear = ((float)encoder_pulses_RR/20) * 3.14 * 0.065;
			// despl_linear.right_front = ((float)encoder_pulses_FR/20) * 3.14 * 0.065;
			// despl_linear.left_rear = ((float)encoder_pulses_RL/20) * 3.14 * 0.065;
			// despl_linear.left_front = ((float)encoder_pulses_FL/20) * 3.14 * 0.065;

			// // printf("Datos de encoders- right_rear: %d, right_front:%d, left_rear:%d, left_front: %d.\n", encoder_pulses_RR,  encoder_pulses_FR,  encoder_pulses_RL, encoder_pulses_FL);
			// // printf("Datos de desplazamient lineal - right_rear: %f, right_front:%f, left_rear:%f, left_front: %f.\n", despl_linear.right_rear,  despl_linear.right_front,  despl_linear.left_rear, despl_linear.left_front);
			
			// encoder_pulses_FR_count += encoder_pulses_FR;
			// encoder_pulses_FL_count += encoder_pulses_FL;
			// encoder_pulses_RL_count += encoder_pulses_RL;
			// encoder_pulses_RR_count += encoder_pulses_RR;

			// printf("Datos de encoders- right_rear: %d, right_front:%d, left_rear:%d, left_front: %d.\n",encoder_pulses_RR_count, encoder_pulses_FR_count,  encoder_pulses_RL_count, encoder_pulses_FL_count);

			// //Reinicio el contador de las ruedas
			// encoder_pulses_FR = 0;
			// encoder_pulses_FL = 0;
			// encoder_pulses_RL = 0;
			// encoder_pulses_RR = 0;

			// VelocityWheels vel_wheels = calculate_velocities_wheels(despl_linear, dt);
			// // printf("Velocidades - izquierda: %f, derecha: %f.\n", vel_wheels.left_wheels, vel_wheels.right_wheels);

			// //Estimaciones de yaw.
			// yaw = calculate_yaw_wheels(vel_wheels, yaw, 0.13, dt);

			// //Estimaciones de las posiciones.
			// position = calculate_position_wheels(vel_wheels, position, dt, yaw);
			// ------------------------------------------------------------------------------


			// CALCULO CON IMU  ------------------------------------------

			// printf("Datos de IMU linear acceleration - x: %f, y: %f, z: %f.\n", values[0], values[1], values[2]);
			// printf("Datos de IMU angular velocities - x: %f, y: %f, z: %f.\n", values[3], values[4], values[5]);

			DataIMU linear_acceleration = {
				convertGToMS2(values[0]),
				convertGToMS2(values[1]),
				convertGToMS2(values[2])
			};

			DataIMU angular_velocities = {
				convertDegreesToRadians(values[3]),
				convertDegreesToRadians(values[4]),
				convertDegreesToRadians(values[5])
			};

			yaw = calculate_yaw_imu(angular_velocities, prev_yaw, dt);
			prev_yaw = yaw;

			position = calculate_position_imu(linear_acceleration, &prev_velocities_imu, prev_position, dt, yaw);
			prev_position = position;
			// ------------------------------------------------------------------------------

			values_output[0] = position.x;
			values_output[1] = position.y;
			values_output[2] = yaw;
			printf("Calculos realizados - x: %f, y:%f, yaw:%f.\n", values_output[0],  values_output[1],  values_output[2]);

	    	// if (xQueueSend(EstimedQueue, &values_output, portMAX_DELAY) != pdPASS) {
	    	// 	printf("ERROR: full queue.\n");
	    	// }
		}
	}
}