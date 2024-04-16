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
		1,
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
		NULL);*/
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
	
	while(1){
		//printf("Stack free - READ: %d \n",uxTaskGetStackHighWaterMark(NULL));
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
void TaskPublishDataSensors(void *argument){

	float values[6];
	sensor_msgs__msg__Imu *dataIMU = sensor_msgs__msg__Imu__create();
    std_msgs__msg__Float32MultiArray angular_velocity;
	std_msgs__msg__Float32MultiArray__init (&angular_velocity);
	angular_velocity.data.data = malloc(sizeof(float)*4);
	angular_velocity.data.capacity = 4;
	angular_velocity.data.size = 4; 

	while(1){

    if(xQueueReceive(IMUPublishValuesQueue, values, portMAX_DELAY) == pdTRUE){ //Get the data from the queue
		dataIMU->linear_acceleration.x = convertGToMS2(values[0]);
		dataIMU->linear_acceleration.y = convertGToMS2(values[1]);
		dataIMU->linear_acceleration.z = convertGToMS2(values[2]);

		dataIMU->angular_velocity.x = convertDegreesToRadians(values[3]);
		dataIMU->angular_velocity.y = convertDegreesToRadians(values[4]);
		dataIMU->angular_velocity.z = convertDegreesToRadians(values[5]);

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
	
		vTaskDelay(pdMS_TO_TICKS(100));
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
		//printf("Stack free - PWM: %d \n",uxTaskGetStackHighWaterMark(NULL));
		vTaskDelay(100);
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