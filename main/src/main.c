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

	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	
}


/**
 * @brief Create the tasks and queue.
 */
void FreeRTOS_Init(){
	
	IMUQueue = xQueueCreate(10, sizeof(float[6]));
	if(IMUQueue == NULL){ // Check if queue creation failed
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

	xTaskCreate(TaskPWM,
		"Task PWM",
		2100,
		NULL,
		1,
		NULL);
	
	/*xTaskCreate(TaskSPI,
		"Task SPI Slave",
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
		
    	if (xQueueSend(IMUQueue, &values, portMAX_DELAY) != pdPASS) {
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

    if(xQueueReceive(IMUQueue, values, portMAX_DELAY) == pdTRUE){ //Get the data from the queue
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
	/* TO DO: falta dividirlo por el tiempo. todas los contadores estan rotos porque se resetean
	en calculate_current_velocity. Considerar poner los valores en una cola
	*/
	angular_velocity.data.data[0] = current_velocity_FL; //((float)encoder_pulses_FL/20)*360;
	angular_velocity.data.data[1] = current_velocity_FR; //((float)encoder_pulses_FR/20)*360;
	angular_velocity.data.data[2] = current_velocity_RR; //((float)encoder_pulses_RR/20)*360;
	angular_velocity.data.data[3] = current_velocity_RL; //((float)encoder_pulses_RL/20)*360;

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
	PID_Init();

	//MOTORES APAGADOS
	set_velocity(CHANNEL_FL, 0);
	set_velocity(CHANNEL_FR, 0);
	set_velocity(CHANNEL_RR, 0);
	set_velocity(CHANNEL_RL, 0);
	for(;;){

		set_velocity(CHANNEL_FL, 0.4);
		set_velocity(CHANNEL_FR, 0.4);
		set_velocity(CHANNEL_RR, 0.4);
		set_velocity(CHANNEL_RL, 0.4);
		vTaskDelay(10);

		//printf("Stack free - PWM: %d \n",uxTaskGetStackHighWaterMark(NULL));

		//vTaskDelay(10); //1 tick 10 ms
	}
}