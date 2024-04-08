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
		1,
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

		RCSOFTCHECK(rcl_publish(&publisher_IMU, dataIMU, NULL));


	}
		angular_velocity.data.data[0] = ((float)encoder_pulses_FL/20)*360;
		angular_velocity.data.data[1] = ((float)encoder_pulses_FR/20)*360;
		angular_velocity.data.data[2] = ((float)encoder_pulses_RR/20)*360;
		angular_velocity.data.data[3] = ((float)encoder_pulses_RL/20)*360;
		RCSOFTCHECK(rcl_publish(&publisher_encoder, &angular_velocity, NULL));
		

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

	char recvbuf[100]="";

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
	t.length = 100 * 8;
    t.rx_buffer = recvbuf;

	while(1) {

		memset(recvbuf,'\0', sizeof(recvbuf));

        // Wait for data from master
		//printf("Esperando datos\n");
        esp_err_t ret = spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY);
        if(ret == ESP_OK) {
            printf("Received: %s\n", recvbuf);

        } else {
            printf("SPI slave error\n");
        }
		//vTaskDelay(pdMS_TO_TICKS(1000));
    }
}*/