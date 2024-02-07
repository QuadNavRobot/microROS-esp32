#include "../inc/main.h"

void app_main(void){

	init_microROS();

	init_encoder();

	createTask();

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
 * @brief Create the tasks.
 */
void createTask(){
	xTaskCreate(TaskEncoder,
        "Task encoder",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);	
	
	xTaskCreate(TaskIMU,
		"Task IMU",
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
 * @brief Task that gets data from MPU6050 and publishes it in the topic.
 * @param argument 
 */
void TaskIMU(void *argument){
	esp_err_t ret;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

	sensor_msgs__msg__Imu *dataIMU = sensor_msgs__msg__Imu__create();

    i2c_sensor_mpu6050_init();

    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
	while(1){
		ret = mpu6050_get_acce(mpu6050, &acce);
		TEST_ASSERT_EQUAL(ESP_OK, ret);
		
		dataIMU->linear_acceleration.x = convertGToMS2(acce.acce_x);
		dataIMU->linear_acceleration.y = convertGToMS2(acce.acce_y);
		dataIMU->linear_acceleration.z = convertGToMS2(acce.acce_z);

		ret = mpu6050_get_gyro(mpu6050, &gyro);
    	TEST_ASSERT_EQUAL(ESP_OK, ret);
    	
		dataIMU->angular_velocity.x = convertDegreesToRadians(gyro.gyro_x);
		dataIMU->angular_velocity.y = convertDegreesToRadians(gyro.gyro_y);
		dataIMU->angular_velocity.z = convertDegreesToRadians(gyro.gyro_z);

		RCSOFTCHECK(rcl_publish(&publisher_IMU, dataIMU, NULL));
		printf("Datos IMU publicados.\n");

		vTaskDelay(pdMS_TO_TICKS(1000));
	}

    mpu6050_delete(mpu6050);
    ret = i2c_driver_delete(I2C_NUM_0);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
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