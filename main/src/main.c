#include "../inc/main.h"

void app_main(void){

	init_microROS();
	init_encoder();
	FreeRTOS_Init();
}

/**
 * @brief Configure micro-ROS. Create the robot node, the odometry publisher and the velocity command subscriber
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
	
	RCCHECK(rclc_publisher_init_best_effort(
		&publisher_IMU,
		&esp32_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"IMU"));

	RCCHECK(rclc_publisher_init_best_effort(
		&publisher_encoder,
		&esp32_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistWithCovarianceStamped),
		"encoders"));

	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
	
	geometry_msgs__msg__Twist__init(&twist_msg);

	RCCHECK(rclc_subscription_init_best_effort(
		&subscription_velocities,
		&esp32_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel"
	));

	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&subscription_velocities,
		&twist_msg,
		&twist_callback,
		ON_NEW_DATA
	));


	RCCHECK(rclc_subscription_init_best_effort(
		&sub_unix_time,
		&esp32_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(builtin_interfaces, msg, Time),
		"unix_time"
	));

	builtin_interfaces__msg__Time__init(&received_time);
	
	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&sub_unix_time,
		&received_time,
		&unix_time_callback,
		ON_NEW_DATA
	));
}

/**
 * @brief Create the tasks and queues
 */
void FreeRTOS_Init(){
	
	IMUQueue = xQueueCreate(10, sizeof(float[6]));
	if(IMUQueue == NULL){
		printf("Error xQueueCreate function\n");
	}
	TwistReceivedQueue = xQueueCreate(10, sizeof(ReceivedTwist));
	if(IMUQueue == NULL){
		printf("Error xQueueCreate function\n");
	}

	xTaskCreate(TaskReadDataIMU,
		"Read IMU",
		2000,
		NULL,
		1,
		NULL);

	xTaskCreate(TaskPublishDataSensors,
		"Publish data sensors",
		3000,
		NULL,
		1,
		NULL);

	xTaskCreate(TaskMotorControl,
		"Task motor control",
		3000,
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
		current_gyro_z += gyro.gyro_z * SAMPLING_TIME;
		current_gyro_z = angle_wrap(current_gyro_z);
		
    	if (xQueueSend(IMUQueue, &values, portMAX_DELAY) != pdPASS){
        	printf("ERROR: full queue.\n");
    	}
		
		vTaskDelay(pdMS_TO_TICKS(DELAY_TIME));
	}
}

/**
 * @brief Task that publishes the robot's odometry from the IMU data and the encoders
 * @param argument Pointer to the task arguments (not used)
 */
void TaskPublishDataSensors(void *argument){
	float values[6];
	Position position;
	
	sensor_msgs__msg__Imu *dataIMU = sensor_msgs__msg__Imu__create();
	geometry_msgs__msg__TwistWithCovarianceStamped dataEncoders;

	rosidl_runtime_c__String__init(&dataIMU->header.frame_id);
	rosidl_runtime_c__String__init(&dataEncoders.header.frame_id);
	 memset(&dataEncoders, 0, sizeof(geometry_msgs__msg__TwistWithCovarianceStamped));

	while(1){

		if(xQueueReceive(IMUQueue, values, portMAX_DELAY) == pdTRUE){ //Get data from the queue
			dataIMU->linear_acceleration.x = convertGToMS2(values[0]);
			dataIMU->linear_acceleration.y = convertGToMS2(values[1]);
			dataIMU->linear_acceleration.z = convertGToMS2(values[2]);
			dataIMU->angular_velocity.x = convertDegreesToRadians(values[3]);
			dataIMU->angular_velocity.y = convertDegreesToRadians(values[4]);
			dataIMU->angular_velocity.z = convertDegreesToRadians(values[5]);
		}

		dataIMU->header.stamp.sec = seconds;
		dataIMU->header.stamp.nanosec = nanoseconds;
		rosidl_runtime_c__String__assign(&dataIMU->header.frame_id, "base_link");
		double orientation_covariance[9] = {-1, 0, 0, 0, -1, 0, 0, 0, -1};
		memcpy(dataIMU->orientation_covariance, orientation_covariance, sizeof(orientation_covariance));
		double angular_velocity_covariance [9] = {0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.005};		
		memcpy(dataIMU->angular_velocity_covariance, angular_velocity_covariance, sizeof(angular_velocity_covariance));
		double linear_acceleration_covariance [9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
		memcpy(dataIMU->linear_acceleration_covariance, linear_acceleration_covariance, sizeof(linear_acceleration_covariance));

		dataEncoders.header.stamp.sec = seconds;
		dataEncoders.header.stamp.nanosec = nanoseconds;
		double twist_covariance[36] = {0.001, 0, 0, 0, 0, 0,  // v_x
									   0, 0.001, 0, 0, 0, 0,	 // v_y
									   0, 0, 0.001, 0, 0, 0,	 // v_z
									   0, 0, 0, 0.001, 0, 0,	 // w_x
									   0, 0, 0, 0, 0.001, 0,	 //	w_y
									   0, 0, 0, 0, 0, 1000}; // w_z

		memcpy(dataEncoders.twist.covariance, twist_covariance, sizeof(twist_covariance));
		rosidl_runtime_c__String__assign(&dataEncoders.header.frame_id, "base_link");
		
		dataEncoders.twist.twist.linear.x = linear_velocity_x;
		dataEncoders.twist.twist.angular.z = angular_velocity_z;

		RCSOFTCHECK(rcl_publish(&publisher_IMU, dataIMU, NULL));
		RCSOFTCHECK(rcl_publish(&publisher_encoder, &dataEncoders, NULL));
		rclc_executor_spin_some(&executor,0);
		vTaskDelay(pdMS_TO_TICKS(DELAY_TIME));
	}
}

/**
 * @brief Task to control the speed of the engines from the twist received 
 * @param argument Pointer to the task arguments (not used)
*/
void TaskMotorControl(void *argument){
	ReceivedTwist twist;
	AngularVelocityWheels angular_velocity_wheels;
	
	PWM_config();
	PID_Init();
	
	for(;;){
		if(xQueueReceive(TwistReceivedQueue, &twist, 0) == pdTRUE){
			angular_velocity_wheels = convertTwistToAngularVelocity(twist);
		}
		set_angular_velocity(angular_velocity_wheels);	
		vTaskDelay(pdMS_TO_TICKS(DELAY_TIME));
	}
}

/**
 * @brief Callback function that receives the twist message from the nav2
 */
void twist_callback(const void * msgin){
	ReceivedTwist twist;
	const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
	twist.v_x = msg->linear.x;
	twist.w_z = msg->angular.z;
	if(twist.w_z > -2.5 && twist.w_z < 2.5){
		twist.w_z = msg->angular.z * 2.5;
	}

	if (xQueueSend(TwistReceivedQueue, &twist, 0) != pdPASS){
       	printf("ERROR: full queue.\n");
    }
}

/**
 * @brief Convert the twist message to angular velocity
 * @param twist 
 * @return AngularVelocityWheels 
 */
AngularVelocityWheels convertTwistToAngularVelocity(ReceivedTwist twist){
	AngularVelocityWheels angular_velocity_wheels;
	angular_velocity_wheels.w_l = (twist.v_x - twist.w_z * WHEEL_SEPARATION / 2) / RADIUS_WHEEL;
	angular_velocity_wheels.w_r = (twist.v_x + twist.w_z * WHEEL_SEPARATION / 2) / RADIUS_WHEEL;
	return angular_velocity_wheels;
}


void unix_time_callback(const void * msgin){
	const builtin_interfaces__msg__Time *msg = (const builtin_interfaces__msg__Time *)msgin;
    seconds = msg->sec;
	nanoseconds = msg->nanosec;
}