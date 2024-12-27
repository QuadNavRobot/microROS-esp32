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
		&publisher_encoder,
		&esp32_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
		"odom"));

	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	
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
	EulerAngle euler;
	Quaternion quaternion;
	Position position;
	uint32_t seconds, nanoseconds;
	AngularVelocityIMU angular_velocity_IMU;
	LinearAccelerationIMU linear_acceleration_IMU;
	nav_msgs__msg__Odometry odom_msg;

	rosidl_runtime_c__String__init(&odom_msg.header.frame_id);
	rosidl_runtime_c__String__init(&odom_msg.child_frame_id);

	while(1){
		get_current_time(&seconds, &nanoseconds);

		if(xQueueReceive(IMUQueue, values, portMAX_DELAY) == pdTRUE){ //Get data from the queue
			linear_acceleration_IMU.x = convertGToMS2(values[0]);
			linear_acceleration_IMU.y = convertGToMS2(values[1]);
			linear_acceleration_IMU.z = convertGToMS2(values[2]);
			angular_velocity_IMU.x = convertDegreesToRadians(values[3]);
			angular_velocity_IMU.y = convertDegreesToRadians(values[4]);
			angular_velocity_IMU.z = convertDegreesToRadians(values[5]);
			euler.pitch = 0.0;
			euler.roll = 0.0;
			euler.yaw += angular_velocity_IMU.z * SAMPLING_TIME; 
			quaternion = euler_to_quaternion(euler);
		}

		memset(&odom_msg, 0, sizeof(nav_msgs__msg__Odometry));
		odom_msg.header.stamp.sec = seconds;
		odom_msg.header.stamp.nanosec = nanoseconds;
		rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
		rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
		position.x += current_velocity_total * SAMPLING_TIME * cos(euler.yaw); // se calcula cada vez que se llama a set_angular_velocity en TaskMotorControl 
		position.y += current_velocity_total * SAMPLING_TIME * sin(euler.yaw);
		position.z = 0.0;
		odom_msg.pose.pose.position.x = position.x;
		odom_msg.pose.pose.position.y = position.y;
		odom_msg.pose.pose.position.z = position.z;
		odom_msg.pose.pose.orientation.x = quaternion.x;
		odom_msg.pose.pose.orientation.y = quaternion.y;
		odom_msg.pose.pose.orientation.z = quaternion.z;
		odom_msg.pose.pose.orientation.w = quaternion.w;
		odom_msg.twist.twist.linear.x = current_velocity_total * cos(euler.yaw);
		odom_msg.twist.twist.linear.y = current_velocity_total * sin(euler.yaw);
		odom_msg.twist.twist.linear.z = 0.0;
		odom_msg.twist.twist.angular.x = 0.0;
		odom_msg.twist.twist.angular.y = 0.0;
		odom_msg.twist.twist.angular.z = angular_velocity_IMU.z;
		memset(&odom_msg.pose.covariance, 0, sizeof(odom_msg.pose.covariance));
		memset(&odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));

		RCSOFTCHECK(rcl_publish(&publisher_encoder, &odom_msg, NULL));
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