#include <stdio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <rosidl_runtime_c/string_functions.h>
#include <uros_network_interfaces.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_timer.h"
#include "../inc/utils.h"
#include "../inc/sensors.h"
#include "../inc/motor_control.h"

// Micro-ROS definitions
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define DELAY_TIME (SAMPLING_TIME * 1000) // In milliseconds

rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rmw_init_options_t* rmw_options;
rcl_publisher_t publisher_encoder;
rcl_publisher_t publisher_IMU;
rcl_node_t esp32_node;
rcl_subscription_t subscription_velocities;

mpu6050_handle_t mpu6050 = NULL;

QueueHandle_t IMUQueue;
QueueHandle_t TwistReceivedQueue;

int encoder_pulses_FR = 0; // Cuenta los pulsos del encoder
int encoder_pulses_FL = 0;
int encoder_pulses_RR = 0;
int encoder_pulses_RL = 0;

TickType_t total_ticks;
int ticks_FL;
int ticks_FR;
int ticks_RR;
int ticks_RL;

float current_gyro_z = 0.0;

typedef struct{
    float x, y, z;
  } LinearAccelerationIMU; 

typedef struct{
    float x, y, z;
  } AngularVelocityIMU; 

typedef struct{
    float x,y,z;
} Position;

typedef struct{
  float v_x, w_z;
} ReceivedTwist;

geometry_msgs__msg__Twist twist_msg;

// Functions
void init_microROS();
void FreeRTOS_Init();
void TaskReadDataIMU(void *argument);
void TaskMotorControl(void *argument);
void TaskPublishDataSensors(void *argument);
void twist_callback(const void * msgin);
AngularVelocityWheels convertTwistToAngularVelocity(ReceivedTwist twist);