#include <stdio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <uros_network_interfaces.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/spi_slave.h"
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
#define DEBUG_MODE 1

// SPI pins
#define GPIO_MOSI 12  // 38 vision bonnet
#define GPIO_SCLK 15  // 40 vision bonnet
#define GPIO_CS 14    // 12 vision bonnet


// Variables
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rmw_init_options_t* rmw_options;
rcl_publisher_t publisher_encoder;
rcl_publisher_t publisher_IMU;
rcl_node_t esp32_node;

mpu6050_handle_t mpu6050 = NULL;

QueueHandle_t IMUQueue;

int encoder_pulses_FR = 0; // Cuenta los pulsos del encoder
int encoder_pulses_FL = 0;
int encoder_pulses_RR = 0;
int encoder_pulses_RL = 0;

// Functions
void init_microROS();
void FreeRTOS_Init();
void TaskPublishDataSensors(void *argument);
void TaskReadDataIMU(void *argument);
void TaskPWM(void *argument);
//void TaskSPI(void *argument);