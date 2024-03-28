// Includes
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <uros_network_interfaces.h>
#include "unity.h"
#include "esp_log.h"
#include "esp_system.h"
#include  "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "../inc/mpu6050.h"
#include "../inc/i2c_config.h"
#include "driver/mcpwm.h"
#include "driver/spi_slave.h"
#include "driver/ledc.h"

// Micro-ROS definitions
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Functions
void PWM_config();
void FreeRTOS_Init();
void init_microROS();
esp_err_t init_encoder();
void TaskPWM(void *argument);
void TaskEncoder(void *argument);
float convertGToMS2(float value);
void TaskReadDataIMU(void *argument);
void TaskPublishDataIMU(void *argument);
static void i2c_sensor_mpu6050_init(void);
float convertDegreesToRadians(float value);
static void IRAM_ATTR isr_handler(void* arg);
void motor_stop();
void motor_forward(int driver_motor, ledc_channel_t channel, uint32_t dutty);
void motor_backward(int driver_motor, ledc_channel_t channel, uint32_t dutty);
void TaskSPI(void *argument);

// Variables
rcl_allocator_t allocator;
rclc_support_t support;
rcl_init_options_t init_options;
rmw_init_options_t* rmw_options;
rclc_executor_t executor;
rcl_publisher_t publisher_encoder;
rcl_node_t node_encoder;
rcl_publisher_t publisher_IMU;
rcl_node_t node_IMU;

static mpu6050_handle_t mpu6050 = NULL;

QueueHandle_t IMUQueue;

int encoder_pulses_FR = 0; // Cuenta los pulsos del encoder
int encoder_pulses_FL = 0;
int encoder_pulses_RR = 0;
int encoder_pulses_RL = 0;

#define GPIO_MOSI 12  // azul     -> 38 vision bonnet
//#define GPIO_MISO 13  // amarillo -> 35 vision bonnet
#define GPIO_SCLK 15  // verde    -> 40 vision bonnet
#define GPIO_CS 14    // lila     -> 12 vision bonnet

#define GPIO_ENCODER_FR 34 // front right wheel
#define GPIO_ENCODER_FL 35 // front left wheel
#define GPIO_ENCODER_RR 32 // rear right wheel
#define GPIO_ENCODER_RL 33 // rear left wheel

// PWM motor
#define GPIO_IN1_DM1 15
#define GPIO_IN2_DM1 12
#define GPIO_IN1_DM2 27
#define GPIO_IN2_DM2 13

#define GPIO_ENA_M1 4
#define GPIO_ENA_M2 16
#define GPIO_ENA_M3 25
#define GPIO_ENA_M4 26