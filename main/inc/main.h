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


// Micro-ROS definitions
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Functions
void PWM_config();
void isr_handler();
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
void motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, mcpwm_generator_t gen_in1, mcpwm_generator_t gen_in2);
void motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle, mcpwm_generator_t gen_low, mcpwm_generator_t gen_pwm);
void motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle, mcpwm_generator_t gen_low, mcpwm_generator_t gen_pwm);

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

int encoder_pulses = 0; // Cuenta los pulsos del encoder