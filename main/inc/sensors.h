#include  "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "../inc/mpu6050.h"
#include "driver/i2c.h"
#include "unity.h"

#define GPIO_ENCODER_FR 34 // front right wheel
#define GPIO_ENCODER_FL 35 // front left wheel
#define GPIO_ENCODER_RR 32 // rear right wheel
#define GPIO_ENCODER_RL 33 // rear left wheel

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

extern int encoder_pulses_FL;
extern int encoder_pulses_RR;
extern int encoder_pulses_FR; 
extern int encoder_pulses_RL;

extern mpu6050_handle_t mpu6050;

void init_encoder();
void IRAM_ATTR encoder_isr_handler(void* arg);
esp_err_t i2c_Init();
mpu6050_handle_t i2c_sensor_mpu6050_init(void);
