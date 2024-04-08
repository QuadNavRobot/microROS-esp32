#include "../inc/sensors.h"

//---------------------- ENCODERS

/**
 * @brief Configure GPIO pins to receive encoder interrupts.
 * @return esp_err_t 
 */
void init_encoder(){

	gpio_config_t gpio_config_struct = {
		.pin_bit_mask = ((1ULL << GPIO_ENCODER_FR) | (1ULL << GPIO_ENCODER_FL) | (1ULL << GPIO_ENCODER_RR) | (1ULL << GPIO_ENCODER_RL)),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type = GPIO_INTR_POSEDGE
	};

	gpio_config(&gpio_config_struct);
	
	gpio_install_isr_service(0);

	gpio_isr_handler_add(GPIO_ENCODER_FR, encoder_isr_handler, (void*)GPIO_ENCODER_FR);
	gpio_isr_handler_add(GPIO_ENCODER_FL, encoder_isr_handler, (void*)GPIO_ENCODER_FL);
	gpio_isr_handler_add(GPIO_ENCODER_RR, encoder_isr_handler, (void*)GPIO_ENCODER_RR);
	gpio_isr_handler_add(GPIO_ENCODER_RL, encoder_isr_handler, (void*)GPIO_ENCODER_RL);

    //return ESP_OK;
}

/**
 * @brief ISR of encoder interrupts
 */
void IRAM_ATTR encoder_isr_handler(void* arg){
	uint32_t gpio_num = (uint32_t) arg;
	
	if(gpio_num == GPIO_ENCODER_FL){
		encoder_pulses_FL++;
	}else if(gpio_num == GPIO_ENCODER_FR){
		encoder_pulses_FR++;
	}else if(gpio_num == GPIO_ENCODER_RR){
		encoder_pulses_RR++;
	}else{
		encoder_pulses_RL++;
	}
}

//---------------------- IMU

/**
 * @brief Initialize i2c and MPU6050.
 */
mpu6050_handle_t i2c_sensor_mpu6050_init(void){
    esp_err_t ret;

    i2c_Init();
    mpu6050 = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    return mpu6050;
}

/**
 * @brief Configure I2C for MPU6050
 * @return esp_err_t 
 */
esp_err_t i2c_Init(){

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = I2C_MASTER_FREQ_HZ, 
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));

    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,0,0, 0)); 
    
    return ESP_OK;
}