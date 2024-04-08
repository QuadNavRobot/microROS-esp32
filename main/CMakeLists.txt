idf_component_register(SRCS 
                        src/main.c
                        src/mpu6050.c  
                        src/utils.c
                        src/motor_control.c
                        src/sensors.c
                        src/compl_filter_library.c
                    INCLUDE_DIRS "inc"
                    )