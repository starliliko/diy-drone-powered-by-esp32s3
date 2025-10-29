idf_component_register(SRCS "jy901_i2c_driver.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES i2c_bus
                       PRIV_REQUIRES crazyflie platform)
