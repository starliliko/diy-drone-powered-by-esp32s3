idf_component_register(
    SRCS 
        "bmi088.c"
        "bmi088_spi.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES
        freertos
        esp_common
        log
        spi_bus
    PRIV_REQUIRES
        hal
        driver
)

# 添加编译定义
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    BMI088_SPI_DRIVER=1
)