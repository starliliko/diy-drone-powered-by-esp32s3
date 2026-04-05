idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash qmc5883p i2c_bus
                       LDFRAGMENTS linker_fragment.lf)
