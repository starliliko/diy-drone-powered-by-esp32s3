idf_component_register(SRCS "remote_server.c" "wifi_esp32.c"
                      INCLUDE_DIRS "." "include"
                      REQUIRES crazyflie platform config esp_wifi)