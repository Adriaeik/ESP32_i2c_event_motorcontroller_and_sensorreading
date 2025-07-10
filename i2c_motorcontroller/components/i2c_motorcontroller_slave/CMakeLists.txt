
idf_component_register(
    SRCS "i2c_motorcontroller_slave.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_event buoye_datatypes
    PRIV_REQUIRES esp_system
)