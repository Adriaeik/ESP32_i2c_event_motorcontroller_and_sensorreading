idf_component_register(
    SRCS "i2c_motorcontroller_master.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_event buoye_datatypes i2c_master_manager
    PRIV_REQUIRES esp_system
)
