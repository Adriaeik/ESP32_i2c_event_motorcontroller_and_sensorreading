idf_component_register(
    SRCS "can_bus_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_event esp_timer heap buoye_datatypes esp_system log_thread buoye_datatypes log_thread
    PRIV_REQUIRES esp_system
)
