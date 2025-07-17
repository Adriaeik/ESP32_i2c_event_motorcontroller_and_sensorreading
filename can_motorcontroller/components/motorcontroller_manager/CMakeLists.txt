idf_component_register(
    SRCS "motorcontroller_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_event esp_timer heap buoye_datatypes esp_system log_thread can_motctrl can_motctrl_manager
    PRIV_REQUIRES esp_system
)
