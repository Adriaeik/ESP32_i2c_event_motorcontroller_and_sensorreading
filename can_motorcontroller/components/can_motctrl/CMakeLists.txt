idf_component_register(
    SRCS "can_motctrl.c" "can_motctrl_common.c" "can_serde_helper.c"
    INCLUDE_DIRS "include"
        REQUIRES driver freertos esp_event esp_timer heap buoye_datatypes esp_system log_thread buoye_datatypes log_thread can_bus_manager
    PRIV_REQUIRES esp_system
)
