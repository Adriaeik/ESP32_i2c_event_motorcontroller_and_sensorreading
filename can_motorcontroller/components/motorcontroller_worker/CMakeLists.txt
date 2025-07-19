
idf_component_register(
    SRCS "state_winch_temp.c" "state_winch_temp.c" "motorcontroller_worker.c"
         "inputs.c"
         "outputs.c"
         "timer.c"

    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_event esp_timer heap buoye_datatypes esp_system log_thread can_motctrl can_motctrl_worker can_bus_manager can_motctrl_manager can_motctrl_worker
    PRIV_REQUIRES esp_system
)
