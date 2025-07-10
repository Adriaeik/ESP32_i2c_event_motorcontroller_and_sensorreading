idf_component_register(SRCS "serde_helper.c" "set_data.c" "RTC_manager.c" "json_helper.c" "RTC_manager.c" "serde_helper.c" "buoye_structs.c"
                       INCLUDE_DIRS "."
                       REQUIRES 
                        esp_timer 
                        json 
                        log_thread
                        esp_hw_support
                        )
