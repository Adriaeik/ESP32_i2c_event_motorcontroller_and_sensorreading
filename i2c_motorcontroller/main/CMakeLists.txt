idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    
    REQUIRES 
        buoye_datatypes
        i2c_master_manager
        i2c_motorcontroller_master
        i2c_motorcontroller_slave
        driver
        esp_timer
        log
        
    PRIV_REQUIRES 
)