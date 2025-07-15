# I2C Motor Controller & Sensor Manager

This project provides a flexible I2C communication system with two main components:
1. **Flexible I2C Master Manager** - Handles multiple I2C devices (sensors, custom devices)
2. **Motor Controller Master Wrapper** - Simple API for motor controller communication
3. **Motor Controller Slave** - Runs on the motor controller ESP32

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Main ESP32 (Master)                     │
├─────────────────────┬───────────────────────────────────────┤
│ Your Application   │                                       │
│                    │                                       │
├─────────────────────┼───────────────────────────────────────┤
│ Motor Controller   │        I2C Master Manager            │
│     Wrapper        │                                       │
│ (Simple API)       │  (Flexible Multi-Device Manager)     │
├─────────────────────┴───────────────────────────────────────┤
│                    I2C Bus                                  │
└─────────────────────┬───────────────────────────────────────┘
                      │
         ┌────────────┼────────────┬─────────────────────────┐
         │            │            │                         │
    ┌────▼───┐   ┌────▼───┐   ┌────▼───┐               ┌────▼───┐
    │ Motor  │   │LSM6DS  │   │Custom  │      ...      │Custom  │
    │Control │   │ Sensor │   │Device 1│               │Device N│
    │ Slave  │   │ 0x6A   │   │ 0x50   │               │ 0x5N   │
    │ 0x42   │   └────────┘   └────────┘               └────────┘
    └────────┘
```

## Project Structure

```
your_project/
├── components/
│   ├── i2c_master_manager/              ← Core flexible I2C manager
│   │   ├── Kconfig.projbuild            ← I2C pins, speed, sensor configs
│   │   ├── CMakeLists.txt
│   │   ├── include/i2c_master_manager.h
│   │   └── i2c_master_manager.c
│   ├── i2c_motorcontroller_master/      ← Simple motor controller wrapper
│   │   ├── Kconfig.projbuild            ← Motor controller address & wake pin
│   │   ├── CMakeLists.txt
│   │   ├── include/i2c_motorcontroller_master.h
│   │   └── i2c_motorcontroller_master.c
│   ├── i2c_motorcontroller_slave/       ← Motor controller slave
│   │   ├── Kconfig.projbuild            ← Slave I2C config
│   │   ├── CMakeLists.txt
│   │   ├── include/i2c_motorcontroller_slave.h
│   │   └── i2c_motorcontroller_slave.c
│   └── buoye_datatypes/                 ← Your existing structs & serialization
│       ├── buoye_structs.h
│       └── serde_helper.h
├── main/
│   └── main.c                           ← Your application
└── CMakeLists.txt                       ← Project dependencies
```

## Configuration (menuconfig)

### I2C Master Manager Configuration
```
Component config → I2C Master Manager Configuration
├── I2C port number: 0
├── SCL GPIO number: 22
├── SDA GPIO number: 21  
├── I2C clock speed (Hz): 100000
├── LSM6DS032TR Sensor
│   ├── [*] Enable LSM6DS032TR IMU Sensor
│   ├── LSM6DS032TR I2C address: 0x6A
│   └── Sample rate (Hz): 104
└── Custom I2C Devices
    ├── [*] Enable Custom Device 1
    ├── Custom Device 1 Name: "MY_SENSOR"
    ├── Custom Device 1 I2C address: 0x50
    ├── [*] Enable Custom Device 2
    ├── Custom Device 2 Name: "ANOTHER_DEVICE"
    └── Custom Device 2 I2C address: 0x51
```

### Motor Controller Master Configuration
```
Component config → Motor Controller Master Configuration
├── Motor controller I2C address: 0x42
└── Motor controller wake-up pin: 25 (or -1 to disable)
```

### I2C Slave Motor Controller Configuration
```
Component config → I2C Slave Motor Controller Configuration
├── I2C port number: 0
├── SCL GPIO number: 22
├── SDA GPIO number: 21
└── Motor controller slave I2C address: 0x42
```

## Usage Examples

### Option 1: Simple Motor Controller Only

```c
#include "i2c_motorcontroller_master.h"

void app_main() {
    // Initialize motor controller
    ESP_ERROR_CHECK(i2c_motctrl_master_init());
    
    // Wake up slave
    i2c_motctrl_master_wake_up_motorcontroller();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Create and send package
    motorcontroller_pkg_t pkg;
    motorcontroller_pkg_init_default(&pkg);
    pkg.STATE = LOWERING;
    pkg.end_depth = 100;
    
    ESP_ERROR_CHECK(i2c_motctrl_master_send_pkg(&pkg, 10));
    
    // Wait for response
    motorcontroller_response_t resp;
    int wait_time = motctrl_master_calculate_wait_time(
        pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent);
    ESP_ERROR_CHECK(i2c_motctrl_master_wait_response(&resp, wait_time));
    
    // Cleanup
    i2c_motctrl_master_deinit();
}
```

### Option 2: Motor Controller + Sensors

```c
#include "i2c_motorcontroller_master.h"
#include "i2c_master_manager.h"

// Callback for I2C events
void i2c_event_callback(const i2c_master_event_data_t *event, void *user_data) {
    switch (event->event_type) {
        case I2C_MASTER_EVT_OPERATION_COMPLETE:
            ESP_LOGI("APP", "I2C operation completed for device 0x%02X", event->device_addr);
            break;
        case I2C_MASTER_EVT_OPERATION_ERROR:
            ESP_LOGE("APP", "I2C operation failed: %s", esp_err_to_name(event->error_code));
            break;
    }
}

void app_main() {
    // Initialize motor controller (this also initializes the manager)
    ESP_ERROR_CHECK(i2c_motctrl_master_init());
    
    // Motor controller operations (same as Option 1)
    // ... motor controller code ...
    
    // Read from LSM6DS032TR sensor
    uint8_t who_am_i;
    ESP_ERROR_CHECK(i2c_master_sensor_read_reg(0x6A, 0x0F, &who_am_i, 1, 1000));
    ESP_LOGI("APP", "LSM6DS032TR WHO_AM_I: 0x%02X", who_am_i);
    
    // Configure LSM6DS032TR
    uint8_t config = 0x60; // 104 Hz, 2g range
    ESP_ERROR_CHECK(i2c_master_sensor_write_reg(0x6A, 0x10, &config, 1, 1000));
    
    // Read accelerometer data
    uint8_t accel_data[6];
    ESP_ERROR_CHECK(i2c_master_sensor_read_reg(0x6A, 0x28, accel_data, 6, 1000));
    
    // Check if custom device is available
    if (i2c_master_device_is_available(0x50)) {
        ESP_LOGI("APP", "Custom device detected at 0x50");
    }
}
```

### Option 3: Direct Manager Use (Maximum Flexibility)

```c
#include "i2c_master_manager.h"

void i2c_event_callback(const i2c_master_event_data_t *event, void *user_data) {
    // Handle all I2C events
}

void app_main() {
    // Initialize manager only
    ESP_ERROR_CHECK(i2c_master_manager_init(i2c_event_callback, NULL));
    
    // Manually add motor controller device
    i2c_device_config_t motctrl_config = {
        .name = "MOTOR_CTRL",
        .address = 0x42,
        .type = I2C_DEVICE_TYPE_MOTOR_CONTROLLER,
        .speed_hz = 100000,
        .enabled = true
    };
    ESP_ERROR_CHECK(i2c_master_add_device(&motctrl_config));
    
    // Add custom sensor not in Kconfig
    i2c_device_config_t custom_sensor = {
        .name = "PRESSURE_SENSOR",
        .address = 0x77,
        .type = I2C_DEVICE_TYPE_CUSTOM,
        .speed_hz = 100000,
        .enabled = true
    };
    ESP_ERROR_CHECK(i2c_master_add_device(&custom_sensor));
    
    // Use sensor functions
    uint8_t sensor_data[4];
    ESP_ERROR_CHECK(i2c_master_sensor_read_reg(0x77, 0xF4, sensor_data, 4, 1000));
}
```

## Motor Controller Slave Usage

```c
#include "i2c_motorcontroller_slave.h"

void app_main() {
    // Initialize slave
    ESP_ERROR_CHECK(i2c_innstall_slave_driver_cnfig());
    
    while (true) {
        // Wait for package from master
        motorcontroller_pkg_t pkg;
        esp_err_t err = i2c_motctrl_slave_wait_pkg(&pkg, 30);
        
        if (err == ESP_ERR_TIMEOUT) {
            continue; // Keep waiting
        } else if (err != ESP_OK) {
            ESP_LOGE("SLAVE", "Package receive error: %s", esp_err_to_name(err));
            continue;
        }
        
        ESP_LOGI("SLAVE", "Received package - depth: %d", pkg.end_depth);
        
        // Set working state
        i2c_motctrl_slave_set_working();
        
        // Simulate motor work
        int work_time_ms = 5000; // 5 seconds
        vTaskDelay(pdMS_TO_TICKS(work_time_ms));
        
        // Prepare response
        motorcontroller_response_t resp;
        motorcontroller_response_init_default(&resp);
        resp.STATE = pkg.STATE;
        resp.result = ESP_OK;
        resp.working_time = work_time_ms / 1000;
        resp.estimated_cm_per_s = 5000; // 5.0 cm/s
        
        // Send response with retry
        for (int i = 0; i < 5; i++) {
            err = i2c_motctrl_slave_send_response(&resp, 15);
            if (err == ESP_OK) {
                ESP_LOGI("SLAVE", "Response sent successfully");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}
```

## Component Dependencies

### Your main CMakeLists.txt:
```cmake
# For Option 1 (Motor Controller Only)
set(EXTRA_COMPONENT_DIRS components)
target_link_libraries(${COMPONENT_LIB} INTERFACE i2c_motorcontroller_master buoye_datatypes)

# For Option 2 & 3 (Motor Controller + Sensors)  
set(EXTRA_COMPONENT_DIRS components)
target_link_libraries(${COMPONENT_LIB} INTERFACE i2c_motorcontroller_master i2c_master_manager buoye_datatypes)
```

### Component Dependencies:
- **i2c_master_manager**: Independent, only depends on ESP-IDF drivers
- **i2c_motorcontroller_master**: Depends on `i2c_master_manager` and `buoye_datatypes`
- **i2c_motorcontroller_slave**: Independent, depends on `buoye_datatypes`

## Adding New Sensors

### Via Kconfig (Recommended):
1. Edit `components/i2c_master_manager/Kconfig.projbuild`
2. Add new sensor section:
```kconfig
menu "BMP280 Pressure Sensor"
    config BMP280_DEVICE_ENABLED
        bool "Enable BMP280 Pressure Sensor"
        default n
        
    config BMP280_I2C_ADDR
        hex "BMP280 I2C address"
        range 0x76 0x77
        default 0x77
        depends on BMP280_DEVICE_ENABLED
endmenu
```

3. Add to auto-configuration in `i2c_master_manager.c`:
```c
#ifdef CONFIG_BMP280_DEVICE_ENABLED
    i2c_device_config_t bmp280_config = {
        .name = "BMP280",
        .address = CONFIG_BMP280_I2C_ADDR,
        .type = I2C_DEVICE_TYPE_CUSTOM,
        .speed_hz = CONFIG_I2C_MASTER_CLK_SPEED,
        .enabled = true
    };
    i2c_master_add_device(&bmp280_config);
#endif
```

### Programmatically:
```c
i2c_device_config_t my_sensor = {
    .name = "MY_SENSOR",
    .address = 0x48,
    .type = I2C_DEVICE_TYPE_CUSTOM,
    .speed_hz = 100000,
    .enabled = true
};
ESP_ERROR_CHECK(i2c_master_add_device(&my_sensor));
```

## Troubleshooting

### Common Issues:

1. **"Device not found" errors**:
   - Check wiring and pull-up resistors
   - Verify I2C addresses in menuconfig
   - Use `i2c_master_device_is_available()` to scan

2. **Motor controller communication fails**:
   - Ensure both master and slave use same I2C address
   - Check wake-up pin configuration
   - Verify `buoye_datatypes` component is properly included

3. **Compilation errors**:
   - Make sure all components are in `components/` directory
   - Check CMakeLists.txt dependencies
   - Run `idf.py fullclean` and rebuild

4. **Kconfig symbol conflicts**:
   - Each component should have its own Kconfig.projbuild
   - Use unique prefixes (I2C_MASTER_, MOTCTRL_, etc.)

### Debug Commands:
```c
// Scan I2C bus for devices
for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    if (i2c_master_device_is_available(addr)) {
        ESP_LOGI("SCAN", "Device found at 0x%02X", addr);
    }
}

// Check device configuration
const i2c_device_config_t *config = i2c_master_get_device_config(0x42);
if (config) {
    ESP_LOGI("CONFIG", "Device: %s, Type: %d", config->name, config->type);
}
```

## Migration from Old Code

If you have existing motor controller code:

1. **Replace includes**:
   ```c
   // Old
   #include "old_i2c_master.h"
   
   // New  
   #include "i2c_motorcontroller_master.h"
   ```

2. **Update initialization**:
   ```c
   // Old
   i2c_config_t conf = {...};
   i2c_param_config(I2C_NUM_0, &conf);
   i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
   
   // New
   ESP_ERROR_CHECK(i2c_motctrl_master_init());
   ```

3. **Function names remain the same**:
   - `i2c_motctrl_master_send_pkg()`
   - `i2c_motctrl_master_wait_response()`
   - `i2c_motctrl_master_wake_up_motorcontroller()`

## Performance Notes

- **Queue-based operations**: All I2C operations are queued and executed sequentially
- **Non-blocking**: Motor controller operations don't block sensor readings
- **Memory usage**: ~2KB RAM for manager context + device configs
- **Typical timing**: Motor controller operations take 2-10 seconds, sensor reads take <10ms

## Future Enhancements

- [ ] Add interrupt-based sensor data collection
- [ ] Implement sensor-specific driver layers
- [ ] Add I2C error recovery mechanisms
- [ ] Support for multiple I2C buses
- [ ] Add sensor data filtering and processing

---

## Quick Start Checklist

- [ ] Copy components to your project
- [ ] Add component dependencies to main CMakeLists.txt  
- [ ] Configure I2C pins and addresses in menuconfig
- [ ] Enable desired sensors in menuconfig
- [ ] Update your application code
- [ ] Flash and test!

For questions or issues, check the troubleshooting section above.