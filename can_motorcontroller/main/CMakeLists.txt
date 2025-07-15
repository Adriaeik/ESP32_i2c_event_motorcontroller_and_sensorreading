# main/CMakeLists.txt - Main Application

set(MAIN_SRCS 
    "main.c"
     # Test functions (avoids component circular dependencies)
)

# Base requirements that are always needed
set(MAIN_REQUIRES
    can_motctrl      # Common utilities
    freertos
    nvs_flash
    esp_system
)

# Add conditional requirements based on what might be included
# Note: Since we use #define in main.c, we need to include all possible components
# ESP-IDF will only link what's actually used
list(APPEND MAIN_REQUIRES
    can_motctrl_manager    # For manager role
    can_motctrl_worker     # For worker role
)

# Register main component
idf_component_register(
    SRCS ${MAIN_SRCS}
    INCLUDE_DIRS "include"
    REQUIRES ${MAIN_REQUIRES}
)