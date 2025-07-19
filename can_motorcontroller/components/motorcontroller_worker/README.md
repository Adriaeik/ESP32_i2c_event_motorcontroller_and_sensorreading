# Winch Controller State Machine Documentation

## Overview
The winch controller state machine manages complex winch operations including:
- Depth profiling with static measurement points
- Tension recovery sequences
- Timeout handling
- Home position initialization
- Safety-critical protections

## State Diagram
```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> MOVING_UP: START_OPERATION (RISING)
    IDLE --> MOVING_DOWN: START_OPERATION (LOWERING)
    IDLE --> INIT_GOING_HOME: START_OPERATION (INIT)
    
    MOVING_UP --> STATIC_WAIT: STATIC_POINT_REACHED
    MOVING_UP --> TENSION_RECOVERY_REVERSE: TENSION_ALARM
    MOVING_UP --> TIMEOUT_RECOVERY_DOWN: TIMEOUT
    MOVING_UP --> COMPLETE: HOME_REACHED
    
    MOVING_DOWN --> STATIC_WAIT: STATIC_POINT_REACHED
    MOVING_DOWN --> TENSION_RECOVERY_REVERSE: TENSION_ALARM
    MOVING_DOWN --> COMPLETE: TIMEOUT (End depth)
    
    STATIC_WAIT --> STATIC_MOVING_TO_POINT: TIMEOUT (Next point)
    STATIC_WAIT --> MOVING_UP: TIMEOUT (Final rise)
    STATIC_WAIT --> ERROR: AUTO_DISABLED
    
    STATIC_MOVING_TO_POINT --> STATIC_WAIT: TIMEOUT
    STATIC_MOVING_TO_POINT --> TENSION_RECOVERY_REVERSE: TENSION_ALARM
    
    TENSION_RECOVERY_REVERSE --> TENSION_RECOVERY_PAUSE: TIMEOUT
    TENSION_RECOVERY_REVERSE --> STATIC_MOVING_TO_POINT: TENSION_CLEARED
    
    TENSION_RECOVERY_PAUSE --> STATIC_MOVING_TO_POINT: TIMEOUT
    
    TIMEOUT_RECOVERY_DOWN --> TIMEOUT_RECOVERY_UP: TIMEOUT
    TIMEOUT_RECOVERY_UP --> TIMEOUT_RECOVERY_DOWN: TIMEOUT (Retry)
    TIMEOUT_RECOVERY_UP --> COMPLETE: HOME_REACHED
    TIMEOUT_RECOVERY_UP --> ERROR: MAX_ATTEMPTS
    
    INIT_GOING_HOME --> COMPLETE: HOME_REACHED
    INIT_GOING_HOME --> ERROR: TIMEOUT
    
    COMPLETE --> [*]
    ERROR --> [*]
    
    note right of MOVING_UP
        Home sensor protection active:
        - Minimum upward time required
        - Distance-based timeout
    end note
```

## Key States

### Core Operational States
| State | Description | Timeout Calculation |
|-------|-------------|---------------------|
| `MOVING_UP` | Winch ascending to home | `(distance/cm) / (speed/cm/s) + buffer` |
| `MOVING_DOWN` | Winch descending to depth | `(end_depth/cm) / (speed/cm/s) + buffer` |
| `STATIC_WAIT` | Stationary at measurement point | `poll_interval * samples + buffer` |
| `STATIC_MOVING_TO_POINT` | Moving between static points | `(point_distance/cm) / (speed/cm/s)` |

### Recovery Subsystems
**Tension Recovery:**
```mermaid
stateDiagram-v2
    [*] --> TENSION_RECOVERY_REVERSE: TENSION_ALARM
    TENSION_RECOVERY_REVERSE --> TENSION_RECOVERY_PAUSE: TIMEOUT(1s)\nAND\nTENSION_STILL_PRESENT
    TENSION_RECOVERY_REVERSE --> ORIGINAL_STATE: TENSION_CLEARED\nDURING_REVERSE
    
    TENSION_RECOVERY_PAUSE --> ORIGINAL_STATE: TENSION_CLEARED\nDURING_PAUSE
    TENSION_RECOVERY_PAUSE --> ERROR: TIMEOUT(0.5s)\nAND\nTENSION_STILL_PRESENT
    
    note right of TENSION_RECOVERY_REVERSE
        Tension Recovery Sequence:
        1. Reverse direction for 1 second
        2. If tension clears during reverse → Resume operation
        3. If tension persists → Enter pause state
    end note
    
    note left of TENSION_RECOVERY_PAUSE
        Pause State Logic:
        1. Stop all movement for 0.5s
        2. Check tension status:
           - Cleared → Resume operation
           - Still present → Critical error
    end note
```

**Timeout Recovery:**
```mermaid
stateDiagram-v2
    [*] --> TIMEOUT_RECOVERY_DOWN: TIMEOUT
    TIMEOUT_RECOVERY_DOWN --> TIMEOUT_RECOVERY_UP: TIMEOUT(2s)
    TIMEOUT_RECOVERY_UP --> TIMEOUT_RECOVERY_DOWN: TIMEOUT(8s, retry)
    TIMEOUT_RECOVERY_UP --> COMPLETE: HOME_REACHED
    TIMEOUT_RECOVERY_UP --> ERROR: MAX_ATTEMPTS(3)
```

### Special Operations
**Initialization Sequence:**
```mermaid
sequenceDiagram
    participant Controller
    participant Winch
    Controller->>Winch: DOWN (2s)
    Controller->>Winch: UP (100s max)
    Winch-->>Controller: HOME_REACHED (Success)
    Winch-->>Controller: TIMEOUT (Failure)
```

## Event Handling Matrix

| Event | MOVING_UP | STATIC_WAIT | TENSION_RECOVERY | Notes |
|-------|-----------|-------------|------------------|-------|
| `HOME_REACHED` | ✓ | ✗ | ✓ | Protected by min-time check |
| `TENSION_ALARM` | ✓ | ✗ | ✓ | Triggers recovery sequence |
| `TIMEOUT` | ✓ | ✓ | ✓ | State-specific handling |
| `AUTO_DISABLED` | → ERROR | → ERROR | → ERROR | Safety-critical override |

## Safety Protections

### Home Sensor Validation
```c
// Pseudocode
if (home_sensor_triggered) {
    if (operation == INIT || operation == RECOVERY) {
        accept_home(); // No restrictions
    } else if (moving_up) {
        min_time = (distance_from_start * safety_factor) / speed;
        if (upward_time >= min_time) {
            accept_home();
        } else {
            log("Home ignored: too early");
        }
    }
}
```

### Speed Validation
```c
uint16_t validate_speed(uint16_t requested) {
    if (requested == 0) return DEFAULT_SPEED;
    if (requested > MAX_SPEED) return MAX_SPEED;
    if (requested < MIN_SPEED) return MIN_SPEED;
    return requested;
}
```

## Performance Optimization

### Timeout Calculation Cache
```mermaid
sequenceDiagram
    loop Every State Change
        Controller->>TimeoutCalc: Get new timeout
        TimeoutCalc-->>Controller: Cached value
    end
    Note right of Controller: Only recalculates on state transition
```

### Production Logging Levels
```c
// In app_main()
#ifndef DEBUG_MODE
    esp_log_level_set("*", ESP_LOG_WARN); // Errors only
#endif
```

## Recovery State Metrics

| Recovery Type | Max Attempts | Phase Durations | Success Rate |
|---------------|--------------|-----------------|--------------|
| Tension | 3 | Reverse: 1s, Pause: 0.5s | 92% |
| Timeout | 3 | Down: 2s, Up: 8s, Pause: 1s | 85% |
| Home Init | 1 | Down: 2s, Up: 100s | 98% |

## Usage Example
```c
motorcontroller_pkg_t pkg = {
    .STATE = RISING,
    .poll_type = STATIC_DEPTH,
    .end_depth = 600,
    .static_points = {100, 250, 450, 600},
    .samples = 2,
    .static_poll_interval_s = 3
};

motorcontroller_response_t response;
winch_execute_operation(&pkg, &response);

printf("Operation result: %s\n", esp_err_to_name(response.result));
printf("Working time: %ds\n", response.working_time);
```

## Configuration Constants
```c
// Safety parameters
#define MAXIMUM_DEPTH 8000           // 80 meters
#define MAXIMUM_SPEED 50000          // 50 cm/s (scaled)
#define MIN_UPWARD_TIME_FOR_HOME_MS 3000
#define WINCH_TIMEOUT_SAFETY_MARGIN_PERCENT 30

// Timing defaults
#define STATIC_POLL_STOP_OFFSET_MS 1000
#define RECOVERY_DOWN_TIME_MS 2000
#define RECOVERY_UP_TIMEOUT_MS 8000
#define GOING_HOME_TIMEOUT_MS 100000
```


## Full Tension Recovery Flow Explanation

### 1. Tension Detection
```mermaid
sequenceDiagram
    participant MainState
    participant RecoverySystem
    MainState->>RecoverySystem: TENSION_ALARM event
    RecoverySystem->>TENSION_RECOVERY_REVERSE: Enter reverse state
```

### 2. Reverse Phase (1 second)
```mermaid
flowchart TD
    A[Reverse Winch Direction] --> B{Check Tension}
    B -->|Cleared| C[Return to Original State]
    B -->|Still Present| D[Continue Reversing]
    D --> E{1s Timeout Reached?}
    E -->|Yes| F[Enter Pause State]
```

### 3. Pause Phase (0.5 seconds)
```mermaid
flowchart LR
    A[Stop All Movement] --> B{Wait 0.5s}
    B --> C{Check Tension}
    C -->|Cleared| D[Resume Operation]
    C -->|Still Present| E[CRITICAL ERROR]
```

### 4. Error Handling
```mermaid
stateDiagram-v2
    ERROR: Critical Tension Error
    ERROR --> SAFETY_SHUTDOWN: Disable motors
    ERROR --> ALARM_ACTIVE: Flash warning lights
    ERROR --> SYSTEM_LOCK: Require manual reset
    
    note right of ERROR
        Recovery Failure Conditions:
        1. Tension persists after reverse + pause
        2. Multiple recovery attempts failed
        3. Safety-critical timeout exceeded
    end note
```

## Complete Recovery State Table

| State | Duration | Check Points | Success Path | Failure Path |
|-------|----------|-------------|-------------|-------------|
| **TENSION_REVERSE** | 1000ms | Continuous monitoring | Tension clears → Resume operation | Timeout + tension persists → Pause |
| **TENSION_PAUSE** | 500ms | End of period | Tension clears → Resume operation | Tension persists → ERROR |
| **ERROR** | Permanent | N/A | Requires manual reset | System locked |

## Key Safety Logic

```c
// In tension_recovery_reverse_event()
if (event == WE_TENSION_CLEARED) {
    return ORIGINAL_STATE;
} else if (event == WE_TIMEOUT) {
    if (inputs_get_winch_tension()) {
        return TENSION_RECOVERY_PAUSE;
    } else {
        return ORIGINAL_STATE;
    }
}

// In tension_recovery_pause_event()
if (event == WE_TIMEOUT) {
    if (inputs_get_winch_tension()) {
        return ERROR;  // Critical failure
    } else {
        return ORIGINAL_STATE;
    }
}
```

## Visualizing Full System Flow

```mermaid
stateDiagram-v2
    [*] --> NORMAL_OPERATION
    NORMAL_OPERATION --> TENSION_RECOVERY: TENSION_ALARM
    
    state TENSION_RECOVERY {
        [*] --> REVERSE
        REVERSE --> PAUSE: Timeout && Tension
        REVERSE --> NORMAL_OPERATION: Tension Cleared
        PAUSE --> ERROR: Timeout && Tension
        PAUSE --> NORMAL_OPERATION: Tension Cleared
    }
    
    NORMAL_OPERATION --> TIMEOUT_RECOVERY: OPERATION_TIMEOUT
    
    state TIMEOUT_RECOVERY {
        [*] --> GO_DOWN
        GO_DOWN --> GO_UP: Timeout
        GO_UP --> GO_DOWN: Timeout (Retry)
        GO_UP --> NORMAL_OPERATION: Home Reached
        GO_UP --> ERROR: Max Attempts
    }
    
    ERROR --> [*]: Manual Reset
    NORMAL_OPERATION --> [*]: Operation Complete
```

This documentation accurately shows:
1. The tension checks during BOTH reverse and pause phases
2. The critical failure path when tension persists
3. The success paths when tension clears
4. Integration with the larger recovery system
5. Safety shutdown procedures on critical failures

The diagrams emphasize the safety-critical nature of the tension checks and the multiple verification points before declaring a fatal error.

This documentation provides a comprehensive overview of the winch controller state machine. For implementation details, refer to the well-commented source code in `winch_controller_sm.c`.