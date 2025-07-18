#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

// Sensorar (input)
#define GPIO_WINCH_HOME    CONFIG_GPIO_WINCH_HOME
#define GPIO_WINCH_TENSION CONFIG_GPIO_WINCH_TENSION
#define GPIO_WINCH_AUTO    CONFIG_GPIO_WINCH_AUTO

// Aktuatorar (output)
#define GPIO_WINCH_DOWN    CONFIG_GPIO_WINCH_DOWN
#define GPIO_WINCH_UP      CONFIG_GPIO_WINCH_UP

#if CONFIG_USE_WINCH_RUNNING_LAMP
#  define GPIO_LAMP        CONFIG_GPIO_LAMP
#endif

#if CONFIG_USE_AUTO_LAMP
#  define GPIO_AUTO_LAMP   CONFIG_GPIO_AUTO_LAMP
#endif

#if CONFIG_USE_ALARM_LAMP
#  define GPIO_ALARM_LAMP  CONFIG_GPIO_ALARM_LAMP
#endif

// ----- Compile‑time konflikt­sjekk for CAN‑pinnar -----
#ifdef CONFIG_CAN_TX_GPIO
  _Static_assert(CONFIG_CAN_TX_GPIO != GPIO_WINCH_HOME,
                 "GPIO‑konflikt: CAN_TX_GPIO kolliderer med WINCH_HOME");
  _Static_assert(CONFIG_CAN_TX_GPIO != GPIO_WINCH_TENSION,
                 "GPIO‑konflikt: CAN_TX_GPIO kolliderer med WINCH_TENSION");
  _Static_assert(CONFIG_CAN_TX_GPIO != GPIO_WINCH_AUTO,
                 "GPIO‑konflikt: CAN_TX_GPIO kolliderer med WINCH_AUTO");
  _Static_assert(CONFIG_CAN_TX_GPIO != GPIO_WINCH_DOWN,
                 "GPIO‑konflikt: CAN_TX_GPIO kolliderer med WINCH_DOWN");
  _Static_assert(CONFIG_CAN_TX_GPIO != GPIO_WINCH_UP,
                 "GPIO‑konflikt: CAN_TX_GPIO kolliderer med WINCH_UP");
  #if CONFIG_USE_WINCH_RUNNING_LAMP
    _Static_assert(CONFIG_CAN_TX_GPIO != GPIO_LAMP,
                   "GPIO‑konflikt: CAN_TX_GPIO kolliderer med WINCH_RUNNING_LAMP");
  #endif
  #if CONFIG_USE_AUTO_LAMP
    _Static_assert(CONFIG_CAN_TX_GPIO != GPIO_AUTO_LAMP,
                   "GPIO‑konflikt: CAN_TX_GPIO kolliderer med AUTO_LAMP");
  #endif
  #if CONFIG_USE_ALARM_LAMP
    _Static_assert(CONFIG_CAN_TX_GPIO != GPIO_ALARM_LAMP,
                   "GPIO‑konflikt: CAN_TX_GPIO kolliderer med ALARM_LAMP");
  #endif
#endif

#ifdef CONFIG_CAN_RX_GPIO
  _Static_assert(CONFIG_CAN_RX_GPIO != GPIO_WINCH_HOME,
                 "GPIO‑konflikt: CAN_RX_GPIO kolliderer med WINCH_HOME");
  _Static_assert(CONFIG_CAN_RX_GPIO != GPIO_WINCH_TENSION,
                 "GPIO‑konflikt: CAN_RX_GPIO kolliderer med WINCH_TENSION");
  _Static_assert(CONFIG_CAN_RX_GPIO != GPIO_WINCH_AUTO,
                 "GPIO‑konflikt: CAN_RX_GPIO kolliderer med WINCH_AUTO");
  _Static_assert(CONFIG_CAN_RX_GPIO != GPIO_WINCH_DOWN,
                 "GPIO‑konflikt: CAN_RX_GPIO kolliderer med WINCH_DOWN");
  _Static_assert(CONFIG_CAN_RX_GPIO != GPIO_WINCH_UP,
                 "GPIO‑konflikt: CAN_RX_GPIO kolliderer med WINCH_UP");
  #if CONFIG_USE_WINCH_RUNNING_LAMP
    _Static_assert(CONFIG_CAN_RX_GPIO != GPIO_LAMP,
                   "GPIO‑konflikt: CAN_RX_GPIO kolliderer med WINCH_RUNNING_LAMP");
  #endif
  #if CONFIG_USE_AUTO_LAMP
    _Static_assert(CONFIG_CAN_RX_GPIO != GPIO_AUTO_LAMP,
                   "GPIO‑konflikt: CAN_RX_GPIO kolliderer med AUTO_LAMP");
  #endif
  #if CONFIG_USE_ALARM_LAMP
    _Static_assert(CONFIG_CAN_RX_GPIO != GPIO_ALARM_LAMP,
                   "GPIO‑konflikt: CAN_RX_GPIO kolliderer med ALARM_LAMP");
  #endif
#endif

#endif // GPIO_CONFIG_H
