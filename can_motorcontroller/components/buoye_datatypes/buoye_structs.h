#pragma once
#include <time.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sdkconfig.h"


typedef enum {
    INIT = 0,
    LOWERING = 1,
    RISING = 2,
} state_t;
const char* state_to_string(state_t s);

typedef enum{
    ALPHA_DEPTH = 0, // polynom funksjon
    STATIC_DEPTH, // do n pols at m depths (this needs to include n and m)
    LIN_TIME, // for debug, we can use time since the depth wont chande on land..
}POLL_TYPE;
const char* poll_to_string(POLL_TYPE s);


#define MAX_POINTS 80
typedef struct {
    POLL_TYPE poll_type;
    time_t timeout; // if this is reached we need to finnish our polling task!
    uint16_t start_depth; //in cm
    uint16_t end_depth; //in cm
    uint16_t static_points[MAX_POINTS]; // in cm [10, 100, 165... 0(end), 0, ...,0]cm // stop at 0
    uint16_t samples; // if poll_type == STATIC_DEPTH how many sampels to take at each depth.
                    // else how many samples to be take between start and end_depth
    uint8_t alpha_u8; // uint8 rep of a float between -1.0 and 1.0, where 100 = 0.0 and then 0 = -1.0 and 200 = 1.0. giving 2 points of reselution
    uint16_t static_poll_s; // time in sec betwen each static sample
} poll_config_t;

void fill_sorted_static_points(const char* input, uint16_t* output_array, size_t max_len);
poll_config_t poll_config_default(void);

/**
 * @brief Dinna deler vi under mqtt-tasksa. Den oppdateres fra config topicen
 * Eller brker default/ forrige sheared_config_t
 */
typedef struct {
    int periodic_time_sec; 
    int periodic_time_lowering_sec; 
    int periodic_time_rising_sec; 
    poll_config_t poll_config; // 
    bool service_flag; //if we want to do service we set this to true, given the service_time is set to a date and time where the sensor stops
    time_t service_time; //if we want to do service we set this to a date and time where the sensor stops, given the service flag is true
} sheared_config_t;

sheared_config_t sheared_config_default(void);
esp_err_t sheared_config_copy(sheared_config_t *dst, const sheared_config_t *src);

typedef struct {
    sheared_config_t sheared_config;
    time_t current_time_to_start_work; // saves last time we started work WHEN we start work, so we can rollback if anny mqtt fail
    time_t time_to_start_next_work; //current_time_to_start_work + periode  
} buoye_config_t;

buoye_config_t buoye_config_default(void);
esp_err_t buoye_config_overwrite(buoye_config_t *dst, const buoye_config_t *src);

typedef struct {
    int battery_level; // 0-100%
    int clock_error_s ; // 0-100%
    int estimated_depth; // cm - the depth we think we went down.
    time_t time_to_start_cycle; // time_t
} sheared_status_t;

sheared_status_t sheared_status_default(void);

/// FOR MOTORCONTROLLER
// MOTORCONTROLLER package: sent from master to controller to initiate lowering or raising
// old
// typedef struct {
//     state_t STATE;                   // current operation mode: LOWERING or RISING
//     int prev_working_time;          // duration (seconds) of last operation, used for estimating next time
//     int rising_timeout_percent;     // factor to define timeout threshold for rising if home sensor not seen
//     uint16_t prev_reported_depth;   // last measured depth (cm) from the sensor
//     uint16_t prev_end_depth;        // previous target depth (cm) what we tried to rach. if we did correct this == prev_reported_depth
//     uint16_t prev_estimated_cm_per_s;// last estimated lowering speed in cm/s (scaled e.g., 10000 => 10.000 cm/s)
//     POLL_TYPE poll_type;            // type of polling (e.g., STATIC_DEPTH or CONTINUOUS)
//     uint16_t end_depth;             // target depth for this operation (cm) remember to set this to the max point of static_points if poll_type = static
//     uint16_t static_points[MAX_POINTS]; // depths at which to pause and sample (cm), terminated by 0 
//     uint16_t samples;               // sample count at each static point
//     uint16_t static_poll_interval_s; // interval between static samples (seconds)

//     // added for filter operation and timing
//     double alpha;                   // alpha gain for alpha-beta filter
//     double beta;                    // [UNUSED] beta gain for alpha-beta filter
// } motorcontroller_pkg_t;

//new
typedef struct {
    state_t STATE;                      // current operation mode: LOWERING or RISING
    int rising_timeout_percent;         // factor to define timeout threshold for rising if home sensor not seen
    uint16_t estimated_cm_per_s_x1000;        // estimated lowering speed in cm/s (scaled e.g., 10000 => 10.000 cm/s)
    POLL_TYPE poll_type;                // type of polling (e.g., STATIC_DEPTH or CONTINUOUS)
    uint16_t end_depth;                 // target depth for this operation (cm) remember to set this to the max point of static_points if poll_type = static
    uint16_t static_points[MAX_POINTS]; // depths at which to pause and sample (cm), terminated by 0 
    uint16_t samples;                   // if STATIC_DEPTH: sample count at each static point(how many sampels to take at each static point)
    uint16_t static_poll_interval_s;    // if STATIC_DEPTH: interval between static samples (seconds). if LIN_TIME: (if lowering: how long to run. if rising: this*rising_timeout_percent is the timeout before we have to hit home, else its timout error)
    int prev_working_time;          // duration (seconds) of last operation, used for estimating next time
    uint16_t prev_reported_depth;   // last measured depth (cm) from the sensor
} motorcontroller_pkg_t;


void motorcontroller_pkg_init_default(motorcontroller_pkg_t *pkg);
// MOTORCONTROLLER response package: sent back when finished
typedef struct {
    state_t STATE;                  // operation mode performed
    esp_err_t result;               // result code: ESP_OK, ESP_ERR_TIMEOUT, TENSION_ALARM, etc.
    int working_time;               // total time (seconds) taken for the elevation
    uint16_t estimated_cm_per_s;    // new speed estimate (scaled)
} motorcontroller_response_t;


void motorcontroller_response_init_default(motorcontroller_response_t *resp);
/*
// Inputs: measurement depth z_meas (cm), dt in seconds.
// State: x (predicted depth), v (velocity estimate in cm/s)
// Gains: alpha, beta

void alpha_beta_update(double dt, double z_meas,
                       double *x, double *v,
                       double alpha, double beta)
{
    // prediction step
    double x_pred = *x + (*v) * dt;
    double v_pred = *v;

    // residual
    double r = z_meas - x_pred;

    // correction
    *x = x_pred + alpha * r;
    *v = v_pred + (beta * r) / dt;
}
*/


// MODBUS SPESIFIC
#if CONFIG_USE_TROLL_DATA
/**
* @brief Enum for sensorar definert i Appendix A av Aqua TROLL 500-manualen
*/
typedef enum {
   TEMP = 1,
   PRESSURE = 2,
   DEPTH = 3,
   LEVEL_DEPTH = 4,
   LEVEL_SURFACE = 5,
   CONDUCTIVITY_ACTUAL = 9,
   CONDUCTIVITY_SPECIFIC = 10,
   RESISTIVITY = 11,
   SALINITY = 12,
   TOTAL_DISSOLVED_SOLIDS = 13,
   WATER_DENSITY = 14,
   BAROMETRIC_PRESSURE = 16,
   PH = 17,
   PH_MV = 18,
   ORP = 19,
   DISSOLVED_OXYGEN_CONC = 20,
   DISSOLVED_OXYGEN_SAT = 21,
   CHLORIDE = 24,
   TURBIDITY = 25,
   OXYGEN_PARTIAL_PRESSURE = 30,
   TOTAL_SUSPENDED_SOLIDS = 31,
   EXTERNAL_VOLTAGE = 32,
   BATTERY_CAPACITY = 33,
   RHODAMINE_CONC = 34,
   RHODAMINE_FLUO = 35,
   CHLORIDE_MV = 36,
   NITRATE_CONC = 37,
   NITRATE_MV = 38,
   AMMONIUM_CONC = 39,
   AMMONIUM_MV = 40,
   AMMONIA_CONC = 41,
   TOTAL_AMMONIA = 42,
   EH = 48,
   VELOCITY = 49,
   CHLOROPHYLL_CONC = 50,
   CHLOROPHYLL_FLUO = 51,
   CHLOROPHYLL_A_CONC = 52,
   CHLOROPHYLL_A_FLUO = 53,
   BLUE_GREEN_ALGAE_PHYCOCYANIN = 54,
   BLUE_GREEN_ALGAE_FLUO = 55,
   BLUE_GREEN_ALGAE_PHYCOERYTHRIN = 58,
   BLUE_GREEN_ALGAE_PHYCOERYTHRIN_FLUO = 59,
   FLUORESCEIN_CONC = 67,
   FLUORESCEIN_FLUO = 68,
   DISSOLVED_ORGANIC_MATTER = 69,
   DISSOLVED_ORGANIC_MATTER_FLUO = 70,
   CRUDE_OIL_CONC = 80,
   CRUDE_OIL_FLUO = 81,
   COLORED_DISSOLVED_ORGANIC_MATTER = 87,
   SENSOR_END = 0,
} SensorID;

const char* sensorID_to_string(SensorID id);

typedef enum {
    // Temperatur (1–3)
    UNIT_CELSIUS = 1,
    UNIT_FAHRENHEIT = 2,
    UNIT_KELVIN = 3,

    // Trykk (17–27)
    UNIT_PSI = 17,
    UNIT_PASCAL = 18,
    UNIT_KILOPASCAL = 19,
    UNIT_BAR = 20,
    UNIT_MILLIBAR = 21,
    UNIT_MMHG = 22,
    UNIT_INHG = 23,
    UNIT_CMH2O = 24,
    UNIT_INH2O = 25,
    UNIT_TORR = 26,
    UNIT_ATM = 27,

    // Lengde (33–38)
    UNIT_MILLIMETER = 33,
    UNIT_CENTIMETER = 34,
    UNIT_METER = 35,
    UNIT_KILOMETER = 36,
    UNIT_INCH = 37,
    UNIT_FEET = 38,

    // Koordinater (49–51)
    UNIT_DEGREE = 49,
    UNIT_MINUTE = 50,
    UNIT_SECOND = 51,

    // Konduktivitet (65–66)
    UNIT_MICROSIEMENS_PER_CM = 65,
    UNIT_MILLISIEMENS_PER_CM = 66,

    // Resistivitet (81)
    UNIT_OHM_CM = 81,

    // Saltholdighet (97–98)
    UNIT_PSU = 97,
    UNIT_PPT_SALINITY = 98,

    // Konsentrasjon (113–121)
    UNIT_PPM = 113,
    UNIT_PPT_CONCENTRATION = 114,
    UNIT_MG_PER_L = 117,
    UNIT_UG_PER_L = 118,
    UNIT_G_PER_L = 120,
    UNIT_PPB = 121,

    // Tetthet (129)
    UNIT_G_PER_CM3 = 129,

    // pH (145)
    UNIT_PH = 145,

    // Spenning (161–163)
    UNIT_MICROVOLT = 161,
    UNIT_MILLIVOLT = 162,
    UNIT_VOLT = 163,

    // Oksygenmetning (177)
    UNIT_PERCENT_SATURATION = 177,

    // Turbiditet (193–195)
    UNIT_FNU = 193,
    UNIT_NTU = 194,
    UNIT_FTU = 195,

    // Strømning (209–224)
    UNIT_CUBIC_FEET_PER_SECOND = 209,
    UNIT_CUBIC_FEET_PER_DAY = 212,
    UNIT_GALLONS_PER_SECOND = 213,
    UNIT_GALLONS_PER_MINUTE = 214,
    UNIT_GALLONS_PER_HOUR = 215,
    UNIT_MILLIONS_GALLONS_PER_DAY = 216,
    UNIT_CUBIC_METERS_PER_SECOND = 217,
    UNIT_CUBIC_METERS_PER_HOUR = 219,
    UNIT_LITERS_PER_SECOND = 221,
    UNIT_MILLIONS_LITERS_PER_DAY = 222,
    UNIT_MILLILITERS_PER_MINUTE = 223,
    UNIT_THOUSANDS_LITERS_PER_DAY = 224,

    // Volum (225–234)
    UNIT_CUBIC_FEET = 225,
    UNIT_GALLON = 226,
    UNIT_MILLIONS_GALLONS = 227,
    UNIT_CUBIC_METER = 228,
    UNIT_LITER = 229,
    UNIT_ACRE_FEET = 230,
    UNIT_MILLILITER = 231,
    UNIT_MILLIONS_LITERS = 232,
    UNIT_THOUSANDS_LITERS = 233,
    UNIT_ACRE_INCHES = 234,

    // Prosent (241)
    UNIT_PERCENT = 241,

    // Fluorescens (257)
    UNIT_RFU = 257,

    // Lavstrøm (273–276)
    UNIT_MILLILITERS_PER_SECOND = 273,
    UNIT_MILLILITERS_PER_HOUR = 274,
    UNIT_LITERS_PER_MINUTE = 275,
    UNIT_LITERS_PER_HOUR = 276,

    // Strøm (289–291)
    UNIT_MICROAMPERE = 289,
    UNIT_MILLIAMPERE = 290,
    UNIT_AMPERE = 291,

    // Hastighet (305–306)
    UNIT_FEET_PER_SECOND = 305,
    UNIT_METERS_PER_SECOND = 306,

    UNIT_UNKNOWN = 0xFFFF
} Unit;

Unit unit_id_to_enum(uint16_t unit_id);
uint16_t unit_enum_to_id(Unit unit);
const char* unit_to_string(Unit unit);

#define SENSOR_BATCH_MAX_COUNT 20
/**
 * @brief Struktur for éin sensoravlesing. 
 */
typedef struct __attribute__((packed)) {
    SensorID sensor_id;      ///< SensorID (enum-verdi)
    float value;            ///< Målt sensorverdi
    uint16_t quality;       ///< Kvalitet på måling (0-100%)
    Unit unit;              ///< Kontoler at målinmga er som forventa!
} SensorReading; //

/**
 * @brief Struktur for batch med sensoravlesingar.
 */
typedef struct __attribute__((packed)) {
    state_t STATE;
    uint16_t count;                    ///< Tal på målingar i denne batchen
    time_t timestamp;       ///< Tidspunkt i s sidan 1970
    float depth;            ///< Gjeldande dybde ved måling
    SensorReading readings[SENSOR_BATCH_MAX_COUNT]; ///< Sensorlesingar
} SensorBatch;



#endif