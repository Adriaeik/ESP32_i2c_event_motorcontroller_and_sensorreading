#include "buoye_structs.h"
#include "sdkconfig.h"
#include "log_thread.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

// === sheared_config_t ===
esp_err_t sheared_config_copy(sheared_config_t *dst, const sheared_config_t *src) {
    if (!dst || !src) return ESP_ERR_INVALID_ARG;
    dst->periodic_time_sec          = src->periodic_time_sec;
    dst->periodic_time_lowering_sec = src->periodic_time_lowering_sec;
    dst->periodic_time_rising_sec   = src->periodic_time_rising_sec;
    dst->service_flag               = src->service_flag;
    dst->service_time               = src->service_time;
    dst->poll_config                = src->poll_config; // direkte kopi
    return ESP_OK;
}


// === buoye_config_t ===
esp_err_t buoye_config_overwrite(buoye_config_t *dst, const buoye_config_t *src) {
    if (!dst || !src) return ESP_ERR_INVALID_ARG;
    dst->current_time_to_start_work = src->current_time_to_start_work;
    dst->time_to_start_next_work    = src->time_to_start_next_work;
    return sheared_config_copy(&dst->sheared_config, &src->sheared_config);
}


void fill_sorted_static_points(const char* input, uint16_t* output_array, size_t max_len) {
    if (!input || !output_array || max_len == 0) return;

    // Midlertidig buffer for uparsa verdiar
    uint16_t temp[MAX_POINTS];
    size_t count = 0;

    // Dupliser strengen fordi strtok øydelegg den
    char buf[max_len+1]; // for tryggleik, maks lengde bør vere definert
    strncpy(buf, input, sizeof(buf));
    buf[sizeof(buf) - 1] = '\0';

    // Parse strengen
    char* token = strtok(buf, ",");
    while (token && count < max_len) {
        int val = atoi(token);
        if (val >= 0 && val <= 0xFFFF) {
            temp[count++] = (uint16_t)val;
        }
        token = strtok(NULL, ",");
    }

    // Sorter stigande (boble-sort, ok sidan datasettet er lite)
    for (size_t i = 0; i + 1 < count; ++i) {
        for (size_t j = 0; j + 1 < count - i; ++j) {
            if (temp[j] > temp[j + 1]) {
                uint16_t tmp = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = tmp;
            }
        }
    }

    // Kopier sortert til output_array
    for (size_t i = 0; i < max_len; ++i) {
        output_array[i] = (i < count) ? temp[i] : 0;
    }
}

poll_config_t poll_config_default(void) {
    poll_config_t config = {
        .poll_type = CONFIG_DEFAULT_POLL_TYPE,
        .timeout = CONFIG_DEFAULT_TIMEOUT,
        .start_depth = CONFIG_DEFAULT_START_DEPTH,
        .end_depth = CONFIG_DEFAULT_END_DEPTH,
        .samples = CONFIG_DEFAULT_SAMPLE_COUNT,
        .alpha_u8 = 100, // centerd
        .static_poll_s = CONFIG_DEFAULT_STATIC_POLL_PERIOD
    };
    // memset(config.static_points, 0, sizeof(config.static_points));
     fill_sorted_static_points(CONFIG_DEFAULT_STATIC_POINTS, config.static_points, MAX_POINTS);
    return config;
}

sheared_config_t sheared_config_default(void) {
    sheared_config_t config = {
        .periodic_time_sec = CONFIG_DEFAULT_PERIODIC_TIME_SEC,
        .periodic_time_lowering_sec = CONFIG_DEFAULT_PERIODIC_TIME_SEC,
        .periodic_time_rising_sec = CONFIG_DEFAULT_PERIODIC_TIME_SEC,
    #ifdef CONFIG_DEFAULT_SERVICE_FLAG
        .service_flag = true,
    #else
        .service_flag = false,
    #endif
        .service_time = CONFIG_DEFAULT_SERVICE_TIME,
        .poll_config = poll_config_default()
    };
    return config;
}


buoye_config_t buoye_config_default(void) {
    buoye_config_t config = {
        .sheared_config = sheared_config_default(),
        .current_time_to_start_work = 0,
        .time_to_start_next_work = 0
    };
    return config;
}

sheared_status_t sheared_status_default(void) {
    sheared_status_t status = {
        .battery_level = 100,
        .clock_error_s = 0,
        .estimated_depth = 0,
        .time_to_start_cycle = 0
    };
    return status;
}

void motorcontroller_pkg_init_default(motorcontroller_pkg_t *pkg)
{
    if (pkg == NULL) {
        return;
    }
    
    memset(pkg, 0, sizeof(motorcontroller_pkg_t));
    
    pkg->STATE = LOWERING;
    pkg->rising_timeout_percent = 30;
    pkg->estimated_cm_per_s_x1000 = 50000;  // Default 50 cm/s - make config
    pkg->poll_type = CONFIG_DEFAULT_POLL_TYPE;
    pkg->end_depth = CONFIG_DEFAULT_END_DEPTH;
    pkg->samples = CONFIG_DEFAULT_SAMPLE_COUNT;
    pkg->static_poll_interval_s = CONFIG_DEFAULT_STATIC_POLL_PERIOD;
    pkg->prev_reported_depth = CONFIG_DEFAULT_END_DEPTH;
    pkg->prev_working_time = CONFIG_DEFAULT_END_DEPTH/pkg->estimated_cm_per_s_x1000/1000;
    // Initialize static points array (null-terminated)
    memset(pkg->static_points, 0, sizeof(pkg->static_points));
}

void motorcontroller_response_init_default(motorcontroller_response_t *resp)
{
    if (resp == NULL) {
        return;
    }
    
    memset(resp, 0, sizeof(motorcontroller_response_t));
    
    resp->STATE = LOWERING;
    resp->result = ESP_OK;
    resp->working_time = 0;
    resp->estimated_cm_per_s = 50;  // Default estimate
}


const char* state_to_string(state_t s) {
    switch (s) {
        case LOWERING: return "LOWERING";
        case RISING:   return "RISING";
        case INIT:     return "INIT";
        default:       return "UNKNOWN";
    }
}

const char* poll_to_string(POLL_TYPE s){
    switch (s) {
        case ALPHA_DEPTH: return "ALPHA_DEPTH";
        case STATIC_DEPTH:   return "STATIC_DEPTH";
        case LIN_TIME:     return "LIN_TIME";
        default:       return "UNKNOWN";
    }
}

/// MODBUS - AQUA TROLL -  SPESIFIC DATA
#if CONFIG_USE_TROLL_DATA

// === sensor_batch (valfri, for komplettheit) ===
esp_err_t sensor_batch_copy(SensorBatch *dst, const SensorBatch *src) {
    if (!dst || !src) return ESP_ERR_INVALID_ARG;
    *dst = *src; // OK sidan det er packed struct og fast array
    return ESP_OK;
}
// kan vurdere å legge til noko #if CONFIG_* her
const char* sensorID_to_string(SensorID id) {
    switch (id) {
        case TEMP: return "Temperature";
        case PRESSURE: return "Pressure";
        case DEPTH: return "Depth";
        case LEVEL_DEPTH: return "Level (Depth)";
        case LEVEL_SURFACE: return "Level (Surface)";
        case CONDUCTIVITY_ACTUAL: return "Conductivity (Actual)";
        case CONDUCTIVITY_SPECIFIC: return "Conductivity (Specific)";
        case RESISTIVITY: return "Resistivity";
        case SALINITY: return "Salinity";
        case TOTAL_DISSOLVED_SOLIDS: return "Total Dissolved Solids";
        case WATER_DENSITY: return "Water Density";
        case BAROMETRIC_PRESSURE: return "Barometric Pressure";
        case PH: return "pH";
        case PH_MV: return "pH (mV)";
        case ORP: return "ORP";
        case DISSOLVED_OXYGEN_CONC: return "Dissolved Oxygen (Conc)";
        case DISSOLVED_OXYGEN_SAT: return "Dissolved Oxygen (Saturation)";
        case CHLORIDE: return "Chloride";
        case TURBIDITY: return "Turbidity";
        case OXYGEN_PARTIAL_PRESSURE: return "Oxygen Partial Pressure";
        case TOTAL_SUSPENDED_SOLIDS: return "Total Suspended Solids";
        case EXTERNAL_VOLTAGE: return "External Voltage";
        case BATTERY_CAPACITY: return "Battery Capacity";
        case RHODAMINE_CONC: return "Rhodamine (Conc)";
        case RHODAMINE_FLUO: return "Rhodamine (Fluorescence)";
        case CHLORIDE_MV: return "Chloride (mV)";
        case NITRATE_CONC: return "Nitrate (Conc)";
        case NITRATE_MV: return "Nitrate (mV)";
        case AMMONIUM_CONC: return "Ammonium (Conc)";
        case AMMONIUM_MV: return "Ammonium (mV)";
        case AMMONIA_CONC: return "Ammonia (Conc)";
        case TOTAL_AMMONIA: return "Total Ammonia";
        case EH: return "Eh";
        case VELOCITY: return "Velocity";
        case CHLOROPHYLL_CONC: return "Chlorophyll (Conc)";
        case CHLOROPHYLL_FLUO: return "Chlorophyll (Fluorescence)";
        case CHLOROPHYLL_A_CONC: return "Chlorophyll-a (Conc)";
        case CHLOROPHYLL_A_FLUO: return "Chlorophyll-a (Fluorescence)";
        case BLUE_GREEN_ALGAE_PHYCOCYANIN: return "Blue-Green Algae (Phycocyanin)";
        case BLUE_GREEN_ALGAE_FLUO: return "Blue-Green Algae (Fluorescence)";
        case BLUE_GREEN_ALGAE_PHYCOERYTHRIN: return "Blue-Green Algae (Phycoerythrin)";
        case BLUE_GREEN_ALGAE_PHYCOERYTHRIN_FLUO: return "Blue-Green Algae (Phycoerythrin, Fluorescence)";
        case FLUORESCEIN_CONC: return "Fluorescein (Conc)";
        case FLUORESCEIN_FLUO: return "Fluorescein (Fluorescence)";
        case DISSOLVED_ORGANIC_MATTER: return "Dissolved Organic Matter";
        case DISSOLVED_ORGANIC_MATTER_FLUO: return "Dissolved Organic Matter (Fluorescence)";
        case CRUDE_OIL_CONC: return "Crude Oil (Conc)";
        case CRUDE_OIL_FLUO: return "Crude Oil (Fluorescence)";
        case COLORED_DISSOLVED_ORGANIC_MATTER: return "Colored Dissolved Organic Matter";
        case SENSOR_END: return "End Marker";
        default: return "Unknown Sensor";
    }
}


/*__________________UNIT_________________*/
Unit unit_id_to_enum(uint16_t unit_id) {
  switch (unit_id) {
      // Temperatur
      case 1: return UNIT_CELSIUS;
      case 2: return UNIT_FAHRENHEIT;
      case 3: return UNIT_KELVIN;

      // Trykk
      case 17: return UNIT_PSI;
      case 18: return UNIT_PASCAL;
      case 19: return UNIT_KILOPASCAL;
      case 20: return UNIT_BAR;
      case 21: return UNIT_MILLIBAR;
      case 22: return UNIT_MMHG;
      case 23: return UNIT_INHG;
      case 24: return UNIT_CMH2O;
      case 25: return UNIT_INH2O;
      case 26: return UNIT_TORR;
      case 27: return UNIT_ATM;

      // Lengde
      case 33: return UNIT_MILLIMETER;
      case 34: return UNIT_CENTIMETER;
      case 35: return UNIT_METER;
      case 36: return UNIT_KILOMETER;
      case 37: return UNIT_INCH;
      case 38: return UNIT_FEET;

      // Koordinatar
      case 49: return UNIT_DEGREE;
      case 50: return UNIT_MINUTE;
      case 51: return UNIT_SECOND;

      // Konduktivitet
      case 65: return UNIT_MICROSIEMENS_PER_CM;
      case 66: return UNIT_MILLISIEMENS_PER_CM;

      // Resistivitet
      case 81: return UNIT_OHM_CM;

      // Salthaldighet
      case 97: return UNIT_PSU;
      case 98: return UNIT_PPT_SALINITY;

      // Konsentrasjon
      case 113: return UNIT_PPM;
      case 114: return UNIT_PPT_CONCENTRATION;
      case 117: return UNIT_MG_PER_L;
      case 118: return UNIT_UG_PER_L;
      case 120: return UNIT_G_PER_L;
      case 121: return UNIT_PPB;

      // Tettleik
      case 129: return UNIT_G_PER_CM3;

      // pH
      case 145: return UNIT_PH;

      // Spenning
      case 161: return UNIT_MICROVOLT;
      case 162: return UNIT_MILLIVOLT;
      case 163: return UNIT_VOLT;

      // DO metting
      case 177: return UNIT_PERCENT_SATURATION;

      // Turbiditet
      case 193: return UNIT_FNU;
      case 194: return UNIT_NTU;
      case 195: return UNIT_FTU;

      // Flow
      case 209: return UNIT_CUBIC_FEET_PER_SECOND;
      case 212: return UNIT_CUBIC_FEET_PER_DAY;
      case 213: return UNIT_GALLONS_PER_SECOND;
      case 214: return UNIT_GALLONS_PER_MINUTE;
      case 215: return UNIT_GALLONS_PER_HOUR;
      case 216: return UNIT_MILLIONS_GALLONS_PER_DAY;
      case 217: return UNIT_CUBIC_METERS_PER_SECOND;
      case 219: return UNIT_CUBIC_METERS_PER_HOUR;
      case 221: return UNIT_LITERS_PER_SECOND;
      case 222: return UNIT_MILLIONS_LITERS_PER_DAY;
      case 223: return UNIT_MILLILITERS_PER_MINUTE;
      case 224: return UNIT_THOUSANDS_LITERS_PER_DAY;

      // Volum
      case 225: return UNIT_CUBIC_FEET;
      case 226: return UNIT_GALLON;
      case 227: return UNIT_MILLIONS_GALLONS;
      case 228: return UNIT_CUBIC_METER;
      case 229: return UNIT_LITER;
      case 230: return UNIT_ACRE_FEET;
      case 231: return UNIT_MILLILITER;
      case 232: return UNIT_MILLIONS_LITERS;
      case 233: return UNIT_THOUSANDS_LITERS;
      case 234: return UNIT_ACRE_INCHES;

      // Prosent
      case 241: return UNIT_PERCENT;

      // Fluorescens
      case 257: return UNIT_RFU;

      // Lavflow
      case 273: return UNIT_MILLILITERS_PER_SECOND;
      case 274: return UNIT_MILLILITERS_PER_HOUR;
      case 275: return UNIT_LITERS_PER_MINUTE;
      case 276: return UNIT_LITERS_PER_HOUR;

      // Strøm
      case 289: return UNIT_MICROAMPERE;
      case 290: return UNIT_MILLIAMPERE;
      case 291: return UNIT_AMPERE;

      // Fart
      case 305: return UNIT_FEET_PER_SECOND;
      case 306: return UNIT_METERS_PER_SECOND;

      default: return UNIT_UNKNOWN;
  }
}
uint16_t unit_enum_to_id(Unit unit) {
  return (unit == UNIT_UNKNOWN) ? 0xFFFF : (uint16_t)unit;
}

const char* unit_to_string(Unit unit) {
  switch (unit) {
      // Temperatur
      case UNIT_CELSIUS: return "Celsius";
      case UNIT_FAHRENHEIT: return "Fahrenheit";
      case UNIT_KELVIN: return "Kelvin";

      // Trykk
      case UNIT_PSI: return "psi";
      case UNIT_PASCAL: return "Pa";
      case UNIT_KILOPASCAL: return "kPa";
      case UNIT_BAR: return "bar";
      case UNIT_MILLIBAR: return "mbar";
      case UNIT_MMHG: return "mmHg";
      case UNIT_INHG: return "inHg";
      case UNIT_CMH2O: return "cmH2O";
      case UNIT_INH2O: return "inH2O";
      case UNIT_TORR: return "Torr";
      case UNIT_ATM: return "atm";

      // Lengde
      case UNIT_MILLIMETER: return "mm";
      case UNIT_CENTIMETER: return "cm";
      case UNIT_METER: return "m";
      case UNIT_KILOMETER: return "km";
      case UNIT_INCH: return "in";
      case UNIT_FEET: return "ft";

      // Koordinatar
      case UNIT_DEGREE: return "°";
      case UNIT_MINUTE: return "′";
      case UNIT_SECOND: return "″";

      // Konduktivitet
      case UNIT_MICROSIEMENS_PER_CM: return "µS/cm";
      case UNIT_MILLISIEMENS_PER_CM: return "mS/cm";

      // Resistivitet
      case UNIT_OHM_CM: return "Ω·cm";

      // Salthaldighet
      case UNIT_PSU: return "PSU";
      case UNIT_PPT_SALINITY: return "ppt (salinity)";

      // Konsentrasjon
      case UNIT_PPM: return "ppm";
      case UNIT_PPT_CONCENTRATION: return "ppt (concentration)";
      case UNIT_MG_PER_L: return "mg/L";
      case UNIT_UG_PER_L: return "µg/L";
      case UNIT_G_PER_L: return "g/L";
      case UNIT_PPB: return "ppb";

      // Tettleik
      case UNIT_G_PER_CM3: return "g/cm³";

      // pH
      case UNIT_PH: return "pH";

      // Spenning
      case UNIT_MICROVOLT: return "µV";
      case UNIT_MILLIVOLT: return "mV";
      case UNIT_VOLT: return "V";

      // DO metting
      case UNIT_PERCENT_SATURATION: return "% sat";

      // Turbiditet
      case UNIT_FNU: return "FNU";
      case UNIT_NTU: return "NTU";
      case UNIT_FTU: return "FTU";

      // Flow
      case UNIT_CUBIC_FEET_PER_SECOND: return "ft³/s";
      case UNIT_CUBIC_FEET_PER_DAY: return "ft³/day";
      case UNIT_GALLONS_PER_SECOND: return "gal/s";
      case UNIT_GALLONS_PER_MINUTE: return "gal/min";
      case UNIT_GALLONS_PER_HOUR: return "gal/h";
      case UNIT_MILLIONS_GALLONS_PER_DAY: return "MGD";
      case UNIT_CUBIC_METERS_PER_SECOND: return "m³/s";
      case UNIT_CUBIC_METERS_PER_HOUR: return "m³/h";
      case UNIT_LITERS_PER_SECOND: return "L/s";
      case UNIT_MILLIONS_LITERS_PER_DAY: return "MLD";
      case UNIT_MILLILITERS_PER_MINUTE: return "mL/min";
      case UNIT_THOUSANDS_LITERS_PER_DAY: return "kL/day";

      // Volum
      case UNIT_CUBIC_FEET: return "ft³";
      case UNIT_GALLON: return "gal";
      case UNIT_MILLIONS_GALLONS: return "Mgal";
      case UNIT_CUBIC_METER: return "m³";
      case UNIT_LITER: return "L";
      case UNIT_ACRE_FEET: return "acre·ft";
      case UNIT_MILLILITER: return "mL";
      case UNIT_MILLIONS_LITERS: return "ML";
      case UNIT_THOUSANDS_LITERS: return "kL";
      case UNIT_ACRE_INCHES: return "acre·in";

      // Prosent
      case UNIT_PERCENT: return "%";

      // Fluorescens
      case UNIT_RFU: return "RFU";

      // Lavflow
      case UNIT_MILLILITERS_PER_SECOND: return "mL/s";
      case UNIT_MILLILITERS_PER_HOUR: return "mL/h";
      case UNIT_LITERS_PER_MINUTE: return "L/min";
      case UNIT_LITERS_PER_HOUR: return "L/h";

      // Strøm
      case UNIT_MICROAMPERE: return "µA";
      case UNIT_MILLIAMPERE: return "mA";
      case UNIT_AMPERE: return "A";

      // Fart
      case UNIT_FEET_PER_SECOND: return "ft/s";
      case UNIT_METERS_PER_SECOND: return "m/s";

      case UNIT_UNKNOWN:
      default:
          return "Unknown";
  }
}

#endif