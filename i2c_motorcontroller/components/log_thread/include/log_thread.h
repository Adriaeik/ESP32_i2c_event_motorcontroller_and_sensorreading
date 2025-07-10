#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Slå av/på logging per nivå
#define ENABLE_LOGE_THREAD 1
#define ENABLE_LOGW_THREAD 1
#define ENABLE_LOGI_THREAD 1
#define ENABLE_LOGD_THREAD 1
#define ENABLE_LOGV_THREAD 1
#define ENABLE_LOG_THREAD 1

#define THREAD_PREFIX "{%-20s}|[%-20s] : "

// Intern makro som bruker ESP sitt system fullt ut
#define LOG_WITH_THREAD_X(level, tag, fmt, ...) \
    ESP_LOG_LEVEL_LOCAL(level,  "", THREAD_PREFIX fmt, tag, pcTaskGetName(NULL), ##__VA_ARGS__)

#if ENABLE_LOGI_THREAD
    #define ESP_LOGI_THREAD(tag, fmt, ...) LOG_WITH_THREAD_X(ESP_LOG_INFO, tag, fmt, ##__VA_ARGS__)
#else
    #define ESP_LOGI_THREAD(tag, fmt, ...)
#endif

#if ENABLE_LOGW_THREAD
    #define ESP_LOGW_THREAD(tag, fmt, ...) LOG_WITH_THREAD_X(ESP_LOG_WARN, tag, fmt, ##__VA_ARGS__)
#else
    #define ESP_LOGW_THREAD(tag, fmt, ...)
#endif

#if ENABLE_LOGE_THREAD
    #define ESP_LOGE_THREAD(tag, fmt, ...) LOG_WITH_THREAD_X(ESP_LOG_ERROR, tag, fmt, ##__VA_ARGS__)
#else
    #define ESP_LOGE_THREAD(tag, fmt, ...)
#endif

#if ENABLE_LOGD_THREAD
    #define ESP_LOGD_THREAD(tag, fmt, ...) LOG_WITH_THREAD_X(ESP_LOG_DEBUG, tag, fmt, ##__VA_ARGS__)
#else
    #define ESP_LOGD_THREAD(tag, fmt, ...)
#endif

#if ENABLE_LOGV_THREAD
    #define ESP_LOGV_THREAD(tag, fmt, ...) LOG_WITH_THREAD_X(ESP_LOG_VERBOSE, tag, fmt, ##__VA_ARGS__)
#else
    #define ESP_LOGV_THREAD(tag, fmt, ...)
#endif


#if ENABLE_LOG_THREAD
    #define ESP_LOG_THREAD(tag, fmt, ...) \
        esp_log_write(ESP_LOG_INFO, "", THREAD_PREFIX fmt "\n", tag, pcTaskGetName(NULL), ##__VA_ARGS__)

#else
    #define ESP_LOG_THREAD(tag, fmt, ...)
#endif

/* visstr eg får esp idf sin option til å funke
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Trådnamn-prefiks som du limer inn i meldinga
#define THREAD_FMT "[%-12s]\t"

#define ESP_LOGI_THREAD(tag, fmt, ...) ESP_LOGI(tag, THREAD_FMT fmt, pcTaskGetName(NULL), ##__VA_ARGS__)
#define ESP_LOGW_THREAD(tag, fmt, ...) ESP_LOGW(tag, THREAD_FMT fmt, pcTaskGetName(NULL), ##__VA_ARGS__)
#define ESP_LOGE_THREAD(tag, fmt, ...) ESP_LOGE(tag, THREAD_FMT fmt, pcTaskGetName(NULL), ##__VA_ARGS__)
#define ESP_LOGD_THREAD(tag, fmt, ...) ESP_LOGD(tag, THREAD_FMT fmt, pcTaskGetName(NULL), ##__VA_ARGS__)
#define ESP_LOGV_THREAD(tag, fmt, ...) ESP_LOGV(tag, THREAD_FMT fmt, pcTaskGetName(NULL), ##__VA_ARGS__)

*/