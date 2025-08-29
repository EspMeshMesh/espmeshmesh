#pragma once

#include <functional>
#include <cstdarg>

#define LIB_LOG_LEVEL LIB_LOG_LEVEL_DEBUG

#define LIB_LOG_LEVEL_NONE 0
#define LIB_LOG_LEVEL_ERROR 1
#define LIB_LOG_LEVEL_WARN 2
#define LIB_LOG_LEVEL_INFO 3
#define LIB_LOG_LEVEL_CONFIG 4
#define LIB_LOG_LEVEL_DEBUG 5
#define LIB_LOG_LEVEL_VERBOSE 6
#define LIB_LOG_LEVEL_VERY_VERBOSE 7

#define LIB_LOGE(tag, format, ...) libLog(LIB_LOG_LEVEL_ERROR, tag, __LINE__, format, ##__VA_ARGS__)
#define LIB_LOGW(tag, format, ...) libLog(LIB_LOG_LEVEL_WARN, tag, __LINE__, format, ##__VA_ARGS__)
#define LIB_LOGI(tag, format, ...) libLog(LIB_LOG_LEVEL_INFO, tag, __LINE__, format, ##__VA_ARGS__)
#define LIB_LOGCONFIG(tag, format, ...) libLog(LIB_LOG_LEVEL_CONFIG, tag, __LINE__, format, ##__VA_ARGS__)

#if LIB_LOG_LEVEL >= LIB_LOG_LEVEL_VERY_VERBOSE
#define LIB_LOGVV(tag, format, ...) libLog(LIB_LOG_LEVEL_VERY_VERBOSE, tag, __LINE__, format, ##__VA_ARGS__)
#else
#define LIB_LOGVV(tag, format, ...)
#endif

#if LIB_LOG_LEVEL >= LIB_LOG_LEVEL_VERBOSE
#define LIB_LOGV(tag, format, ...) libLog(LIB_LOG_LEVEL_VERBOSE, tag, __LINE__, format, ##__VA_ARGS__)
#else
#define LIB_LOGV(tag, format, ...)
#endif

#if LIB_LOG_LEVEL >= LIB_LOG_LEVEL_DEBUG
#define LIB_LOGD(tag, format, ...) libLog(LIB_LOG_LEVEL_DEBUG, tag, __LINE__, format, ##__VA_ARGS__)
#else
#define LIB_LOGD(tag, format, ...)
#endif

namespace espmeshmesh {
    typedef std::function<void(int level, const char *tag, int line, const char *format, va_list args)> LogCbFn;
    void setLibLogCb(LogCbFn cb);
    void libLog(int level, const char *tag, int line, const char *format, ...);
}
