#include "log.h"

namespace espmeshmesh {
    LogCbFn mLogCb{nullptr};

    void setLibLogCb(LogCbFn cb) { mLogCb = cb; }

    void __attribute__((hot)) libLog_(int level, const char *tag, int line, const char *format, va_list args) {
        if (mLogCb) {
            mLogCb(level, tag, line, format, args);
        }
    }

    void __attribute__((hot)) libLog(int level, const char *tag, int line, const char *format, ...) {  // NOLINT
        va_list arg;
        va_start(arg, format);
        libLog_(level, tag, line, format, arg);
        va_end(arg);
    }

}