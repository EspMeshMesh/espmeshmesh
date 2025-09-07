#include "defines.h"

#ifdef IDF_VER
#include <esp_timer.h>
#include <esp_random.h>
#endif

#ifdef ESP8266
#include <osapi.h>
#endif

#ifdef IDF_VER
uint32_t millis() { 
    return (uint32_t) (esp_timer_get_time() / 1000ULL); 
}
#endif

uint32_t random_uint32() {
#ifdef ESP8266
    return os_random();
#endif
#ifdef IDF_VER
    return esp_random();
#endif
}

