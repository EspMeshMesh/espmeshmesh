#include "defines.h"

#ifdef IDF_VER
#include <esp_timer.h>
#include <esp_random.h>
#endif

#ifdef ESP8266
#include <osapi.h>
#endif

#ifdef USE_LINUX
#include <malloc.h>
#endif

namespace espmeshmesh {

#ifdef IDF_VER
uint32_t millis() {
    return (uint32_t) (esp_timer_get_time() / 1000ULL);
}
#endif

#ifdef USE_LINUX
uint32_t millis() {
    return (uint32_t) (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}
#endif

uint32_t random_uint32() {
#ifdef ESP8266
    return os_random();
#endif
#ifdef IDF_VER
    return esp_random();
#endif
#ifdef USE_LINUX
    return static_cast<uint32_t>(random());
#endif
}

uint32_t chipId() {
#ifdef IDF_VER
uint64_t macAddress;
esp_efuse_mac_get_default((uint8_t *) &macAddress);
macAddress = __builtin_bswap64(macAddress);
return (uint32_t) (macAddress >> 16) & 0xFFFFFF;
#endif
#ifdef ESP8266
    return system_get_chip_id();
#endif
#ifdef USE_LINUX
    return 0;
#endif
}


uint32_t hwFreeHeap() {
#ifdef IDF_VER
    return heap_caps_get_free_size(MALLOC_CAP_8BIT);
#endif
#ifdef ESP8266
    return ESP.getFreeHeap();
#endif
#ifdef USE_LINUX
    return 100000;
#endif
}

} // namespace espmeshmesh