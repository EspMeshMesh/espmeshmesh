/**
 * Override ESP-IDF ieee80211_raw_frame_sanity_check so esp_wifi_80211_tx can
 * send MeshMesh custom 802.11 data frames (ToDS=0 / FromDS=0 IBSS-style).
 *
 * ESPHome 2026.6.x recommends ESP-IDF 5.5.x, which enforces this check and
 * aborts (or fails fatally) for frames MeshMesh has always used.
 *
 * Requires linker flag -Wl,-zmuldefs so this definition wins over libnet80211.a.
 */
#if defined(IDF_VER) || defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
#include <stdint.h>

extern "C" {
  int ieee80211_raw_frame_sanity_check(int32_t arg, int32_t arg2, int32_t arg3) {
    (void)arg;
    (void)arg2;
    (void)arg3;
    return 0;
  }
}
#endif
