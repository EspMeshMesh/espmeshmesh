#ifdef IDF_VER
#include "wifi_esp32.h"
#include "log.h"

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_rom_md5.h>
#include <cstring>

static const char *TAG = "Wifi";

namespace espmeshmesh {

WifiEsp32::WifiEsp32(EspMeshMesh *mesh): Wifi(mesh) {
}

void WifiEsp32::setup(std::string hostname, uint8_t channel, uint8_t txPower) {
  Wifi::setup(hostname, channel, txPower);
  LIB_LOGI(TAG, "Setting up meshmesh wifi...");
  if(!setupWifiStation()) {
    LIB_LOGE(TAG, "Failed to setup wifi station");
    return;
  }
  LIB_LOGD(TAG, "Wifi succesful!!!!");
}  

bool WifiEsp32::setupWifiAp() {
  esp_err_t res;
  wifi_config_t wcfg;
  strcpy((char *) wcfg.ap.ssid, "esphome");
  strcpy((char *) wcfg.ap.password, "esphome");
  wcfg.ap.ssid_len = 0;
  wcfg.ap.channel = mChannel;

  wcfg.ap.authmode = WIFI_AUTH_OPEN;
  wcfg.ap.ssid_hidden = 1;
  wcfg.ap.max_connection = 4;
  wcfg.ap.beacon_interval = 60000;
  esp_netif_t *netif;
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  const wifi_promiscuous_filter_t filt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA};
  res = esp_netif_init();
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_netif_init error %d", res);
    return false;
  }

  res = esp_event_loop_create_default();
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_event_loop_create_default error %d", res);
    return false;
  }

  netif = esp_netif_create_default_wifi_ap();
  if (!netif) {
    LIB_LOGE(TAG, "%s wifi ap creation failed: %s", __func__, esp_err_to_name(res));
    return false;
  }

  res = esp_wifi_init(&cfg);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_init error %d", res);
    return false;
  }

  res = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_storage error %s", esp_err_to_name(res));
    return false;
  }

  res = esp_event_handler_register(
    ESP_EVENT_ANY_BASE, 
    ESP_EVENT_ANY_ID, 
    [](void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) { LIB_LOGD(TAG, "wifi_event_handler %ld", event_id); }, 
    nullptr);

  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_event_handler_instance_register error %d", res);
    return false;
  }

  res = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_storage error %d", res);
    return false;
  }

  res = esp_wifi_set_mode(WIFI_MODE_AP);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_mode error %d", res);
    return false;
  }

  wifiInitMacAddr(ESP_IF_WIFI_AP);

  res = esp_wifi_set_config(WIFI_IF_AP, &wcfg);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_config error %d", res);
    return false;
  }

  LIB_LOGI(TAG, "Selected channel %d", wcfg.ap.channel);

  LIB_LOGD(TAG, "esp_wifi_set_protocol");
  res = esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_protocol error %d", res);
    return false;
  }

  LIB_LOGD(TAG, "Start!!!");
  res = esp_wifi_start();
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_start error %d", res);
    return false;
  }

  LIB_LOGD(TAG, "esp_wifi_set_promiscuous");
  res = esp_wifi_set_promiscuous(true);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_promiscuous error %d", res);
    return false;
  }
  LIB_LOGD(TAG, "esp_wifi_set_promiscuous_filter");
  res = esp_wifi_set_promiscuous_filter(&filt);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_promiscuous_filter error %d", res);
    return false;
  }

  LIB_LOGD(TAG, "esp_wifi_set_max_tx_power");
  res = esp_wifi_set_max_tx_power(84);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_max_tx_power error %d", res);
    return false;
  }

  LIB_LOGD(TAG, "esp_wifi_set_ps");
  res = esp_wifi_set_ps(WIFI_PS_NONE);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_ps error %d", res);
    return false;
  }

  return true;
}

bool WifiEsp32::setupWifiStation() {
  esp_err_t res;

  esp_netif_t *netif;
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  const wifi_promiscuous_filter_t filt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA};
  res = esp_netif_init();
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_netif_init error %d", res);
    return false;
  }

  res = esp_event_loop_create_default();
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_event_loop_create_default error %d", res);
    return false;
  }

  netif = esp_netif_create_default_wifi_sta();
  if (!netif) {
    LIB_LOGE(TAG, "%s wifi ap creation failed: %s", __func__, esp_err_to_name(res));
    return false;
  }

  res = esp_wifi_init(&cfg);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_init error %d", res);
    return false;
  }

  res = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_storage error %s", esp_err_to_name(res));
    return false;
  }

  res = esp_event_handler_register(
    ESP_EVENT_ANY_BASE, 
    ESP_EVENT_ANY_ID, 
    [](void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) { LIB_LOGD(TAG, "wifi_event_handler %ld", event_id); }, 
    nullptr);

  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_event_handler_instance_register error %d", res);
    return false;
  }

  res = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_storage error %d", res);
    return false;
  }

  res = esp_wifi_set_mode(WIFI_MODE_STA);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_mode error %d", res);
    return false;
  }

  wifiInitMacAddr(WIFI_IF_STA);

  LIB_LOGD(TAG, "esp_wifi_set_protocol");
  res = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_protocol error %d", res);
    return false;
  }

  LIB_LOGD(TAG, "Start!!!");
  res = esp_wifi_start();
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_start error %d", res);
    return false;
  }

  LIB_LOGD(TAG, "esp_wifi_set_promiscuous");
  res = esp_wifi_set_promiscuous(true);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_promiscuous error %d", res);
    return false;
  }

  LIB_LOGD(TAG, "esp_wifi_set_promiscuous_filter");
  res = esp_wifi_set_promiscuous_filter(&filt);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_promiscuous_filter error %d", res);
    return false;
  }

  LIB_LOGI(TAG, "Selected channel %d", mChannel);
  res = esp_wifi_set_channel(mChannel, WIFI_SECOND_CHAN_NONE);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_channel error %d", res);
    return false;
  }

  LIB_LOGD(TAG, "esp_wifi_set_max_tx_power");
  res = esp_wifi_set_max_tx_power(84);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_max_tx_power error %d", res);
    return false;
  }

  LIB_LOGD(TAG, "esp_wifi_set_ps");
  res = esp_wifi_set_ps(WIFI_PS_NONE);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_ps error %d", res);
    return false;
  }

  return true;
}

std::string WifiEsp32::encryptPassword(std::string password) {
  uint8_t aespassword[16];
  md5_context_t md5;
  esp_rom_md5_init(&md5);
  esp_rom_md5_update(&md5, (uint8_t *) password.c_str(), password.size());
  esp_rom_md5_final(aespassword, &md5);
  return std::string((char *) aespassword, 16);
}

void WifiEsp32::wifiInitMacAddr(uint8_t index) {
  uint32_t id = Discovery::chipId();
  uint8_t *idptr = (uint8_t *) &id;
  uint8_t mac[6] = {0};
  mac[0] = 0xFE;
  mac[1] = 0x7F;
  mac[2] = idptr[3];
  mac[3] = idptr[2];
  mac[4] = idptr[1];
  mac[5] = idptr[0];

  LIB_LOGD(TAG, "wifiInitMacAddr %02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

  esp_err_t res = esp_wifi_set_mac((wifi_interface_t) index, mac);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_mac error %d", res);
  }
}

}  // namespace espmeshmesh
#endif