#ifdef IDF_VER
#include "wifidrvesp32.h"
#include "log.h"

#include <esp_heap_caps.h>
#include <esp_wifi.h>
#include <esp_private/wifi.h>
#include <freertos/queue.h>

static const char *TAG = "wifidrv_esp32";

namespace espmeshmesh {

WifiDrvEsp32 *wifiDrvEsp32 = nullptr;

WifiDrvEsp32::WifiDrvEsp32() {
    wifiDrvEsp32 = this;
}

WifiDrvEsp32::~WifiDrvEsp32() {
}

std::string WifiEsp32::encryptPassword(std::string password) {
    uint8_t aespassword[16];
    md5_context_t md5;
    esp_rom_md5_init(&md5);
    esp_rom_md5_update(&md5, (uint8_t *) password.c_str(), password.size());
    esp_rom_md5_final(aespassword, &md5);
    return std::string((char *) aespassword, 16);
}
  
void WifiDrvEsp32::driverSetup() {
    esp_wifi_set_tx_done_cb(this->wifiTxDoneCb);

    esp_err_t ret;
    if((ret = esp_wifi_set_promiscuous_rx_cb(_promiscuousRxq)) != ESP_OK) {
        LIB_LOGE(TAG, "esp_wifi_set_promiscuous_rx_cb error %d", ret);
    }

    mRecvQueue = xQueueCreate(16,sizeof(uint32_t));
}

void WifiDrvEsp32::driverLoop() {
    uint32_t index;
    while(xQueueReceive(mRecvQueue, &index, 0)) {
        recvTask(index);
    }
}

bool WifiDrvEsp32::injectFrame(uint8_t *data, uint16_t len) {
    //LIB_LOGD(TAG, "sendFreedom esp_wifi_80211_tx about to send %d bytes", len80211());
    esp_err_t res = esp_wifi_80211_tx(WIFI_IF_STA, ptr80211(), len80211(), true);
    if(res != ESP_OK) {
        LIB_LOGE(TAG, "sendFreedom esp_wifi_80211_tx err %d", res);
        return false;
    }
    return true;
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

bool WifiDrvEsp32::setupWifiAp() {
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
  
bool WifiDrvEsp32::setupWifiStation() {
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

void WifiDrvEsp32::wifiTxDoneCb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus) {
    //LIB_LOGD(TAG, "wifiTxDoneCb if:%d sent:%d len:%d", ifidx, txStatus, *data_len);
    if(mInjectFrameCallback) {
        mInjectFrameCallback(txStatus?0:1);
    }
}

void WifiDrvEsp32::_wifiTxDoneCb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus) {
    if(wifiDrvEsp32) {
        wifiDrvEsp32->wifiTxDoneCb(ifidx, data, data_len, txStatus);
    }
}

//delete pktbufRecvTaskPacket[index].data;
//memset(&pktbufRecvTaskPacket[index], 0, sizeof(pktbuf_recvTask_packet_t));

void WifiDrvEsp32::promiscuousRxq(wifi_promiscuous_pkt_t *pkt) {
    if(isLockdownModeActive) {
        return;
    }
    
    ieee80211_hdr_p ieee80211_hdr = (ieee80211_hdr_p)pkt->payload;
    
#if 0
    if(ieee80211_hdr->frame_control.Subtype == FRAME_SUBTYPE_DATA) {
        LIB_LOGD(TAG, "rawRecv type %d,%d len %d", ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype, pkt->rx_ctrl.sig_len);
        LIB_LOGD(TAG, "mac1 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr1[0], ieee80211_hdr->addr1[1], ieee80211_hdr->addr1[2], \
            ieee80211_hdr->addr1[3], ieee80211_hdr->addr1[4], ieee80211_hdr->addr1[5]);
        LIB_LOGD(TAG, "mac2 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr2[0], ieee80211_hdr->addr2[1], ieee80211_hdr->addr2[2], \
            ieee80211_hdr->addr2[3], ieee80211_hdr->addr2[4], ieee80211_hdr->addr2[5]);
        LIB_LOGD(TAG, "mac3 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr3[0], ieee80211_hdr->addr3[1], ieee80211_hdr->addr3[2], \
            ieee80211_hdr->addr3[3], ieee80211_hdr->addr3[4], ieee80211_hdr->addr3[5]);
    }
#endif
    
    static uint8_t brdaddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if(memcmp(brdaddr, ieee80211_hdr->addr1, 6) != 0 && (ieee80211_hdr->addr1[0]!=0xFE || ieee80211_hdr->addr1[1]!=0x7F ||
        ieee80211_hdr->addr1[2]!=nodeIdPtr()[3] || ieee80211_hdr->addr1[3]!=nodeIdPtr()[2] || ieee80211_hdr->addr1[4]!=nodeIdPtr()[1],
        ieee80211_hdr->addr1[5]!=nodeIdPtr()[0])) {
        return;
    }
    
    if(ieee80211_hdr->frame_control.Type == FRAME_TYPE_DATA && ieee80211_hdr->frame_control.Subtype == FRAME_SUBTYPE_DATA && \
        ieee80211_hdr->addr2[0] == 0xFE && (ieee80211_hdr->addr2[1]&0xF0) == 0x70) {
    
        //LIB_LOGD(TAG, "rawRecv type %d,%d len %d", ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype, pkt->rx_ctrl.sig_len);
        if(pkt->rx_ctrl.sig_len<=PACKETBUF_80211_SIZE+5) {
            LIB_LOGE(TAG, "rawRecv short packet %d", pkt->rx_ctrl.sig_len);
            return;
        }
    
        if(hwFreeHeap()<pktbufRecvTaskPacket[pktbufRecvTaskIndex].length+128 || hwFreeHeap()<MEMORY_TRESHOLD) {
            LIB_LOGE(TAG, "recvTask low memory a:%d r:%ld", heap_caps_get_free_size(MALLOC_CAP_8BIT), pktbufRecvTaskPacket[pktbufRecvTaskIndex].length);
            return;
        }

        if(pktbufRecvTaskPacket[pktbufRecvTaskIndex].length > 0) {
            LIB_LOGE(TAG, "rawRecv fifo full");
            return;
        }

        //LIB_LOGD(TAG, "rawRecv enqueued %d len %d", pktbufRecvTaskIndex, pkt->rx_ctrl.sig_len);
        pktbufRecvTaskPacket[pktbufRecvTaskIndex].length = pkt->rx_ctrl.sig_len;
        pktbufRecvTaskPacket[pktbufRecvTaskIndex].data = new uint8_t[pktbufRecvTaskPacket[pktbufRecvTaskIndex].length];
        memcpy(pktbufRecvTaskPacket[pktbufRecvTaskIndex].data, pkt->payload, pktbufRecvTaskPacket[pktbufRecvTaskIndex].length);
        pktbufRecvTaskPacket[pktbufRecvTaskIndex].rssi = pkt->rx_ctrl.rssi;
        xQueueSend(mRecvQueue, &pktbufRecvTaskIndex, 0);
        if(++pktbufRecvTaskIndex >= PACKETBUF_TASK_QUEUE_LEN) pktbufRecvTaskIndex = 0;
    } else if(ieee80211_hdr->frame_control.Type == FRAME_TYPE_MANAGEMENT && ieee80211_hdr->frame_control.Subtype == FRAME_SUBTYPE_PROBE_REQUEST) {
    
    } else {
        //LIB_LOGD(TAG, "Unknow pkt type:%d sub:type:%d", ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype);
        //DEBUG_ARRAY("Unknow pkt: ", pkt->payload, pkt->rx_ctrl.sig_len);
    }
}

void WifiDrvEsp32::_promiscuousRxq(void* buf, wifi_promiscuous_pkt_type_t type) {
    //wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t*)buf;
    if(type == WIFI_PKT_DATA && wifiDrvEsp32) wifiDrvEsp32->promiscuousRxq((RxPacket *)buf);
}


}
#endif