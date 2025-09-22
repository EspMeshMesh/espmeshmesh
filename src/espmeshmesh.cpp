#include "espmeshmesh.h"
#include <algorithm>

#include "log.h"
#include "crc16.h"
#include "commands.h"
#include "packetbuf.h"
#include "broadcast.h"
#include "broadcast2.h"
#include "unicast.h"
#ifdef USE_MULTIPATH_PROTOCOL
#include "multipath.h"
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
#include "polite.h"
#endif
#ifdef USE_CONNECTED_PROTOCOL
#include "connectedpath.h"
#endif

#ifdef ESP8266
#include <Esp.h>
#include <osapi.h>
#include <user_interface.h>
#include <md5.h>
#endif

#ifdef IDF_VER
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_rom_md5.h>
#include <cstring>
#endif

extern "C" uint32_t _FS_start;
extern "C" uint32_t _SPIFFS_start;

namespace espmeshmesh {

static const char *TAG = "espmeshmesh";

#define BROADCAST_DEFAULT_PORT 0
#define UNICAST_DEFAULT_PORT 0
#define MULTIPATH_DEFAULT_PORT 0

EspMeshMesh *EspMeshMesh::singleton = nullptr;

EspMeshMesh *EspMeshMesh::getInstance() { return singleton; }

EspMeshMesh::EspMeshMesh(int baud_rate, int tx_buffer, int rx_buffer)
    : mBaudRate(baud_rate), mTxBuffer(tx_buffer), mRxBuffer(rx_buffer) {
  if (singleton == nullptr)
    singleton = this;
}

void EspMeshMesh::pre_setup() {
#ifdef ESP8266
  mHwSerial = &Serial;
  LIB_LOGD(TAG, "pre_setup baudrate %d", mBaudRate);
  if (mBaudRate > 0) {
    mHwSerial->begin(mBaudRate);
    mHwSerial->setRxBufferSize(mRxBuffer);
  }
#endif  // ESP8266

#ifdef IDF_VER
  initIdfUart();
#endif  // ESP32

  if (mTxBuffer > 0)
    mUartTxBuffer.resize(mTxBuffer);
}

#ifdef IDF_VER
void EspMeshMesh::wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  LIB_LOGD(TAG, "wifi_event_handler %ld", event_id);
}
#endif

#ifdef IDF_VER
bool EspMeshMesh::setupIdfWifiAP(const char *hostname, uint8_t channel, uint8_t txPower) {
  esp_err_t res;
  wifi_config_t wcfg;
  strcpy((char *) wcfg.ap.ssid, "esphome");
  strcpy((char *) wcfg.ap.password, "esphome");
  wcfg.ap.ssid_len = 0;
  wcfg.ap.channel = channel;

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

  res = esp_event_handler_register(ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr);
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

bool EspMeshMesh::setupIdfWifiStation(const char *hostname, uint8_t channel, uint8_t txPower) {
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

  res = esp_event_handler_register(ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr);
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

  LIB_LOGI(TAG, "Selected channel %d", channel);
  res = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  if(res != ESP_OK) {
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
#endif

void EspMeshMesh::setupWifi(const char *hostname, uint8_t channel, uint8_t txPower) {
  LIB_LOGCONFIG(TAG, "Setting up meshmesh wifi...");
#ifdef IDF_VER
  setupIdfWifiStation(hostname, channel, txPower);
#else
  wifi_station_set_hostname(hostname);
  wifi_set_opmode(STATION_MODE);
  wifiInitMacAddr(STATION_IF);
  wifi_station_set_auto_connect(false);
  wifi_set_phy_mode(PHY_MODE_11B);
  wifi_set_channel(channel);
  system_phy_set_max_tpw(txPower);
  LIB_LOGCONFIG(TAG, "Channel cfg:%d txPower:%d", channel, txPower);
#endif
  LIB_LOGD(TAG, "Wifi succesful!!!!");
}

void EspMeshMesh::setup(EspMeshMeshSetupConfig *config) {
  mUseSerial = mBaudRate > 0;

  setupWifi(config->hostname, config->channel, config->txPower);

  uint8_t aespassword[16];
  if (mAesPassword.size() == 0) {
    memcpy(aespassword, "1234567890ABCDEF", 16);
  } else {
#ifdef ESP8266
    md5_context_t md5;
    MD5Init(&md5);
    MD5Update(&md5, (uint8_t *) mAesPassword.c_str(), mAesPassword.size());
    MD5Final(aespassword, &md5);
#endif
#ifdef IDF_VER
    md5_context_t md5;
    esp_rom_md5_init(&md5);
    esp_rom_md5_update(&md5, (uint8_t *) mAesPassword.c_str(), mAesPassword.size());
    esp_rom_md5_final(aespassword, &md5);
#endif  
  }
  
  packetbuf = PacketBuf::getInstance();
  packetbuf->setup(aespassword, 16);
  broadcast = new Broadcast(packetbuf);
  broadcast2 = new Broadcast2(packetbuf);
  unicast = new Unicast(packetbuf);

#ifdef USE_MULTIPATH_PROTOCOL
  multipath = new MultiPath(packetbuf);
  multipath->setup();
  multipath->bindPort(MULTIPATH_DEFAULT_PORT, std::bind(&EspMeshMesh::multipathRecv, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
#endif

#ifdef USE_POLITE_BROADCAST_PROTOCOL
  mPoliteBroadcast = new PoliteBroadcastProtocol(packetbuf);
  mPoliteBroadcast->setup();
  packetbuf->setPoliteBroadcast(mPoliteBroadcast);
  mPoliteBroadcast->setReceivedHandler(politeBroadcastReceive, this);
#endif

#ifdef USE_CONNECTED_PROTOCOL
  mConnectedPath = new ConnectedPath(this, packetbuf);
  mConnectedPath->setup();
  mConnectedPath->bindPort(onConnectedPathNewClientCb, this, 0);
#endif

  mDiscovery.init();
  broadcast->setRecv_cb(user_broadcast_recv_cb);
  broadcast2->bindPort(BROADCAST_DEFAULT_PORT, std::bind(&EspMeshMesh::user_broadcast2_recv, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  unicast->setup();
  unicast->bindPort(UNICAST_DEFAULT_PORT, std::bind(&EspMeshMesh::unicastRecv, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  dump_config();
  mElapsed1 = millis();

#ifdef USE_BINARY_SENSOR
  for (auto binary : App.get_binary_sensors()) {
    LIB_LOGCONFIG(TAG, "Found binary sensor %s with hash %08X", binary->get_object_id().c_str(),
                  binary->get_object_id_hash());
  }
#endif

#ifdef USE_TEST_PROCEDURE
  mTestProcedureTime = millis();
#endif
}

void EspMeshMesh::dump_config() {
}

void EspMeshMesh::loop() {
  uint32_t now = millis();

  if (mBaudRate > 0) {
#ifdef ESP8266
    int avail = mHwSerial->available();
    if (avail)
      LIB_LOGD(TAG, "MeshmeshComponent::loop available %d", avail);
    while (avail--)
      user_uart_recv_data((uint8_t) mHwSerial->read());
#endif

#ifdef IDF_VER
    size_t avail;
    uart_get_buffered_data_len(mUartNum, &avail);
    uint8_t data;
    while (avail--) {
      uart_read_bytes(mUartNum, &data, 1, 0);
      user_uart_recv_data(data);
    }
#endif
  }

  if (mUartTxBuffer.filledSpace() > 0)
    flushUartTxBuffer();
#ifdef USE_ESP32
  packetbuf->loop();
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  if (mPoliteBroadcast)
    mPoliteBroadcast->loop();
#endif
#ifdef USE_MULTIPATH_PROTOCOL
  // execute multipath loop
  multipath->loop();
#endif
#ifdef USE_CONNECTED_PROTOCOL
  mConnectedPath->loop();
#endif
  // Execute discovery if is running
  if (mDiscovery.isRunning())
    mDiscovery.loop(this);

#ifdef ESP8266
  if (!mWorkAround && elapsedMillis(now, mElapsed1) > 2000) {
    mWorkAround = true;
    uint8_t data[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
    uniCastSendData(data, 0x6, 0x111111);
    LIB_LOGI(TAG, "Sent dummy workaround packet");
  }
#endif

  if (elapsedMillis(now, mElapsed1) > 60000) {
#ifdef IDF_VER
    LIB_LOGI(TAG, "Free Heap %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
#else
    LIB_LOGI(TAG, "Free Heap %d", ESP.getFreeHeap());
#endif
    mElapsed1 = millis();
  }
}

void EspMeshMesh::uartSendData(const uint8_t *buff, uint16_t len) {
  if (!mUseSerial)
    return;

  uint16_t i;
  uint16_t crc16 = 0;
  mUartTxBuffer.pushByte(CODE_DATA_START_CRC16);
  for (i = 0; i < len; i++) {
    if (buff[i] == 0xEA || buff[i] == 0xEF || buff[i] == 0xFE) {
      mUartTxBuffer.pushByte(CODE_DATA_ESCAPE);
      crc16 = crc_ibm_byte(crc16, CODE_DATA_ESCAPE);
    }
    mUartTxBuffer.pushByte(buff[i]);
    crc16 = crc_ibm_byte(crc16, buff[i]);
  }
  mUartTxBuffer.pushByte(CODE_DATA_END);
  mUartTxBuffer.pushByte(crc16 >> 8);
  mUartTxBuffer.pushByte(crc16 & 0xFF);
  flushUartTxBuffer();
}

void EspMeshMesh::broadCastSendData(const uint8_t *buff, uint16_t len) {
  if (broadcast)
    broadcast->send(buff, len);
}

void EspMeshMesh::broadcast2SendData(const uint8_t *buff, uint16_t len, bool port) {
  if (broadcast2)
    broadcast2->send(buff, len, port, nullptr);
}

void EspMeshMesh::registerBroadcast2Port(uint16_t port, Broadcast2ReceiveRadioPacketHandler handler) {
  if (broadcast2)
    broadcast2->bindPort(port, handler);
}

void EspMeshMesh::uniCastSendData(const uint8_t *buff, uint16_t len, uint32_t addr) {
  if (unicast)
    unicast->send(buff, len, addr, UNICAST_DEFAULT_PORT, nullptr);
}

void EspMeshMesh::unicastSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint16_t port) {
  if (unicast)
    unicast->send(buff, len, addr, port, nullptr);
}

#ifdef USE_MULTIPATH_PROTOCOL
void EspMeshMesh::multipathSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint8_t pathlen,
                                          uint8_t *path) {
  if (!multipath)
    return;
  MultiPathPacket *pkt = new MultiPathPacket(nullptr, nullptr);
  pkt->allocClearData(len, pathlen);
  pkt->multipathHeader()->trargetAddress = addr;
  for (int i = 0; i < pathlen; i++)
    pkt->setPathItem(uint32FromBuffer(path) + i * sizeof(uint32_t), i);
  pkt->setPayload(buff);
  multipath->send(pkt, true, nullptr);
}
#endif

#define DEF_CMD_BUFFER_SIZE 0x440
#define MAX_CMD_BUFFER_SIZE 0x440

#ifdef IDF_VER
void EspMeshMesh::initIdfUart() {
  if (!mBaudRate)
    return;

  uart_config_t uart_config{};
  uart_config.baud_rate = (int) mBaudRate;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  uart_config.source_clk = UART_SCLK_DEFAULT;
#endif
  uart_port_t uartNum = UART_NUM_0;
  uart_param_config(uartNum, &uart_config);  // FIXME
  uart_driver_install(uartNum, mTxBuffer, mTxBuffer, 10, nullptr, 0);
}
#endif

void EspMeshMesh::user_uart_recv_data(uint8_t byte) {
  static uint16_t computed_crc16 = 0;
  static uint16_t received_crc16 = 0;

  switch (mRecvState) {
    case WAIT_START:
      if (byte == CODE_DATA_START_CRC16) {
        // LIB_LOGD(TAG, "MeshmeshComponent::user_uart_recv_data find start");
        computed_crc16 = 0;
        mRecvState = WAIT_DATA;
        if (!mRecvBuffer) {
          mRecvBuffer = new uint8_t[DEF_CMD_BUFFER_SIZE];
        }
        memset(mRecvBuffer, 0, DEF_CMD_BUFFER_SIZE);
        mRecvBufferPos = 0;
      }
      break;
    case WAIT_DATA:
      if (byte == CODE_DATA_END) {
        mRecvState = WAIT_CRC16_1;
      } else {
        if (byte == CODE_DATA_ESCAPE) {
          computed_crc16 = crc_ibm_byte(computed_crc16, byte);
          mRecvState = WAIT_ESCAPE;
        } else {
          computed_crc16 = crc_ibm_byte(computed_crc16, byte);
          mRecvBuffer[mRecvBufferPos++] = byte;
        }
      }
      break;
    case WAIT_ESCAPE:
      if (byte == CODE_DATA_ESCAPE || byte == CODE_DATA_END || byte == CODE_DATA_START) {
        computed_crc16 = crc_ibm_byte(computed_crc16, byte);
        mRecvBuffer[mRecvBufferPos++] = byte;
      }
      mRecvState = WAIT_DATA;
      break;
    case WAIT_CRC16_1:
      received_crc16 = uint16_t(byte) << 8;
      mRecvState = WAIT_CRC16_2;
      break;
    case WAIT_CRC16_2:
      received_crc16 = received_crc16 | uint16_t(byte);
      if (computed_crc16 == received_crc16) {
        handleFrame(mRecvBuffer, mRecvBufferPos, SRC_SERIAL, 0xFFFFFFFF);
      } else {
        //handleFrame(mRecvBuffer, mRecvBufferPos, SRC_SERIAL, 0xFFFFFFFF);
        LIB_LOGE(TAG, "CRC16 mismatch %04X %04X", computed_crc16, received_crc16);
      }
      mRecvState = WAIT_START;
      break;
  }
}

void EspMeshMesh::flushUartTxBuffer() {
#ifdef ESP8266
  int avail = mHwSerial->availableForWrite() - 8;
#endif

#ifdef IDF_VER
  int avail = mUartTxBuffer.filledSpace();
#endif

  if (avail > 0) {
    uint8_t *buff = new uint8_t[avail];
    avail = mUartTxBuffer.popData(buff, avail);
#ifdef ESP8266
    mHwSerial->write(buff, avail);
#endif

#ifdef IDF_VER
    uart_write_bytes(mUartNum, buff, avail);
#endif

    delete[] buff;
  }
}

void EspMeshMesh::commandReply(const uint8_t *buff, uint16_t len) {
  uint8_t err = 0;
  switch (commandSource) {
    case SRC_SERIAL:
      uartSendData(buff, len);
      break;
    case SRC_BROADCAST:
    case SRC_BROADCAST2:
    case SRC_UNICAST:
      err = unicast->send(buff, len, uint32FromBuffer(mRecvFromId), UNICAST_DEFAULT_PORT, nullptr);
      break;
    case SRC_MULTIPATH:
#ifdef USE_MULTIPATH_PROTOCOL
      err = multipath->send(buff, len, uint32FromBuffer(mRecvFromId), (uint32_t *)&mRecvPath[0], mRecvPathSize, true, MULTIPATH_DEFAULT_PORT, nullptr);
#endif
      break;
    case SRC_POLITEBRD:
#ifdef USE_POLITE_BROADCAST_PROTOCOL
      if (mPoliteFromAddress != POLITE_DEST_BROADCAST)
        mPoliteBroadcast->send(buff, len, mPoliteFromAddress);
#endif
      break;
    case SRC_CONNPATH:
#ifdef USE_CONNECTED_PROTOCOL
      // mConnectedPath->sendDataTo(buff, len, mConnectionId);
      LIB_LOGE(TAG, "commandReply SRC_CONNPATH not handled");
#endif
      break;
    case SRC_FILTER:
      break;
  }

  commandSource = SRC_SERIAL;
}

void EspMeshMesh::handleFrame(const uint8_t *data, uint16_t len, DataSrc src, uint32_t from) {
  uint8_t err = HANDLE_UART_ERROR;

  uint8_t *buf = new uint8_t[len];
  memcpy(buf, data, len);

  // LIB_LOGD(TAG, "MeshmeshComponent::handleFrame src %d cmd %02X:%02X len %d", src, buf[0], buf[1], len);
  // print_hex_array("handleFrame ", buf, len);

  commandSource = src;
  if (buf[0] & 0x01) {
    replyHandleFrame(buf, len, src, from);
    delete[] buf;
    return;
  }

  switch (buf[0]) {
    case CMD_UART_ECHO_REQ:
      if (len > 1) {
        buf[0] = CMD_UART_ECHO_REP;
        uartSendData(buf, len);
        err = 0;
      }
      break;
    case CMD_NODE_ID_REQ:  // 04 --> 05 0000XXYY
      if (len == 1) {
        uint32_t id = Discovery::chipId();
        uint8_t rep[5] = {0};
        rep[0] = CMD_NODE_ID_REP;
        memcpy(rep + 1, (uint8_t *) &id, 4);
        commandReply(rep, 5);
        err = 0;
      }
      break;
    case CMD_DISCOVERY_REQ:
      if (len > 1) {
        err = mDiscovery.handle_frame(buf + 1, len - 1, this);
      }
      break;
    case CMD_BROADCAST_SEND:  // 70 AABBCCDDEE...ZZ
      if (len > 1) {
        broadcast->send(buf + 1, len - 1);
        err = 0;
      }
      break;
    case CMD_UNICAST_SEND:  // 72 0000XXYY AABBCCDDEE...ZZ
      if (len > 5) {
        LIB_LOGD(TAG, "CMD_UNICAST_SEND len %d", len);
        unicast->send(buf + 5, len - 5, uint32FromBuffer(buf + 1), UNICAST_DEFAULT_PORT, nullptr);
        err = 0;
      }
      break;
    case CMD_MULTIPATH_SEND:  // 76 AAAAAAAA BB CCCCCCCC........DDDDDDDD AABBCCDDEE...ZZ
#ifdef USE_MULTIPATH_PROTOCOL
      if (len > 5) {
        uint8_t pathlen = buf[5];
        if (len > 6 + pathlen * sizeof(uint32_t)) {
          uint16_t payloadsize = len - (6 + pathlen * sizeof(uint32_t));
          MultiPathPacket *pkt = new MultiPathPacket(nullptr, nullptr);
          pkt->allocClearData(payloadsize, pathlen);
          pkt->multipathHeader()->sourceAddress = packetbuf->nodeId();
          pkt->multipathHeader()->trargetAddress = uint32FromBuffer(buf + 1);
          for (int i = 0; i < pathlen; i++)
            pkt->setPathItem(uint32FromBuffer(buf + 6) + i * sizeof(uint32_t), i);
          pkt->setPayload(buf + 6 + sizeof(uint32_t) * pathlen);
          multipath->send(pkt, true, nullptr);
          err = 0;
        }
      }
#endif
      break;
#ifdef USE_POLITE_BROADCAST_PROTOCOL
    case CMD_POLITEBRD_SEND:
      if (len > 5) {
        mPoliteBroadcast->send(buf + 5, len - 5, uint32FromBuffer(buf + 1));
        err = 0;
      }
      break;
#endif
#ifdef USE_CONNECTED_PROTOCOL
    case CMD_CONNPATH_REQUEST:
      if (len > 1 && src == SRC_SERIAL) {
        err = mConnectedPath->receiveUartPacket(buf + 1, len - 1);
      }
      break;
#endif
    case CMD_FILTERED_REQUEST:
      if (len > 5) {
        uint8_t i;
        uint8_t *groups = (uint8_t *) &mFilterGroups;
        for (i = 0; i < 4; i++)
          if (buf[i + 1] != 0xFF && (groups[i] == 0 || (groups[i] != 0xFF && groups[i] != buf[i + 1])))
            break;
        if (i == 4) {
          handleFrame(buf + 5, len - 5, SRC_FILTER, 0);
        }
        err = 0;
      }
      break;
    default:
      for (auto cb : mHandleFrameCbs) {
        int8_t handled = cb(buf, len, from);
        // If callback handled the frame...
        if (handled >= 0) {
          // Keep status and exit loop
          err = handled;
          break;
        }
      }
      break;
  }

  if (err == HANDLE_UART_ERROR && commandSource != SRC_BROADCAST) {
    // Don't reply errors when commd came from broadcast
    uint8_t *rep = new uint8_t[len + 1];
    rep[0] = CMD_ERROR_REP;
    memcpy(rep + 1, buf, len);
    commandReply(rep, len + 1);
    LIB_LOGD(TAG, "EspMeshMesh::handleFrame error frame %02X %02X size %d", buf[0], buf[1], len);
    delete[] rep;
  }

  delete[] buf;
}

void EspMeshMesh::replyHandleFrame(uint8_t *buf, uint16_t len, DataSrc src, uint32_t from) {
  // All replies go to the serial, if the serial is active
  switch (buf[0]) {
    case CMD_LOGEVENT_REP:
      // Add the source to the log essage
      memcpy(buf + 3, (uint8_t *) &from, 4);
      uartSendData(buf, len);
      break;
    case CMD_BEACONS_RECV:
      // Compatibility with previous discovery procedure
      if (len == sizeof(BaconsDataCompat_t)) {
        BaconsDataCompat_t *b = (BaconsDataCompat_t *) buf;
        mDiscovery.process_beacon(b->id, b->rssi, 0);
      }
      break;
    default:
      uartSendData(buf, len);
      break;
  }
}

#define CMD_FLASH_GETMD5 0x01
#define CMD_FLASH_ERASE 0x02
#define CMD_FLASH_WRITE 0x03
#define CMD_FLASH_EBOOT 0x04
#define CMD_FLASH_PREPARE 0x05

void EspMeshMesh::user_broadcast_recv_cb(uint8_t *data, uint16_t size, uint8_t *from, int16_t rssi) {
  if (singleton) {
    singleton->user_broadcast_recv(data, size, from, rssi);
  }
}

void EspMeshMesh::user_broadcast_recv(uint8_t *data, uint16_t size, uint8_t *from, int16_t rssi) {
  // Ignore error frame frmo broadcast
  if (size == 0 || data[0] == 0x7F)
    return;
  memcpy(&mBroadcastFromAddress, from, 4);
  memcpy(mRecvFromId, from, 4);
  mRssiHandle = rssi;
  uint32_t *addr = (uint32_t *) from;
  LIB_LOGD(TAG, "MeshmeshComponent::user_broadcast_recv from %06lX size %d cmd %02X", *addr, size, data[0]);
  handleFrame(data, size, SRC_BROADCAST, *(uint32_t *) from);
}

void EspMeshMesh::user_broadcast2_recv(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi) {
  // Ignore error frame frmo broadcast
  if (size == 0 || data[0] == 0x7F)
    return;
  memcpy(mRecvFromId, (uint8_t *) &from, 4);
  mRssiHandle = rssi;
  LIB_LOGD(TAG, "MeshmeshComponent::user_broadcast2_recv from %06lX size %d cmd %02X", from, size, data[0]);
  handleFrame(data, size, SRC_BROADCAST2, from);
}

void EspMeshMesh::unicastRecv(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi) {
  // LIB_LOGD(TAG, "unicastRecv %d", size);
  memcpy(mRecvFromId, (uint8_t *) &from, 4);
  mRssiHandle = rssi;
  handleFrame(data, size, SRC_UNICAST, from);
}

void EspMeshMesh::multipathRecv(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi, uint8_t *path,
                                      uint8_t pathSize) {
  memcpy(mRecvFromId, (uint8_t *) &from, 4);
  mRecvPathSize = pathSize;
  if (mRecvPathSize)
    memcpy(mRecvPath, path, mRecvPathSize * sizeof(uint32_t));
  mRssiHandle = rssi;
  handleFrame(data, size, SRC_MULTIPATH, from);
}

void EspMeshMesh::politeBroadcastReceive(void *arg, uint8_t *data, uint16_t size, uint32_t from) {
  ((EspMeshMesh *) arg)->politeBroadcastReceiveCb(data, size, from);
}

void EspMeshMesh::politeBroadcastReceiveCb(uint8_t *data, uint16_t size, uint32_t from) {
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  if (size == 0 || data[0] == 0x7F)
    return;
  mPoliteFromAddress = from;
  handleFrame(data, size, SRC_POLITEBRD, from);
#endif
}

void EspMeshMesh::onConnectedPathNewClientCb(void *arg, uint32_t from, uint16_t handle) {
  ((EspMeshMesh *) arg)->onConnectedPathNewClient(from, handle);
}

void EspMeshMesh::onConnectedPathNewClient(uint32_t from, uint16_t handle) {
#ifdef USE_CONNECTED_PROTOCOL
  LIB_LOGD(TAG, "MeshmeshComponent::onConnectedPathNewClien %06lX:%04X", from, handle);
  mConnectedPath->setReceiveCallback(onConnectedPathReceiveCb, nullptr, this, from, handle);
#endif
}

void EspMeshMesh::onConnectedPathReceiveCb(void *arg, const uint8_t *data, uint16_t size, uint8_t connid) {
  ((EspMeshMesh *) arg)->onConnectedPathReceive(data, size, connid);
}

void EspMeshMesh::onConnectedPathReceive(const uint8_t *data, uint16_t size, uint8_t connid) {
#ifdef USE_CONNECTED_PROTOCOL
  mConnectionId = connid;
  handleFrame(data, size, SRC_CONNPATH, connid);
#endif
}

void EspMeshMesh::sendLog(int level, const char *tag, const char *payload, size_t payload_len) {
  if (mBaudRate == 0 && mLogDestination == 0)
    return;

  // uint16_t buffersize = 7+taglen+1+payloadlen;
  uint16_t buffersize = 7 + payload_len;

  auto buffer = new uint8_t[buffersize];
  auto buffer_ptr = buffer;

  *buffer_ptr = CMD_LOGEVENT_REP;
  buffer_ptr++;
  uint16toBuffer(buffer_ptr, (uint16_t) level);
  buffer_ptr += 2;
  uint32toBuffer(buffer_ptr, 0);
  buffer_ptr += 4;

  if (payload_len > 0)
    memcpy(buffer_ptr, payload, payload_len);
  if (mBaudRate > 0)
    uartSendData(buffer, buffersize);
  if (mLogDestination == 1) {
    if (broadcast)
      broadcast->send(buffer, buffersize);
  } else if (mLogDestination > 1) {
    if (unicast)
      unicast->send(buffer, buffersize, mLogDestination, UNICAST_DEFAULT_PORT, nullptr);
  }

  delete[] buffer;
}

void EspMeshMesh::wifiInitMacAddr(uint8_t index) {
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

#ifdef USE_ESP32
  esp_err_t res = esp_wifi_set_mac((wifi_interface_t) index, mac);
  if (res != ESP_OK) {
    LIB_LOGD(TAG, "esp_wifi_set_mac error %d", res);
  }
#else
  wifi_set_macaddr(index, mac);
#endif
}

}  // namespace espmeshmesh
