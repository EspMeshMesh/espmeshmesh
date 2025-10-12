#include "discovery.h"
#include "log.h"
#include "commands.h"
#include "espmeshmesh.h"

#if USE_ESP32
#include <esp_random.h>
#include <esp_mac.h>
#endif

#include <cstring>

namespace espmeshmesh {

#define INF_RSSI 1000
#define NULL_RSSI -1000

#define BEACONS_PERIOD 12
#define BEACONS_DELAY(X) (BEACONS_PERIOD / 2 + BEACONS_SLOT(X) * BEACONS_PERIOD)
#ifdef USE_ESP32
#define BEACONS_SLOT(X) (esp_random() % X)
#else
#define BEACONS_SLOT(X) (random() % X)
#endif

#define DISCCMD_RESET_TABLE_REQ 0x00
#define DISCCMD_RESET_TABLE_REP 0x01
#define DISCCMD_TABLE_SIZE_REQ 0x02
#define DISCCMD_TABLE_SIZE_REP 0x03
#define DISCCMD_TABLE_ITEM_GET_REQ 0x04
#define DISCCMD_TABLE_ITEM_GET_REP 0x05
#define DISCCMD_START_REQ 0x06
#define DISCCMD_START_REP 0x07
#define DISCCMD_BEACONS_SEND_REQ 0x08
#define DISCCMD_BEACONS_SEND_REP 0x09

static const char *TAG = "espmeshmesh.discovery";

void Discovery::init() { clear_table(); }

void Discovery::loop(EspMeshMesh *parent) {
  uint32_t now = millis();
  if (mRunPhase == 1) {
    if (EspMeshMesh::elapsedMillis(now, mStartTime) > 50) {
      mRunPhase++;
      mStartTime = now;
    }
  } else if (mRunPhase == 2) {
    mStart.cmd1 = CMD_DISCOVERY_REQ;
    mStart.cmd2 = DISCCMD_BEACONS_SEND_REQ;

    mStartCompat.cmd1 = CMD_BEACONS_SEND;
    mStartCompat.filter = mStart.filter;
    mStartCompat.mask = mStart.mask;
    mStartCompat.slotnum = mStart.slotnum;

    parent->broadCastSendData((uint8_t *) &mStart, sizeof(CmdStart_t));
    // parent->broadCastSendData((uint8_t *)&mStartCompat, sizeof(CmdStartCompat_t));
    // parent->broadCastSendData((uint8_t *)&mStart, sizeof(CmdStart_t));
    // parent->broadCastSendData((uint8_t *)&mStartCompat, sizeof(mStartCompat));

    mRunPhase++;
  } else if (mRunPhase == 3) {
    if (EspMeshMesh::elapsedMillis(now, mStartTime) > 2000) {
      LIB_LOGD(TAG, "Discovery::loop discovery end");
      mRunPhase = 0;
      mStartTime = now;
    }
  }

  if (mRunPhase == 11) {
    if (EspMeshMesh::elapsedMillis(now, mStartTime) > mBeaconDelay) {
      BaconsData_t data;
      data.reply1 = CMD_DISCOVERY_REQ;
      data.reply2 = DISCCMD_BEACONS_SEND_REP;
      data.id = Discovery::chipId();
      // FIXME: Is not true anyore beacuse the recv packet queue
      data.rssi = (int16_t) parent->lastPacketRssi();

      parent->uniCastSendData((uint8_t *) &data, sizeof(BaconsData_t), parent->broadcastFromAddress());
      LIB_LOGD(TAG, "Discovery::loop beacon reply end");
      mRunPhase = 0;
    }
  }
}

void Discovery::process_beacon(uint32_t id, int16_t rssi1, int16_t rssi2) {
  uint8_t i;
  LIB_LOGD(TAG, "discovery_process_beacon id:%06lX rssi1:%d rssi2:%d\n", id, rssi1, rssi2);
  for (i = 0; i < discovery_table_index; i++)
    if (discovery_table[i].id == id)
      break;
  if (i < DISCOVERY_TABLE_SIZE) {
    discovery_table[i].rssi1 = rssi1;
    discovery_table[i].rssi2 = rssi2;
    discovery_table[i].id = id;
    if (i == discovery_table_index)
      discovery_table_index++;
  }
}

void Discovery::clear_table(void) {
  memset(discovery_table, '\0', sizeof(DiscoveryItem_t) * 64);
  discovery_table_index = 0;
}

uint8_t Discovery::handle_frame(const uint8_t *buf, uint16_t len, EspMeshMesh *parent) {
  uint8_t err = 1;
  switch (buf[0]) {
    case DISCCMD_RESET_TABLE_REQ:
      if (len == 1) {
        uint8_t rep[2];
        rep[0] = CMD_DISCOVERY_REP;
        rep[1] = DISCCMD_RESET_TABLE_REP;
        clear_table();
        parent->commandReply(rep, 2);
        err = 0;
      }
      break;
    case DISCCMD_TABLE_SIZE_REQ:
      if (len == 1) {
        uint8_t rep[3];
        rep[0] = CMD_DISCOVERY_REP;
        rep[1] = DISCCMD_TABLE_SIZE_REP;
        rep[2] = discovery_table_index;
        parent->commandReply(rep, 3);
        err = 0;
      }
      break;
    case DISCCMD_TABLE_ITEM_GET_REQ:
      if (len == 2 && buf[1] < discovery_table_index) {
        uint8_t rep[3 + sizeof(DiscoveryItem_t)];
        rep[0] = CMD_DISCOVERY_REP;
        rep[1] = DISCCMD_TABLE_ITEM_GET_REP;
        rep[2] = buf[1];
        memcpy(rep + 3, discovery_table + buf[1], sizeof(DiscoveryItem_t));
        parent->commandReply(rep, 3 + sizeof(DiscoveryItem_t));
        err = 0;
      }
      break;
    case DISCCMD_START_REQ:
      if (len == sizeof(CmdStart_t) - 1) {
        // Silently ignore discovery starts if we already started., master can send mutiple packets
        if (mRunPhase == 0) {
          discoveryStart(buf, len);
          uint8_t rep[2];
          rep[0] = CMD_DISCOVERY_REP;
          rep[1] = DISCCMD_START_REP;
          parent->commandReply(rep, 2);
        }
        err = 0;
      }
      break;
    case DISCCMD_BEACONS_SEND_REQ:
      if (len == sizeof(CmdStart_t) - 1 && !isRunning()) {
        memcpy(((uint8_t *) &mStart) + 1, buf, sizeof(CmdStart_t) - 1);
        if (mStart.mask == 0 || (Discovery::chipId() & mStart.mask) == mStart.filter) {
          mRunPhase = 11;
          mStartTime = millis();
          mBeaconDelay = BEACONS_DELAY(mStart.slotnum);
          LIB_LOGD(TAG, "Discovery::handle_frame beacon reply id %02lX mask %d filt %d slots %d delay %ld",
                   Discovery::chipId() & mStart.mask, mStart.mask, mStart.filter, mStart.slotnum, mBeaconDelay);
        }
        err = 0;
      }
      break;
    case DISCCMD_BEACONS_SEND_REP:
      if (len == sizeof(BaconsData_t) - 1) {
        BaconsData_t data;
        memcpy(((uint8_t *) &data) + 1, buf, sizeof(BaconsData_t) - 1);
        process_beacon(data.id, data.rssi, (int16_t) parent->lastPacketRssi());
        err = 0;
      }
      break;

  }
  return err;
}

uint32_t Discovery::chipId() {
#ifdef IDF_VER
  uint64_t macAddress;
  esp_efuse_mac_get_default((uint8_t *) &macAddress);
  macAddress = __builtin_bswap64(macAddress);
  return (uint32_t) (macAddress >> 16) & 0xFFFFFF;
#else
  return system_get_chip_id();
#endif
}

void Discovery::findMaxRssi(int16_t max, int16_t &maxRssi, uint32_t &maxRssiNodeId) {
  maxRssiNodeId = 0;
  maxRssi = NULL_RSSI;

  for (uint8_t i = 0; i < discovery_table_index; i++) {
    uint16_t rssi = std::min(discovery_table[i].rssi1, discovery_table[i].rssi2);
    if (rssi > maxRssi && rssi < max) {
      maxRssi = rssi;
      maxRssiNodeId = discovery_table[i].id;
    }
  }
}

void Discovery::discoveryStart(const uint8_t *buf, uint16_t len) {
  uint8_t *data = ((uint8_t *) &mStart) + 1;
  memcpy(data, buf, sizeof(CmdStart_t));
  discoveryStart(mStart.slotnum);
}

void Discovery::discoveryStart(uint8_t slotnum) {
  if (mRunPhase != 0)
    return;
  mStart.slotnum = slotnum;
  if (mStart.slotnum < 10)
    mStart.slotnum = 100;
  mStartTime = millis();
  mRunPhase = 1;
  LIB_LOGD(TAG, "Discovery::handle_frame discovery start %d", mStart.slotnum);
}

}  // namespace espmeshmesh
