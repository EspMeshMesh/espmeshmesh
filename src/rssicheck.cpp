#include "rssicheck.h"
#include "commands.h"
#include "espmeshmesh.h"

#define CMD_RSSISTARTCHECK_REQ 0x02
#define CMD_RSSISTARTCHECK_REP 0x03
#define CMD_RSSIROUNDTRIP_REQ 0x04
#define CMD_RSSIROUNDTRIP_REP 0x05

namespace espmeshmesh {

// static const char *TAG = "espmeshmesh.rssicheck";

void RssiCheck::setup() { clearAverageWindow(0); }

void RssiCheck::loop(EspMeshMesh *parent) {
  // uint32_t now = millis();

  // Execute RSSI check
  if (mActiveNodeId > 0) {
    uint32_t now = millis();
    if (EspMeshMesh::elapsedMillis(now, mCheckStartTime) > 30000) {
      mActiveNodeId = 0;
    } else if (EspMeshMesh::elapsedMillis(now, mLastSendTime) > 100) {
      uint8_t buf[2];
      buf[0] = CMD_RSSICHECK_REQ;
      buf[1] = CMD_RSSIROUNDTRIP_REQ;
      parent->uniCastSendData(buf, 2, mActiveNodeId);
    }
  }
}

uint8_t RssiCheck::handleFrame(const uint8_t *buf, uint16_t size, uint32_t from, int16_t rssi,
                               EspMeshMesh *parent) {
  uint8_t err = 1;
  switch (buf[0]) {
    case CMD_RSSISTARTCHECK_REQ:
      if (size == 5) {
        uint32_t activenodeid = uint32FromBuffer(buf + 1, 4);
        if (mActiveNodeId != activenodeid) {
          clearAverageWindow(activenodeid);
          mCheckStartTime = mLastSendTime = millis();
        }

        int16_t local, remote;
        calcAverageRssi(&remote, &local);

        uint8_t rep[6];
        rep[0] = CMD_RSSICHECK_REP;
        rep[1] = CMD_RSSISTARTCHECK_REP;
        uint16toBuffer(rep + 2, remote);
        uint16toBuffer(rep + 4, remote);
        parent->commandReply(rep, 6);
        err = 0;
      }
      break;
    case CMD_RSSIROUNDTRIP_REQ:
      if (size == 1) {
        uint8_t rep[4];
        rep[0] = CMD_RSSICHECK_REP;
        rep[1] = CMD_RSSIROUNDTRIP_REP;
        uint16toBuffer(rep + 2, rssi);
        parent->commandReply(rep, 4);
        err = 0;
      }
      break;
    case CMD_RSSIROUNDTRIP_REP:
      if (size == 3) {
        if (from == mActiveNodeId) {
          int16_t remote = uint16FromBuffer(buf + 1);
          pushAverageData(remote, rssi);
        }
        err = 0;
      }
      break;
  }
  return err;
}

#ifdef USE_SENSOR
void RssiCheck::newRssiData(uint32_t address, int16_t rssi) {
  for (auto *sensor : mRssiSensors) {
    if (sensor->address() == address) {
      sensor->newRssiData(rssi);
    }
  }
}
#endif

void RssiCheck::clearAverageWindow(uint32_t id) {
  mActiveNodeId = id;
  mAverageWindowIndex = 0;
  for (int i = 0; i < RSSICHECK_WINDOWS_SIZE; i++) {
    mAverageWindowLocal[i] = 0;
    mAverageWindowRemote[i] = 0;
  }
}

void RssiCheck::pushAverageData(int16_t remote, int16_t local) {
  mAverageWindowLocal[mAverageWindowIndex] = local;
  mAverageWindowRemote[mAverageWindowIndex++] = remote;
  if (mAverageWindowIndex >= RSSICHECK_WINDOWS_SIZE)
    mAverageWindowIndex = 0;
}

void RssiCheck::calcAverageRssi(int16_t *remote, int16_t *local) {
  int32_t _remote = 0, _local = 0;

  for (int i = 0; i < RSSICHECK_WINDOWS_SIZE; i++) {
    if (mAverageWindowRemote[i] == 0 || mAverageWindowLocal[i] == 0)
      break;
    _remote += mAverageWindowRemote[i];
    _local += mAverageWindowLocal[i];
  }

  *remote = (int16_t) (_remote / RSSICHECK_WINDOWS_SIZE);
  *local = (int16_t) (_local / RSSICHECK_WINDOWS_SIZE);
}

#ifdef USE_SENSOR
void RssiSensor::update() {}

void RssiSensor::newRssiData(uint32_t rssi) { publish_state(rssi); }
#endif

}  // namespace espmeshmesh
