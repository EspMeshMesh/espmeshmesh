#pragma once
#include <stdint.h>

namespace espmeshmesh {

#define RSSICHECK_WINDOWS_SIZE 8

class EspMeshMesh;
class RssiSensor;
class RssiCheck {
 public:
  void setup();
  void loop(EspMeshMesh *parent);
  uint8_t handleFrame(const uint8_t *buf, uint16_t size, uint32_t from, int16_t rssi, EspMeshMesh *parent);
#ifdef USE_SENSOR
  void registerRssiSensor(RssiSensor *sensor) { mRssiSensors.push_back(sensor); }
  bool useRssiStatistics() const { return mRssiSensors.size() > 0; }
#else
  void registerRssiSensor(RssiSensor *sensor) {}
  bool useRssiStatistics() const { return false; }
#endif
  void newRssiData(uint32_t address, int16_t rssi);
  bool isRunning() const { return mActiveNodeId > 0; }

 private:
  void clearAverageWindow(uint32_t id);
  void pushAverageData(int16_t remote, int16_t local);
  void calcAverageRssi(int16_t *remote, int16_t *local);

 private:
  uint8_t mAverageWindowIndex = 0;
  int16_t mAverageWindowLocal[RSSICHECK_WINDOWS_SIZE];
  int16_t mAverageWindowRemote[RSSICHECK_WINDOWS_SIZE];
  uint32_t mActiveNodeId = 0;
  uint32_t mCheckStartTime = 0;
  uint32_t mLastSendTime = 0;

 private:
#ifdef USE_SENSOR
  std::vector<RssiSensor *> mRssiSensors;
#endif
};

/// Internal holder class that is in instance of Sensor so that the hub can create individual sensors.
#ifdef USE_SENSOR
class RssiSensor : public sensor::Sensor, public PollingComponent {
 public:
  RssiSensor(MeshmeshComponent *parent) : mParent(parent) {}
  void update() override;
  uint32_t address() const { return mAddress; }
  void newRssiData(uint32_t rssi);

 public:
  void set_address(uint32_t address) { mAddress = address; }

 protected:
  MeshmeshComponent *mParent;
  uint32_t mAddress;
};
#endif

}  // namespace espmeshmesh
