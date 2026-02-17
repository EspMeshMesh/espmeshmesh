#pragma once
#include <stdint.h>
#include "meshaddress.h"

#define DISCOVERY_TABLE_SIZE 64

namespace espmeshmesh {

struct CmdStartCompat_st {
  uint8_t cmd1;
  uint8_t mask;
  uint8_t filter;
  uint8_t slotnum;
} __attribute__((packed));
typedef struct CmdStartCompat_st CmdStartCompat_t;

struct CmdStart_st {
  uint8_t cmd1;
  uint8_t cmd2;
  uint8_t mask;
  uint8_t filter;
  uint8_t slotnum;
} __attribute__((packed));
typedef struct CmdStart_st CmdStart_t;

struct BaconsDataCompat_st {
  uint8_t reply;
  uint32_t id;
  int16_t rssi;
} __attribute__((packed));
typedef struct BaconsDataCompat_st BaconsDataCompat_t;

struct BaconsData_st {
  uint8_t reply1;
  uint8_t reply2;
  int16_t rssi;
  uint32_t id;
} __attribute__((packed));
typedef struct BaconsData_st BaconsData_t;

struct DiscoveryItem_st {
  uint32_t id;
  int16_t rssi1;
  int16_t rssi2;
  uint16_t flags;
} __attribute__((packed));
typedef struct DiscoveryItem_st DiscoveryItem_t;

struct CmdAssociate_st {
  uint8_t cmd1;
  uint8_t cmd2;
  uint32_t nodeid;
  uint32_t coordinator;
  int16_t rssi[3];
  uint32_t nid[3];
} __attribute__((packed));
typedef struct CmdAssociate_st CmdAssociate_t;

class Discovery {
 public:
  void init();
  void loop();
  bool isRunning() const { return mRunPhase > 0; }
  void process_beacon(uint32_t id, int16_t rssi1, int16_t rssi2);
  void clear_table(void);
  uint8_t handle_frame(const uint8_t *buf, uint16_t len, const MeshAddress &from, int16_t rssi);
  void discoveryStart(const uint8_t *buf, uint16_t len);
  void discoveryStart(uint8_t slotnum = 100);

 private:
  void findMaxRssi(int16_t max, int16_t &maxRssi, uint32_t &maxRssiNodeId);

 private:
  uint8_t discovery_table_index = 0;
  DiscoveryItem_t discovery_table[DISCOVERY_TABLE_SIZE];
  CmdStart_st mStart;
  uint32_t mStartTime = 0;
  uint32_t mBeaconDelay;
  uint8_t mRunPhase = 0;
  int16_t mLastRssi = 0;
  MeshAddress mLastAddress;
};

}  // namespace espmeshmesh
