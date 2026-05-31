#pragma once
#include "log.h"
#include <stdint.h>

namespace espmeshmesh {

struct RecvDupPacket {
    uint32_t address;
    uint32_t time;
    uint16_t handle;
    uint16_t seqno;
};

#if defined(ESPMESH_RECV_DUP_TABLE_SIZE)
#define TABLE_TABLE_SIZE ESPMESH_RECV_DUP_TABLE_SIZE
#else
#define TABLE_TABLE_SIZE 0x20
#endif

class RecvDups {
public:
    RecvDups();
    bool checkDuplicateTable(uint32_t address, uint16_t handle, uint16_t seqno);
#if LIB_LOG_LEVEL >= LIB_LOG_LEVEL_VERBOSE
    void setDebug(bool debug) { mDebug = debug; }
    void printDuplicateTable(uint32_t now);
#endif
    void loop();
    void clear();
private:
    void deleteDuplicate(int index);

    RecvDupPacket mDuplicates[TABLE_TABLE_SIZE];
    int mFirstFreeAddressIndex = 0;
    uint32_t mDuplicateTableTime = 0;

#if LIB_LOG_LEVEL >= LIB_LOG_LEVEL_VERBOSE
    bool mDebug = false;
    uint32_t mLastPrintTime = 0;
#endif
};

} // namespace espmeshmesh