#pragma once
#include <stdint.h>


namespace espmeshmesh {

struct RecvDupPacket {
    uint32_t address;
    uint32_t time;
    uint16_t handle;
    uint16_t seqno;
};

#define TABLE_TABLE_SIZE 0x20
#define TABLE_INVALID_ADDRESS 0xFFFFFFFE
#define TABLE_LAST_ADDRESS 0xFFFFFFFF

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
    RecvDupPacket mDuplicates[TABLE_TABLE_SIZE];
    int mDuplicateTableSize = TABLE_TABLE_SIZE;
    uint32_t mDuplicateTableTime = 0;
#if LIB_LOG_LEVEL >= LIB_LOG_LEVEL_VERBOSE
    bool mDebug = false;
    uint32_t mLastPrintTime = 0;
#endif
};

} // namespace espmeshmesh