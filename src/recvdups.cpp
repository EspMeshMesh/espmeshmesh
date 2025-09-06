#include "recvdups.h"
#include "defines.h"
#include "log.h"
#include "espmeshmesh.h"

#ifdef ESP8266
#include <user_interface.h>
#include <osapi.h>
#endif  // ESP8266

#include <cstring>

namespace espmeshmesh {

static const char *TAG = "espmeshmesh.recvdups";

RecvDups::RecvDups() {
    clear();
};

void RecvDups::loop() {
    uint32_t now=millis();
    if(EspMeshMesh::elapsedMillis(now, mDuplicateTableTime) > 300000) {
        mDuplicateTableTime = now;
        for(int i=0; i<mDuplicateTableSize; i++) {
            if(mDuplicates[i].address != TABLE_INVALID_ADDRESS && EspMeshMesh::elapsedMillis(now, mDuplicates[i].time) > 30000) {
                mDuplicates[i].address = TABLE_INVALID_ADDRESS;
            }
        }
    }
}

void RecvDups::clear() {
    memset(&mDuplicates, 0, sizeof(RecvDupPacket)*mDuplicateTableSize);
    for(int i=0; i<mDuplicateTableSize; i++) mDuplicates[i].address = TABLE_INVALID_ADDRESS;
    mDuplicateTableTime = millis();
}

bool RecvDups::checkDuplicateTable(uint32_t address, uint16_t handle, uint16_t seqno) {
    int foundrow = -1;
    int emptyrow = -1;

    for(int i=0; i<mDuplicateTableSize; i++) {
        RecvDupPacket &dup = mDuplicates[i];
        if(dup.address == address && dup.handle == handle) {
            foundrow = i;
            if(emptyrow != -1) break;
        } else if(emptyrow == -1 && dup.address == TABLE_INVALID_ADDRESS) {
            emptyrow = i;
            if(foundrow != -1) break;
        }
    }

    // Is a new client and Found an empty row
    if(foundrow < 0 && emptyrow >= 0) {
        foundrow = emptyrow;
        mDuplicates[foundrow].time = millis();
        mDuplicates[foundrow].address = address;
        mDuplicates[foundrow].handle = handle;
        mDuplicates[foundrow].seqno = seqno;
        return false;
    }

    // Update time of current row
    if(foundrow >= 0) {
        mDuplicates[foundrow].time = millis();
        uint16_t delta = seqno >= mDuplicates[foundrow].seqno ? seqno - mDuplicates[foundrow].seqno : ~(mDuplicates[foundrow].seqno-seqno)+1;
        // Return true if the sequence number is the same for tihs address and handle
        if(mDuplicates[foundrow].seqno == seqno) return true;
        // else return true if received packet si too simular
        else if(seqno < mDuplicates[foundrow].seqno && delta<5) return true;
        // else sequence number is valid
        else mDuplicates[foundrow].seqno = seqno;
    } else {
        LIB_LOGE(TAG, "No more space in duplicate table for %06lX:%04X", address, handle);
    }

    // Otherwise return always false
    return false;
}

}  // namespace espmeshmesh

