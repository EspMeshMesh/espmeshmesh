#include "recvdups.h"

#include "defines.h"
#include "log.h"
#include "espmeshmesh.h"

#include <cstring>

namespace espmeshmesh {

static const char *TAG = "espmeshmesh.recvdups";

static int findOldestIndex(const RecvDupPacket *duplicates, int count, uint32_t now) {
    int oldest = 0;
    for (int i = 1; i < count; i++) {
        if (EspMeshMesh::elapsedMillis(now, duplicates[i].time) >
            EspMeshMesh::elapsedMillis(now, duplicates[oldest].time)) {
            oldest = i;
        }
    }
    return oldest;
}

RecvDups::RecvDups() {
    clear();
};

#if LIB_LOG_LEVEL >= LIB_LOG_LEVEL_VERBOSE
void RecvDups::printDuplicateTable(uint32_t now) {
    LIB_LOGD(TAG, "Duplicate table size: %d", mFirstFreeAddressIndex);
    for(int i=0; i<mFirstFreeAddressIndex; i++) {
        LIB_LOGD(TAG, "Duplicate table[%d]: address: %06lX:%d, maxSeqno: %d, bitmap: 0x%08lX, time: %ld", 
                 i, mDuplicates[i].address, mDuplicates[i].handle, mDuplicates[i].maxSeqno, 
                 mDuplicates[i].seenBitmap, now-mDuplicates[i].time);
    }
}
#endif

void RecvDups::loop() {
    uint32_t now=millis();
    if(EspMeshMesh::elapsedMillis(now, mDuplicateTableTime) > 30000) {
        mDuplicateTableTime = now;
        for (int i = 0; i < mFirstFreeAddressIndex; ) {
            if (EspMeshMesh::elapsedMillis(now, mDuplicates[i].time) > 30000) {
                deleteDuplicate(i);
            } else {
                i++;
            }
        }
    }
#if LIB_LOG_LEVEL >= LIB_LOG_LEVEL_VERBOSE
    if(mDebug && EspMeshMesh::elapsedMillis(now, mLastPrintTime) > 30000) {
        mLastPrintTime = now;
        printDuplicateTable(now);
    }
#endif
}

void RecvDups::clear() {
    memset(&mDuplicates, 0, sizeof(RecvDupPacket)*TABLE_TABLE_SIZE);
    mFirstFreeAddressIndex = 0;
    mDuplicateTableTime = millis();
}

bool RecvDups::checkDuplicateTable(uint32_t address, uint16_t handle, uint16_t seqno) {
    uint32_t now = millis();
    int foundrow = -1;

    // Check if the address is already in the table
    for(int i=0; i<mFirstFreeAddressIndex; i++) {
        if(mDuplicates[i].address == address && mDuplicates[i].handle == handle) {
            foundrow = i;
            break;
        }
    }

    if(foundrow < 0) {
        // New (address, handle) pair - not a duplicate
        if(mFirstFreeAddressIndex >= TABLE_TABLE_SIZE) {
            int oldest = findOldestIndex(mDuplicates, mFirstFreeAddressIndex, now);
            LIB_LOGW(TAG, "Duplicate table full, evicting %06lX:%04X for %06lX:%04X",
                     mDuplicates[oldest].address, mDuplicates[oldest].handle, address, handle);
            deleteDuplicate(oldest);
        }
        foundrow = mFirstFreeAddressIndex;
        mFirstFreeAddressIndex++;
        mDuplicates[foundrow].time = now;
        mDuplicates[foundrow].address = address;
        mDuplicates[foundrow].handle = handle;
        mDuplicates[foundrow].maxSeqno = seqno;
        mDuplicates[foundrow].seenBitmap = 1;  // Bit 0 = current seqno seen
        return false;
    }

    mDuplicates[foundrow].time = now;
    uint16_t maxSeqno = mDuplicates[foundrow].maxSeqno;
    uint32_t bitmap = mDuplicates[foundrow].seenBitmap;

    // Calculate the distance from maxSeqno (handles uint16 wraparound)
    int16_t delta = (int16_t)(seqno - maxSeqno);

    if (delta == 0) {
        // Exact match with maxSeqno - always a duplicate (bit 0 is always set)
        return true;
    }

    if (delta > 0) {
        // New seqno is ahead of maxSeqno - accept and shift bitmap
        if (delta >= 32) {
            // Large jump - reset bitmap (old entries are too stale)
            bitmap = 1;
        } else {
            // Shift bitmap left and mark new seqno as seen
            bitmap = (bitmap << delta) | 1;
        }
        mDuplicates[foundrow].maxSeqno = seqno;
        mDuplicates[foundrow].seenBitmap = bitmap;
        return false;
    }

    // delta < 0: seqno is behind maxSeqno (out-of-order arrival or late retransmit)
    int offset = -delta;  // How many positions behind maxSeqno

    if (offset >= 32) {
        // seqno is too old (outside our tracking window) - treat as stale duplicate
        // This prevents very old packets from being processed
        return true;
    }

    // Check if this specific seqno was already seen
    uint32_t mask = 1U << offset;
    if (bitmap & mask) {
        // Already seen this seqno - duplicate
        return true;
    }

    // First time seeing this seqno (out-of-order but not duplicate) - accept it
    mDuplicates[foundrow].seenBitmap = bitmap | mask;
    return false;
}

void RecvDups::deleteDuplicate(int index) {
    // Check if the index is valid
    if(index < 0 || index >= mFirstFreeAddressIndex) return;
    // If the index is not the last one, move the last one to the index
    if(index != mFirstFreeAddressIndex-1) {
        mDuplicates[index] = mDuplicates[mFirstFreeAddressIndex-1];
    }
    // Decrease the first free address index
    mFirstFreeAddressIndex--;
}

}  // namespace espmeshmesh
