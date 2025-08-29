#include "memringbuffer.h"
#include "log.h"

#include <memory.h>

namespace espmeshmesh {

static const char *TAG = "espmeshmesh.memringbuffer";

MemRingBuffer::MemRingBuffer(uint16_t size) {
  if (size > 0)
    resize(size);
}

MemRingBuffer::~MemRingBuffer() {
  if (mBuffer)
    deallocateSpace();
}

void MemRingBuffer::pushData(const uint8_t *data, uint16_t size) {
  // esphome::LIB_LOGD(TAG, "MemRingBuffer::pushData size %d alloc %d free %d", size, allocatedSpace(), freeSpace());
  if (size <= freeSpace()) {
    uint16_t contfreespace = continousFreeSpace();
    if (contfreespace > size)
      contfreespace = size;
    pushDataUnsafe(data, contfreespace);

    size -= contfreespace;
    if (size > 0) {
      mTail = mBuffer;
      pushDataUnsafe(data + contfreespace, size);
    }
  }
}

void MemRingBuffer::pushByte(uint8_t byte) {
  if (mSize < mAllocatedSize) {
    *mTail++ = byte;
    if (mTail == mBuffer + mAllocatedSize)
      mTail = mBuffer;
    mSize++;
  }
}

uint16_t MemRingBuffer::popData(uint8_t *data, uint16_t size) {
  // esphome::LIB_LOGD(TAG, "MemRingBuffer::popData size %d alloc %d fill %d", size, allocatedSpace(), filledSpace());
  if (filledSpace() < size || size == 0)
    size = filledSpace();
  uint16_t readed = size;

  int16_t ss = (mTail - mBuffer) - (mHead - mBuffer);
  if (ss < 0)
    ss = mAllocatedSize + ss;
  // Serial1.printf("**%d %d %d==%d\n", mHead-mBuffer, mTail-mBuffer, ss, mSize);

  uint16_t contfillspace = continousFilledSpace();
  if (contfillspace > size)
    contfillspace = size;
  popDataUnsafe(data, contfillspace);

  // Serial1.printf("*%d %d %d\n",size, contfillspace, size-contfillspace);
  size -= contfillspace;
  if (size > 0) {
    mHead = mBuffer;
    popDataUnsafe(data + contfillspace, size);
  }

  return readed;
}

uint8_t MemRingBuffer::popByte() {
  if (mSize > 0) {
    uint8_t byte = *mHead++;
    if (mHead == mBuffer + mAllocatedSize)
      mHead = mBuffer;
    mSize--;
    return byte;
  }
  return 0;
}

uint16_t MemRingBuffer::viewData(uint8_t *data, uint16_t size) const {
  // esphome::LIB_LOGD(TAG, "MemRingBuffer::popData size %d alloc %d fill %d", size, allocatedSpace(), filledSpace());
  if (filledSpace() < size || size == 0)
    size = filledSpace();
  uint16_t readed = size;

  uint16_t contfillspace = continousFilledSpace();
  if (contfillspace > size)
    contfillspace = size;
  memcpy(data, mHead, contfillspace);

  size -= contfillspace;
  if (size > 0) {
    memcpy(data + contfillspace, mBuffer, size);
  }

  return readed;
}

uint16_t MemRingBuffer::viewData2(uint8_t *data, uint16_t size, uint16_t offset) const {
  // If offset is greater than the filled space, return 0
  if (offset > filledSpace()) {
    return 0;
  } else if (offset + size > filledSpace()) {
    // If the offset + size is greater than the filled space, reduce the size to the available data for reading
    size = filledSpace() - offset;
  }

  uint8_t readed = size;
  // Recalculate offset to be relative to the buffer start
  offset = ((mHead - mBuffer) + offset) % mAllocatedSize;
  // Calculate the virtual head of the buffer
  uint8_t *myHead = mBuffer + offset;
  // Return the number of bytes that can be read from the current head to the tail of the buffer without wrapping.
  uint16_t contfillspace = continousFilledSpace2(myHead, mTail);
  // If the number of bytes that can be read is greater than the requested size, read the requested size
  if (contfillspace > size)
    contfillspace = size;
  // Copy the data from the virtual head to the requested data
  memcpy(data, myHead, contfillspace);
  // Note: contfillspace is always less than or equal to size, we can safely subtract it from size.
  size -= contfillspace;
  if (size > 0) {
    // If there is more data to read, copy the data from the buffer start to the requested data
    memcpy(data + contfillspace, mBuffer, size);
  }

  return readed;
}

void MemRingBuffer::resize(uint16_t size) {
  LIB_LOGD(TAG, "MemRingBuffer::resize curr %d new %d", mSize, size);
  if (mSize == size)
    return;
  if (mSize)
    deallocateSpace();
  if (size)
    allocateSpace(size);
}

void MemRingBuffer::deallocateSpace() {
  LIB_LOGD(TAG, "MemRingBuffer::deallocateSpace free %d bytes", mSize);
  delete mBuffer;
  mBuffer = nullptr;
  mTail = mHead = nullptr;
  mSize = mAllocatedSize = 0;
}

void MemRingBuffer::allocateSpace(uint16_t size) {
  mBuffer = new uint8_t[size];
  memset(mBuffer, 0x00, size);
  mTail = mHead = mBuffer;
  mAllocatedSize = size;
  mSize = 0;
}

void MemRingBuffer::pushDataUnsafe(const uint8_t *data, uint16_t size) {
  memcpy(mTail, data, size);
  mTail += size;
  mSize += size;
}

void MemRingBuffer::popDataUnsafe(uint8_t *data, uint16_t size) {
  memcpy(data, mHead, size);
  mHead += size;
  mSize -= size;
}

}  // namespace espmeshmesh
