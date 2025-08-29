#pragma once
#include <stdint.h>

namespace espmeshmesh {

class MemRingBuffer {
 public:
  explicit MemRingBuffer(uint16_t size = 0);
  virtual ~MemRingBuffer();
  void pushData(const uint8_t *data, uint16_t size);
  void pushByte(uint8_t byte);
  uint16_t popData(uint8_t *data, uint16_t size);
  uint8_t popByte();
  uint16_t viewData(uint8_t *data, uint16_t size) const;
  uint16_t viewData2(uint8_t *data, uint16_t size, uint16_t offset) const;
  void resize(uint16_t size);

 public:
  uint16_t allocatedSpace() const { return mAllocatedSize; }
  uint16_t filledSpace() const { return mSize; }
  uint16_t freeSpace() const { return mAllocatedSize - mSize; }

 private:
  void deallocateSpace();
  void allocateSpace(uint16_t size);
  void pushDataUnsafe(const uint8_t *data, uint16_t size);
  void popDataUnsafe(uint8_t *data, uint16_t size);

 private:
  uint16_t continousFreeSpace() const {
    uint8_t *endofbuff = mBuffer + mAllocatedSize;
    if (mHead <= mTail)
      return (uint16_t) (endofbuff - mTail);
    else
      return (uint16_t) (mHead - mTail);
  }

  uint16_t continousFilledSpace() const {
    uint8_t *endofbuff = mBuffer + mAllocatedSize;
    if (mHead <= mTail)
      return (uint16_t) (mTail - mHead);
    else
      return (uint16_t) (endofbuff - mHead);
  }

  uint16_t continousFilledSpace2(uint8_t *head, uint8_t *tail) const {
    uint8_t *endofbuff = mBuffer + mAllocatedSize;
    if (head <= tail)
      return (uint16_t) (tail - head);
    else
      return (uint16_t) (endofbuff - head);
  }

 private:
  uint8_t *mBuffer = nullptr;
  uint8_t *mHead = nullptr;
  uint8_t *mTail = nullptr;
  uint16_t mAllocatedSize = 0;
  uint16_t mSize = 0;
};

}  // namespace espmeshmesh
