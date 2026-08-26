#pragma once

#include <cstdint>

#include "Arduino.h"

class PlatformMutex {
 public:
  void lock() noexcept {
    ++lockCount;
    lastLockSequence = ArduinoTest::nextEventSequence();
    locked = true;
  }

  void unlock() noexcept {
    ++unlockCount;
    lastUnlockSequence = ArduinoTest::nextEventSequence();
    locked = false;
  }

  bool trylock() noexcept {
    if (locked) return false;
    lock();
    return true;
  }

  bool locked = false;
  uint64_t lockCount = 0;
  uint64_t unlockCount = 0;
  uint64_t lastLockSequence = 0;
  uint64_t lastUnlockSequence = 0;
};

struct SafePlatformMutex {
  PlatformMutex raw;
  void lock() { raw.lock(); }
  void unlock() { raw.unlock(); }
  bool trylock() { return raw.trylock(); }
};

class LockGuard {
 public:
  explicit LockGuard(SafePlatformMutex& mutex) : mutex_(mutex) {
    mutex_.lock();
  }
  ~LockGuard() { mutex_.unlock(); }

 private:
  SafePlatformMutex& mutex_;
};
