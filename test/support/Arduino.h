#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ArduinoTest {

struct PinWrite {
  uint8_t pin = 0;
  uint8_t value = 0;
  uint32_t atUs = 0;
  uint64_t sequence = 0;
};

inline uint32_t nowUs = 0;
inline uint64_t eventSequence = 0;
inline uint64_t microsCalls = 0;
inline uint64_t millisCalls = 0;
inline uint64_t delayMicrosecondsCalls = 0;
inline uint64_t delayedMicroseconds = 0;
inline std::vector<PinWrite> pinWrites;

inline uint64_t nextEventSequence() {
  return ++eventSequence;
}

inline void reset(uint32_t startUs = 0) {
  nowUs = startUs;
  eventSequence = 0;
  microsCalls = 0;
  millisCalls = 0;
  delayMicrosecondsCalls = 0;
  delayedMicroseconds = 0;
  pinWrites.clear();
}

inline void clearTrace() {
  eventSequence = 0;
  microsCalls = 0;
  millisCalls = 0;
  delayMicrosecondsCalls = 0;
  delayedMicroseconds = 0;
  pinWrites.clear();
}

inline void advance(uint32_t deltaUs) {
  nowUs += deltaUs;
}

} // namespace ArduinoTest

inline unsigned long micros() {
  ++ArduinoTest::microsCalls;
  return ArduinoTest::nowUs;
}

inline unsigned long millis() {
  ++ArduinoTest::millisCalls;
  return ArduinoTest::nowUs / 1000UL;
}

inline void delayMicroseconds(unsigned int delayUs) {
  ++ArduinoTest::delayMicrosecondsCalls;
  ArduinoTest::delayedMicroseconds += delayUs;
  ArduinoTest::advance(delayUs);
}

inline void pinMode(uint8_t, uint8_t) {}

inline void digitalWrite(uint8_t pin, uint8_t value) {
  ArduinoTest::pinWrites.push_back(
      {pin, value, ArduinoTest::nowUs, ArduinoTest::nextEventSequence()});
}

class Stream {
 public:
  virtual ~Stream() = default;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual std::size_t write(const uint8_t* data, std::size_t len) = 0;
  virtual void flush() = 0;
};

class HardwareSerial : public Stream {
 public:
  virtual void begin(unsigned long baud) = 0;
  virtual void begin(unsigned long baud, uint32_t config) = 0;
};

#ifndef LOW
#define LOW 0x0
#endif
#ifndef HIGH
#define HIGH 0x1
#endif
#ifndef OUTPUT
#define OUTPUT 0x1
#endif

#define SERIAL_8N1 0x06u
#define SERIAL_8N2 0x0Eu
#define SERIAL_8E1 0x26u
#define SERIAL_8E2 0x2Eu
#define SERIAL_8O1 0x36u
#define SERIAL_8O2 0x3Eu

#define highByte(value) static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFFu)
#define lowByte(value) static_cast<uint8_t>(static_cast<uint16_t>(value) & 0xFFu)
#define bitRead(value, bit) (((value) >> (bit)) & 0x01u)
