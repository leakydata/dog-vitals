// Bit-banged I2C probe, for answering "is the sensor actually there?" without
// betting the firmware on it.
//
// The core's TWIM driver cannot be used for this. It spins on hardware events
// with no timeout -- `while(!_p_twim->EVENTS_STOPPED);` has no error escape at
// all -- so pointing it at a bus that is not working is how this board ended
// up mute on every interface at once. Driving the pins directly is slower and
// entirely bounded, which is the right trade for a diagnostic.
#pragma once
#include <Arduino.h>

namespace i2cdiag {

inline void sclHigh(uint8_t scl) { pinMode(scl, INPUT_PULLUP); }
inline void sclLow (uint8_t scl) { pinMode(scl, OUTPUT); digitalWrite(scl, LOW); }
inline void sdaHigh(uint8_t sda) { pinMode(sda, INPUT_PULLUP); }
inline void sdaLow (uint8_t sda) { pinMode(sda, OUTPUT); digitalWrite(sda, LOW); }

inline void qdelay() { delayMicroseconds(5); }   // ~100 kHz

inline void start(uint8_t sda, uint8_t scl) {
  sdaHigh(sda); sclHigh(scl); qdelay();
  sdaLow(sda);  qdelay();
  sclLow(scl);  qdelay();
}

inline void stop(uint8_t sda, uint8_t scl) {
  sdaLow(sda);  qdelay();
  sclHigh(scl); qdelay();
  sdaHigh(sda); qdelay();
}

// Returns true if the target pulled SDA low for the ACK bit.
inline bool writeByte(uint8_t sda, uint8_t scl, uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    (b & (1 << i)) ? sdaHigh(sda) : sdaLow(sda);
    qdelay();
    sclHigh(scl); qdelay();
    sclLow(scl);  qdelay();
  }
  sdaHigh(sda); qdelay();
  sclHigh(scl); qdelay();
  bool ack = (digitalRead(sda) == LOW);
  sclLow(scl); qdelay();
  return ack;
}

// A single address probe: START, address+W, look for the ACK, STOP.
inline bool probe(uint8_t sda, uint8_t scl, uint8_t addr) {
  start(sda, scl);
  bool ack = writeByte(sda, scl, (uint8_t)(addr << 1));
  stop(sda, scl);
  return ack;
}

// Nine clocks with SDA released, to walk a target off a byte it was in the
// middle of when the MCU reset out from under it.
inline void recover(uint8_t sda, uint8_t scl) {
  sdaHigh(sda);
  for (int i = 0; i < 9; i++) { sclHigh(scl); qdelay(); sclLow(scl); qdelay(); }
  stop(sda, scl);
}

inline void release(uint8_t sda, uint8_t scl) {
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
}

}  // namespace i2cdiag
