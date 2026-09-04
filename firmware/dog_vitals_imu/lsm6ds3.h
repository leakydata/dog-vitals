// Minimal direct-register driver for the LSM6DS3TR-C on the XIAO nRF52840
// Sense. Deliberately not the Seeed library: that one hard-codes FIFO_MODE to
// continuous regardless of the setting, writes FIFO_CTRL4 = 0x09 (which pushes
// datasets 3 and 4 into the FIFO so the read pattern is no longer six words per
// sample), and its FIFO ODR table stops at 200/400 Hz so 208 and 416 -- the two
// rates that actually match the accel/gyro ODRs -- cannot be selected at all.
// Getting the FIFO pattern right matters more here than saving eighty lines.
#pragma once
#include <Arduino.h>
#include <Wire.h>

// The IMU sits on the second I2C bus, powered through a GPIO. Both come from
// the variant; naming them here keeps the sketch readable.
#define IMU_WIRE      Wire1
#define IMU_ADDR      0x6A          // SA0 tied low on this board
#define IMU_PWR_PIN   PIN_LSM6DS3TR_C_POWER

// Registers, only the ones this driver touches.
#define REG_FIFO_CTRL1    0x06
#define REG_FIFO_CTRL2    0x07
#define REG_FIFO_CTRL3    0x08
#define REG_FIFO_CTRL4    0x09
#define REG_FIFO_CTRL5    0x0A
#define REG_WHO_AM_I      0x0F
#define REG_CTRL1_XL      0x10
#define REG_CTRL2_G       0x11
#define REG_CTRL3_C       0x12
#define REG_CTRL4_C       0x13
#define REG_CTRL6_C       0x15
#define REG_CTRL7_G       0x16
#define REG_CTRL8_XL      0x17
#define REG_FIFO_STATUS1  0x3A
#define REG_FIFO_STATUS2  0x3B
#define REG_FIFO_STATUS3  0x3C
#define REG_FIFO_STATUS4  0x3D
#define REG_FIFO_DATA_OUT_L 0x3E

// One FIFO "sample" is six 16-bit words: gyro XYZ then accel XYZ. That order is
// the sensor's, not a choice, and it is what goes out over BLE unchanged.
#define WORDS_PER_SAMPLE  6
#define BYTES_PER_SAMPLE  (WORDS_PER_SAMPLE * 2)

struct ImuConfig {
  uint16_t odr_hz;      // 52, 104, 208, 416
  uint8_t  accel_fs_g;  // 2, 4, 8, 16
  uint16_t gyro_fs_dps; // 245, 500, 1000, 2000
};

class Lsm6ds3 {
public:
  bool begin(const ImuConfig& cfg);
  bool configure(const ImuConfig& cfg);

  // Number of unread 16-bit words sitting in the FIFO.
  uint16_t fifoUnread();
  // True if the FIFO overflowed since the last call; clears the flag.
  bool fifoOverrun();
  // Index of the next word the FIFO will hand back, within the 6-word pattern.
  // Used to resynchronise after an overrun rather than emitting shuffled axes.
  uint8_t fifoPatternPhase();
  // Discards the tail of a partially-consumed sample so the next read starts
  // on an axis boundary. Cheaper than fifoReset(), which throws away every
  // buffered sample -- three seconds of them at 208 Hz.
  bool fifoRealign();
  // Reads n complete samples into dst (n * 12 bytes). Returns samples read.
  uint8_t fifoReadSamples(uint8_t* dst, uint8_t n);
  void fifoReset();

  float accelScaleG() const { return _accel_scale; }   // g per LSB
  float gyroScaleDps() const { return _gyro_scale; }   // dps per LSB
  ImuConfig config() const { return _cfg; }
  uint8_t whoAmI() const { return _whoami; }

private:
  bool  write8(uint8_t reg, uint8_t val);
  bool  read8(uint8_t reg, uint8_t* val);
  bool  readBurst(uint8_t reg, uint8_t* dst, uint8_t len);

  ImuConfig _cfg{208, 2, 245};
  float _accel_scale = 0.000061f;
  float _gyro_scale  = 0.00875f;
  uint8_t _whoami = 0;
  bool _overrun_latch = false;
};
