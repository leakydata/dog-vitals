#include "lsm6ds3.h"
#include <nrf_gpio.h>

// The Wire receive buffer on this core is SERIAL_BUFFER_SIZE (64) bytes, so a
// FIFO drain has to be chunked. 48 is the largest multiple of a 12-byte sample
// that fits, which keeps every I2C transaction sample-aligned -- handy when
// debugging with a logic analyser, and it costs nothing.
static const uint8_t I2C_CHUNK = 48;

bool Lsm6ds3::write8(uint8_t reg, uint8_t val) {
  IMU_WIRE.beginTransmission(IMU_ADDR);
  IMU_WIRE.write(reg);
  IMU_WIRE.write(val);
  return IMU_WIRE.endTransmission() == 0;
}

bool Lsm6ds3::read8(uint8_t reg, uint8_t* val) {
  return readBurst(reg, val, 1);
}

bool Lsm6ds3::readBurst(uint8_t reg, uint8_t* dst, uint8_t len) {
  IMU_WIRE.beginTransmission(IMU_ADDR);
  IMU_WIRE.write(reg);
  if (IMU_WIRE.endTransmission(false) != 0) return false;   // repeated start
  uint8_t got = IMU_WIRE.requestFrom((uint8_t)IMU_ADDR, len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) dst[i] = IMU_WIRE.read();
  return true;
}

// ODR_XL and ODR_G share an encoding, and so does ODR_FIFO.
static uint8_t odrCode(uint16_t hz) {
  switch (hz) {
    case 52:  return 0x3;
    case 104: return 0x4;
    case 416: return 0x6;
    case 208:
    default:  return 0x5;
  }
}

// FS_XL is not in ascending order in the register -- 2g, 16g, 4g, 8g -- which
// is exactly the sort of thing a lookup makes safe and a shift makes wrong.
static uint8_t accelFsCode(uint8_t g) {
  switch (g) {
    case 4:  return 0x2;
    case 8:  return 0x3;
    case 16: return 0x1;
    case 2:
    default: return 0x0;
  }
}

static float accelScaleFor(uint8_t g) {
  switch (g) {
    case 4:  return 0.000122f;
    case 8:  return 0.000244f;
    case 16: return 0.000488f;
    case 2:
    default: return 0.000061f;
  }
}

static uint8_t gyroFsCode(uint16_t dps) {
  switch (dps) {
    case 500:  return 0x1;
    case 1000: return 0x2;
    case 2000: return 0x3;
    case 245:
    default:   return 0x0;
  }
}

static float gyroScaleFor(uint16_t dps) {
  switch (dps) {
    case 500:  return 0.0175f;
    case 1000: return 0.035f;
    case 2000: return 0.070f;
    case 245:
    default:   return 0.00875f;
  }
}

// This core's TWIM driver spins on hardware events with no timeout and, for
// EVENTS_STOPPED, no error escape either:
//
//   while(!_p_twim->EVENTS_STOPPED);
//
// So a device that is absent, unpowered, or mid-transfer when the nRF52 reset
// does not produce a failed read -- it produces a firmware that never reaches
// its second line. Everything below exists to avoid handing that driver a bus
// it cannot finish on.

// Clocks out up to nine pulses to free a peripheral that is holding SDA low,
// then issues a STOP. This is the standard recovery for a target left
// mid-byte by an MCU reset, which is exactly what happens when reflashing a
// board whose IMU was streaming at the time.
static void i2cBusRecover(uint8_t sda, uint8_t scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT);
  digitalWrite(scl, HIGH);
  for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
    digitalWrite(scl, LOW);  delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
  }
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);  delayMicroseconds(5);
  digitalWrite(scl, HIGH); delayMicroseconds(5);
  digitalWrite(sda, HIGH); delayMicroseconds(5);
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
}

// Both lines pulled up and idle: the only state the TWIM driver can safely be
// pointed at. If this is false after recovery, the IMU is reported dead rather
// than risking a hang -- a board that advertises and says "no IMU" can be
// diagnosed from across the room; one that hangs cannot be diagnosed at all.
static bool i2cBusIdle(uint8_t sda, uint8_t scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delayMicroseconds(50);
  return digitalRead(sda) == HIGH && digitalRead(scl) == HIGH;
}

// Brings up the IMU's supply rail on P1.08 -- with high drive, which is the
// whole trick.
//
// A plain pinMode(OUTPUT) cannot source enough current to start the sensor.
// The pin comes up, measures low, and the entire IMU section stays dead: the
// bus pull-ups sit on this rail, so SCL, SDA and INT1 all read low as well and
// the failure presents as four shorted pins rather than as a supply that never
// came up. Nothing in the board variant header hints at this, and the Zephyr
// port of this board hits it too -- its devicetree carries the same fix as
// `NRF_GPIO_DRIVE_S0H1` on the enable pin.
//
// Arduino's pinMode has no way to express drive strength, so this goes
// straight to the GPIO config register.
static void imuPowerOn() {
  const uint32_t pin = g_ADigitalPinMap[IMU_PWR_PIN];
  nrf_gpio_cfg(pin,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_H0H1,          // high drive both rails
               NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_set(pin);
}

bool Lsm6ds3::begin(const ImuConfig& cfg) {
  imuPowerOn();
  delay(50);                       // ~3 ms for the rail, ~35 ms for the part

  if (!i2cBusIdle(PIN_WIRE1_SDA, PIN_WIRE1_SCL)) {
    i2cBusRecover(PIN_WIRE1_SDA, PIN_WIRE1_SCL);
    if (!i2cBusIdle(PIN_WIRE1_SDA, PIN_WIRE1_SCL)) return false;
  }

  IMU_WIRE.begin();
  IMU_WIRE.setClock(400000);
  delay(10);

  if (!read8(REG_WHO_AM_I, &_whoami)) return false;
  // 0x6A is the TR-C fitted to the XIAO Sense; 0x69 is the older LSM6DS3. The
  // register map this driver uses is common to both.
  if (_whoami != 0x6A && _whoami != 0x69) return false;

  // Block data update on, register auto-increment on. BDU matters: without it
  // a burst read can straddle a sensor update and pair a new low byte with an
  // old high byte, which shows up as isolated huge spikes -- indistinguishable
  // from a real impulse, and fatal to peak detection.
  if (!write8(REG_CTRL3_C, 0x44)) return false;
  write8(REG_CTRL4_C, 0x00);
  write8(REG_CTRL6_C, 0x00);   // accel high-performance mode
  write8(REG_CTRL7_G, 0x00);   // gyro high-performance mode, HPF off
  write8(REG_CTRL8_XL, 0x00);  // no LPF2/HPF on the accel path

  return configure(cfg);
}

bool Lsm6ds3::configure(const ImuConfig& cfg) {
  _cfg = cfg;
  _accel_scale = accelScaleFor(cfg.accel_fs_g);
  _gyro_scale  = gyroScaleFor(cfg.gyro_fs_dps);

  const uint8_t odr = odrCode(cfg.odr_hz);

  // BW0_XL = 1 selects the 400 Hz analog anti-aliasing filter. At 208 Hz ODR
  // that is comfortably above the ~40 Hz top of the cardiac band while still
  // filtering before the sampler, which is the point of it.
  write8(REG_CTRL1_XL, (uint8_t)((odr << 4) | (accelFsCode(cfg.accel_fs_g) << 2) | 0x01));
  write8(REG_CTRL2_G,  (uint8_t)((odr << 4) | (gyroFsCode(cfg.gyro_fs_dps) << 2)));

  // Gyro and accel into the FIFO with no decimation (code 001 each), and
  // nothing else: FIFO_CTRL4 stays zero so datasets 3 and 4 are excluded and
  // the read pattern is exactly six words per sample.
  write8(REG_FIFO_CTRL1, 0x00);
  write8(REG_FIFO_CTRL2, 0x00);
  write8(REG_FIFO_CTRL3, (uint8_t)((0x1 << 3) | 0x1));
  write8(REG_FIFO_CTRL4, 0x00);

  fifoReset();
  return true;
}

void Lsm6ds3::fifoReset() {
  const uint8_t odr = odrCode(_cfg.odr_hz);
  write8(REG_FIFO_CTRL5, (uint8_t)(odr << 3) | 0x0);  // bypass: drops contents
  delayMicroseconds(200);
  write8(REG_FIFO_CTRL5, (uint8_t)(odr << 3) | 0x6);  // continuous
  _overrun_latch = false;
}

uint16_t Lsm6ds3::fifoUnread() {
  uint8_t s[2];
  if (!readBurst(REG_FIFO_STATUS1, s, 2)) return 0;
  if (s[1] & 0x40) _overrun_latch = true;
  return (uint16_t)s[0] | ((uint16_t)(s[1] & 0x0F) << 8);
}

bool Lsm6ds3::fifoOverrun() {
  bool v = _overrun_latch;
  _overrun_latch = false;
  return v;
}

uint8_t Lsm6ds3::fifoPatternPhase() {
  uint8_t s[2];
  if (!readBurst(REG_FIFO_STATUS3, s, 2)) return 0;
  uint16_t pattern = (uint16_t)s[0] | ((uint16_t)(s[1] & 0x03) << 8);
  return (uint8_t)(pattern % WORDS_PER_SAMPLE);
}

bool Lsm6ds3::fifoRealign() {
  const uint8_t phase = fifoPatternPhase();
  if (phase == 0) return true;
  uint8_t junk[WORDS_PER_SAMPLE * 2];
  const uint8_t words = WORDS_PER_SAMPLE - phase;
  return readBurst(REG_FIFO_DATA_OUT_L, junk, (uint8_t)(words * 2));
}

uint8_t Lsm6ds3::fifoReadSamples(uint8_t* dst, uint8_t n) {
  uint16_t want = (uint16_t)n * BYTES_PER_SAMPLE;
  uint16_t done = 0;
  while (done < want) {
    uint8_t chunk = (uint8_t)min((uint16_t)I2C_CHUNK, (uint16_t)(want - done));
    if (!readBurst(REG_FIFO_DATA_OUT_L, dst + done, chunk)) break;
    done += chunk;
  }
  return (uint8_t)(done / BYTES_PER_SAMPLE);
}
