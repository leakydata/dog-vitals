// Dog vitals: raw IMU streaming over BLE for respiration and cardiac work.
//
// Board: Seeed XIAO nRF52840 Sense (LSM6DS3TR-C on the internal I2C bus).
// FQBN:  Seeeduino:nrf52:xiaonRF52840Sense
//
// The job here is to get *unprocessed* motion off the device with trustworthy
// timing. Breathing sits around 0.1-1 Hz (much higher for a panting dog) and
// the cardiac signal -- carotid pulsation picked up as micro-motion -- lives
// roughly 4-30 Hz at a few milli-g. Both are far below anything an IMU is
// normally asked to resolve, so the two things that actually decide whether
// this works are resolution (hence +/-2 g, 0.061 mg/LSB) and sample timing
// that does not wobble. All filtering and rate estimation happens on the host,
// where it can be re-run against a recording with different parameters. That
// is the whole point of streaming raw.
//
// Timing comes from the sensor's own FIFO rather than from when this firmware
// got round to reading it: every sample is one ODR period after the last, and
// the packet header carries the running sample index so the host can rebuild
// the time base exactly even across a dropped packet.

#include <bluefruit.h>
#include "lsm6ds3.h"
#include "protocol.h"
#include "i2cdiag.h"
#include <nrf_gpio.h>

// ---------------------------------------------------------------- BLE UUIDs

// da7a0000-1e3f-4b2c-9a5d-6f8e0c1b2a34, Nordic-style: one base, the 16-bit
// field at [13:12] selects service vs characteristic. Bluefruit wants the
// bytes least-significant first, which is why this reads backwards.
static uint8_t UUID_BASE[16] = {
  0x34, 0x2A, 0x1B, 0x0C, 0x8E, 0x6F, 0x5D, 0x9A,
  0x2C, 0x4B, 0x3F, 0x1E, 0x00, 0x00, 0x7A, 0xDA
};
static uint8_t uuidSvc[16], uuidData[16], uuidCtrl[16], uuidStat[16];

static void makeUuid(uint8_t* dst, uint16_t id) {
  memcpy(dst, UUID_BASE, 16);
  dst[12] = (uint8_t)(id & 0xFF);
  dst[13] = (uint8_t)(id >> 8);
}

BLEService        svcImu(uuidSvc);
BLECharacteristic chrData(uuidData);
BLECharacteristic chrCtrl(uuidCtrl);
BLECharacteristic chrStat(uuidStat);
BLEDis  bledis;
BLEBas  blebas;

// ------------------------------------------------------------------- state

Lsm6ds3 imu;
static ImuConfig cfg = { 208, 2, 245 };

static volatile bool g_notify_on = false;
static bool     g_stream_req = true;      // host can veto with CMD_STREAM
static bool     g_imu_ok = false;
static uint8_t  g_imu_tries = 0;
static uint32_t g_imu_next_try = 1200;   // let the console attach first
static uint32_t g_sample_idx = 0;
static uint32_t g_stream_t0_ms = 0;
static uint16_t g_seq = 0;
static uint16_t g_overruns = 0;
static uint16_t g_misaligns = 0;
static uint16_t g_depth_max = 0;
static uint16_t g_tx_drops = 0;
static uint8_t  g_pkt_samples = MAX_SAMPLES_PER_PKT;
static uint16_t g_mtu = BLE_GATT_ATT_MTU_DEFAULT;
static bool     g_csv = false;
static uint32_t g_identify_until = 0;

// A packet the stack refused. Held rather than dropped: the samples are
// already out of the FIFO, so discarding them would punch a hole in the
// recording that the sample index would faithfully report but nothing could
// fill. Retried before anything new is drained.
static uint8_t  g_pending[sizeof(PktHeader) + MAX_SAMPLES_PER_PKT * BYTES_PER_SAMPLE];
static uint16_t g_pending_len = 0;

// USB and BLE are independent consumers. Gating the sampler on a BLE
// subscription would mean the wired capture path -- the one that is not
// subject to radio drops, and the one to trust when a recording disagrees with
// itself -- only worked while a browser happened to be connected.
// Restarts the stream's time base. g_stream_t0_ms is what makes the sample
// index recoverable after data is lost on the device -- see streamTask().
static void resetStream() {
  g_sample_idx = 0;
  g_seq = 0;
  g_pending_len = 0;
  g_stream_t0_ms = millis();
  imu.fifoReset();
}

static inline bool streaming() {
  return g_imu_ok && g_stream_req && (g_notify_on || g_csv);
}

// ------------------------------------------------------------------- LEDs

// The XIAO's LEDs are wired to 3V3 through the die, so LOW lights them.
static void xledOff(uint8_t pin) { digitalWrite(pin, HIGH); }
static void xledOn (uint8_t pin) { digitalWrite(pin, LOW);  }

static void ledsInit() {
  pinMode(LED_RED, OUTPUT);   xledOff(LED_RED);
  pinMode(LED_GREEN, OUTPUT); xledOff(LED_GREEN);
  pinMode(LED_BLUE, OUTPUT);  xledOff(LED_BLUE);
}

// Heartbeat rather than steady-on: this ends up taped to a neck and then in a
// collar enclosure, where a lit LED is both wasted current and a nuisance.
static void ledTask() {
  static uint32_t last = 0;
  static bool lit = false;
  uint32_t now = millis();

  if (now < g_identify_until) {            // host asked "which one is it?"
    if (now - last >= 100) { last = now; lit = !lit; lit ? xledOn(LED_GREEN) : xledOff(LED_GREEN); }
    return;
  }

  uint32_t period = streaming() ? 2000 : (Bluefruit.connected() ? 3000 : 1500);
  uint8_t  pin    = streaming() ? LED_BLUE : LED_RED;

  if (lit && now - last >= 15) { xledOff(LED_RED); xledOff(LED_GREEN); xledOff(LED_BLUE); lit = false; last = now; }
  else if (!lit && now - last >= period) { xledOn(pin); lit = true; last = now; }
}

// ---------------------------------------------------------------- battery

// P0.14 gates the divider so it is not a permanent 2.3 uA drain, and the
// divider is 1M/510k, hence the 1510/510 factor against the 3.0 V reference.
static uint16_t readBatteryMv() {
  digitalWrite(VBAT_ENABLE, LOW);
  delayMicroseconds(200);
  analogRead(PIN_VBAT);                        // discard: first conversion after
                                               // switching the input is unsettled
  uint32_t acc = 0;
  for (int i = 0; i < 8; i++) acc += analogRead(PIN_VBAT);
  digitalWrite(VBAT_ENABLE, HIGH);

  float raw = acc / 8.0f;
  float v = raw * (3.0f / 4096.0f) * (1510.0f / 510.0f);
  return (uint16_t)(v * 1000.0f);
}

static uint8_t battPercent(uint16_t mv) {
  if (mv >= 4150) return 100;
  if (mv <= 3300) return 0;
  return (uint8_t)((mv - 3300) * 100 / (4150 - 3300));
}

// ------------------------------------------------------------------ status

static uint16_t g_batt_mv = 0;

static void publishStatus() {
  StatusPkt s = {};
  s.ver         = PROTO_VERSION;
  s.flags       = (streaming() ? 0x01 : 0x00) | (g_imu_ok ? 0x02 : 0x00);
  s.odr_hz      = cfg.odr_hz;
  s.accel_fs_g  = cfg.accel_fs_g;
  s.gyro_fs_dps = cfg.gyro_fs_dps;
  s.accel_scale = imu.accelScaleG();
  s.gyro_scale  = imu.gyroScaleDps();
  s.uptime_ms   = millis();
  s.batt_mv     = g_batt_mv;
  s.overruns    = g_overruns;
  s.tx_drops    = g_tx_drops;
  s.mtu         = g_mtu;

  chrStat.write(&s, sizeof(s));
  if (Bluefruit.connected()) chrStat.notify(&s, sizeof(s));
}

// ---------------------------------------------------------------- callbacks

// Fits as many samples per notification as the negotiated MTU allows.
//
// Must not be called from the connect callback. The ATT MTU exchange is a
// separate transaction that the central starts *after* the link is up, so at
// connect time getMtu() still reports the 23-byte default -- which leaves room
// for a 12-byte header and eight bytes of payload, i.e. a single sample per
// notification. That is 208 notifications a second instead of 11, and it does
// not fit: the first run this way lost 634 samples in 20 seconds to FIFO
// overruns while reporting a perfectly healthy zero tx_drops.
static void updatePacketSize() {
  uint16_t mtu = BLE_GATT_ATT_MTU_DEFAULT;
  BLEConnection* conn = Bluefruit.Connection(Bluefruit.connHandle());
  if (conn && conn->connected()) mtu = conn->getMtu();
  g_mtu = mtu;
  const int fit = ((int)mtu - 3 - (int)sizeof(PktHeader)) / BYTES_PER_SAMPLE;
  g_pkt_samples = (uint8_t)constrain(fit, 1, MAX_SAMPLES_PER_PKT);
}

static void onConnect(uint16_t handle) {
  BLEConnection* conn = Bluefruit.Connection(handle);
  conn->requestConnectionParameter(6, 0, 400);   // 7.5 ms interval, 4 s timeout
  xledOn(LED_GREEN); delay(50); xledOff(LED_GREEN);
}

static void onDisconnect(uint16_t handle, uint8_t reason) {
  (void)handle; (void)reason;
  g_notify_on = false;
  g_pending_len = 0;
}

// Streaming follows the CCCD rather than running unconditionally: an idle
// advertising board that is not being listened to should not be burning 208
// samples a second through the I2C bus and the radio.
static void onCccd(uint16_t handle, BLECharacteristic* chr, uint16_t value) {
  (void)handle;
  if (chr->uuid == chrData.uuid) {
    g_notify_on = (value & 0x0001) != 0;
    if (g_notify_on) {
      updatePacketSize();
      resetStream();
      // Publish straight away rather than waiting for the periodic update.
      // The MTU only becomes real after the exchange, so a status block read
      // at connect time always says 23 -- and a host that trusts it reports a
      // link far worse than the one it actually has.
      publishStatus();
    }
  }
}

static void applyConfig() {
  imu.configure(cfg);
  resetStream();
  publishStatus();
}

static void onCtrlWrite(uint16_t handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)handle; (void)chr;
  if (len < 1) return;
  const uint8_t arg = (len > 1) ? data[1] : 0;

  switch (data[0]) {
    case CMD_SET_ODR: {
      static const uint16_t table[] = { 52, 104, 208, 416 };
      if (arg < 4) { cfg.odr_hz = table[arg]; applyConfig(); }
      break;
    }
    case CMD_SET_ACCEL_FS: {
      static const uint8_t table[] = { 2, 4, 8, 16 };
      if (arg < 4) { cfg.accel_fs_g = table[arg]; applyConfig(); }
      break;
    }
    case CMD_SET_GYRO_FS: {
      static const uint16_t table[] = { 245, 500, 1000, 2000 };
      if (arg < 4) { cfg.gyro_fs_dps = table[arg]; applyConfig(); }
      break;
    }
    case CMD_STREAM:
      g_stream_req = (arg != 0);
      if (g_stream_req) resetStream();
      publishStatus();
      break;
    case CMD_RESET_INDEX:
      resetStream();
      publishStatus();
      break;
    case CMD_IDENTIFY:
      g_identify_until = millis() + (arg ? arg * 1000UL : 3000UL);
      break;
  }
}

// ------------------------------------------------------------------- setup

static void bleSetup() {
  makeUuid(uuidSvc,  0x0001);
  makeUuid(uuidData, 0x0002);
  makeUuid(uuidCtrl, 0x0003);
  makeUuid(uuidStat, 0x0004);

  // Must precede begin(): 247-byte MTU, a long connection event so several
  // notifications fit in one interval, and a deeper HVN queue so a packet
  // built while the radio is busy is buffered instead of refused.
  Bluefruit.configPrphConn(247, 100, 3, 1);
  Bluefruit.begin(1, 0);
  Bluefruit.setTxPower(4);
  Bluefruit.setName("DogVitals");
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  Bluefruit.Periph.setConnInterval(6, 12);      // 7.5 - 15 ms

  bledis.setManufacturer("scholyx");
  bledis.setModel("XIAO nRF52840 Sense");
  bledis.begin();
  blebas.begin();

  svcImu.begin();

  chrData.setProperties(CHR_PROPS_NOTIFY);
  chrData.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrData.setMaxLen(sizeof(PktHeader) + MAX_SAMPLES_PER_PKT * BYTES_PER_SAMPLE);
  chrData.setCccdWriteCallback(onCccd);
  chrData.begin();

  chrCtrl.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  chrCtrl.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  chrCtrl.setMaxLen(4);
  chrCtrl.setWriteCallback(onCtrlWrite);
  chrCtrl.begin();

  chrStat.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  chrStat.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrStat.setFixedLen(sizeof(StatusPkt));
  chrStat.begin();

  // A 128-bit service UUID eats 18 of the 31 advertising bytes, so the name
  // goes in the scan response. Web Bluetooth filters on the service UUID and
  // reads the name from the scan response, so both halves are needed.
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(svcImu);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// Progress markers, printed repeatedly for the first seconds of life.
//
// A USB CDC console only exists once the host has opened the port, and by then
// setup() is long finished -- so a banner printed once at boot is a banner
// nobody ever sees. Worse, when bring-up went wrong the symptom was a board
// that enumerated fine and then said nothing at all, which is equally
// consistent with a hang in setup(), a failed radio init and a dead console.
// Recording how far setup() got and replaying it for ten seconds tells those
// three apart on the first connection instead of the third reflash.
static char g_boot_log[192];
static uint8_t g_boot_len = 0;

static void mark(const char* stage) {
  int n = snprintf(g_boot_log + g_boot_len, sizeof(g_boot_log) - g_boot_len,
                   "%s ", stage);
  if (n > 0) g_boot_len = (uint8_t)min((size_t)(g_boot_len + n), sizeof(g_boot_log) - 1);
}

void setup() {
  Serial.begin(115200);
  ledsInit();
  mark("serial");

  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, HIGH);
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);

  // setup() brings up the radio and nothing else.
  //
  // Both of the other things this board must do at boot -- read the battery
  // divider through the SAADC, and probe the IMU over I2C -- sit on top of
  // core drivers that spin on a hardware event with no timeout. This core's
  // TWIM read ends in `while(!_p_twim->EVENTS_STOPPED);`, which has no error
  // escape at all. A peripheral that does not answer therefore does not
  // produce a failed call, it produces a board that never reaches loop(): no
  // console, no advertising, no way in. That happened here on the first
  // bring-up and cost two reflashes to localise.
  //
  // So they run from loop() instead, after the radio is already advertising
  // and the console is already printing, where a hang is visible rather than
  // silent -- and where a sensor that was merely late to power up gets retried
  // instead of writing the whole session off.
  bleSetup();
  mark("ble");
  publishStatus();
}

// -------------------------------------------------------------------- loop

// Reports what the IMU's bus is physically doing, with the rail off and on.
//
// Worth having as a command rather than a one-off: the internal pull-ups on
// this board sit on the same switched rail as the sensor, so "both lines low"
// and "sensor missing" and "sensor held mid-transaction" all look identical
// from inside a failed read. Watching the levels move as the rail is toggled
// separates them -- and the rail only comes up under high drive, which is the
// fault this command was written to find.
static void i2cDiagnose() {
  const uint8_t sda = PIN_WIRE1_SDA, scl = PIN_WIRE1_SCL;

  Serial.println("# --- IMU bus diagnostic ---");
  Serial.printf("#   6D_PWR=D%u(P1.08)  SCL=D%u(P0.27)  SDA=D%u(P0.07)\n",
                IMU_PWR_PIN, scl, sda);

  const uint32_t pwr = g_ADigitalPinMap[IMU_PWR_PIN];
  nrf_gpio_cfg(pwr, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_clear(pwr);
  delay(150);
  i2cdiag::sdaHigh(sda); i2cdiag::sclHigh(scl); delay(2);
  Serial.printf("#   rail LOW  : SDA=%d SCL=%d\n", digitalRead(sda), digitalRead(scl));

  nrf_gpio_pin_set(pwr);
  static const int kSettle[] = { 5, 20, 60, 150 };
  for (unsigned i = 0; i < sizeof(kSettle) / sizeof(kSettle[0]); i++) {
    delay(kSettle[i]);
    i2cdiag::sdaHigh(sda); i2cdiag::sclHigh(scl); delay(1);
    Serial.printf("#   rail HIGH +%3dms: SDA=%d SCL=%d\n",
                  kSettle[i], digitalRead(sda), digitalRead(scl));
  }

  i2cdiag::recover(sda, scl);
  delay(2);
  Serial.printf("#   after recovery : SDA=%d SCL=%d\n",
                digitalRead(sda), digitalRead(scl));

  // Control group. Two lines reading low with an internal pull-up enabled is
  // either a bus that is genuinely shorted, or a GPIO read that is lying --
  // and those call for completely different next steps. Pins with nothing
  // attached (D0/D1) and the external I2C header (D4/D5, no pull-ups fitted)
  // must read high; if they do not, the fault is in the measurement.
  static const uint8_t kPins[]  = { 0, 1, 4, 5, 15, 16, 17, 18 };
  static const char* const kName[] = { "D0 P0.02 nc", "D1 P0.03 nc",
                                       "D4 P0.04 extSDA", "D5 P0.05 extSCL",
                                       "D15 P1.08 6D_PWR", "D16 P0.27 6D_SCL",
                                       "D17 P0.07 6D_SDA", "D18 P0.11 6D_INT1" };
  Serial.println("#   pin survey (pull-up read, then driven-high readback):");
  for (unsigned i = 0; i < sizeof(kPins) / sizeof(kPins[0]); i++) {
    uint8_t pin = kPins[i];
    pinMode(pin, INPUT_PULLUP);
    delayMicroseconds(200);
    int pu = digitalRead(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delayMicroseconds(200);
    pinMode(pin, INPUT);
    delayMicroseconds(50);
    int drv = digitalRead(pin);
    pinMode(pin, INPUT);
    Serial.printf("#     %-18s pullup=%d  drivenHigh=%d\n", kName[i], pu, drv);
  }

  Serial.print("#   bit-banged scan:");
  int found = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    if (i2cdiag::probe(sda, scl, a)) { Serial.printf(" 0x%02X", a); found++; }
  }
  Serial.println(found ? "" : " (nothing responded)");
  i2cdiag::release(sda, scl);
  Serial.println("# --- end ---");
}

static void mark_once_batt() {
  static bool done = false;
  if (!done) { done = true; mark("probe-batt"); }
}

// Brings the IMU up from loop(), retrying a few times before giving up. The
// marker is written before the probe, not after, so a hang inside it still
// leaves a record of where the firmware got to.
static void imuInitTask() {
  if (g_imu_ok || g_imu_tries >= 6 || millis() < g_imu_next_try) return;
  g_imu_tries++;
  mark("probe-imu");
  g_imu_ok = imu.begin(cfg);
  mark(g_imu_ok ? "imu-ok" : "imu-retry");
  g_imu_next_try = millis() + 5000;
  if (g_imu_ok) xledOff(LED_RED); else xledOn(LED_RED);
  publishStatus();
}

static bool sendPacket(uint8_t* buf, uint16_t len) {
  if (!Bluefruit.connected()) return false;
  return chrData.notify(buf, len);
}

// Emits raw int16 counts rather than scaled floats.
//
// Formatting eight %f values per sample could not keep up at 208 Hz: the loop
// spent long enough inside printf that the sensor FIFO overflowed, and a
// six-second capture logged 191 overruns and arrived at an apparent 164 Hz.
// Integer formatting plus a single write per line is several times cheaper,
// and the scale factors are constant for a run -- so they belong in the header
// once, not multiplied into every field.
static void emitCsv(const PktHeader* h, const int16_t* s, uint8_t n) {
  char line[72];
  for (uint8_t i = 0; i < n; i++) {
    const int16_t* r = s + i * WORDS_PER_SAMPLE;
    int len = snprintf(line, sizeof(line), "%lu,%lu,%d,%d,%d,%d,%d,%d\n",
                       (unsigned long)(h->idx + i), (unsigned long)h->t_ms,
                       r[3], r[4], r[5], r[0], r[1], r[2]);
    if (len > 0) Serial.write((const uint8_t*)line, (size_t)len);
  }
}

// Advances the sample index over samples the device produced but lost.
//
// The index is the host's time base, and it was previously incremented once
// per *transmitted* sample -- so a FIFO overrun did not create a gap in it, it
// silently compressed time instead. A run that dropped 191 FIFO-fulls still
// looked perfectly continuous, and any breathing or heart rate computed from
// it would have been wrong by however much was missing, with nothing in the
// data to show for it. Re-deriving the index from elapsed device time turns
// that back into a visible hole, which the host can interpolate across or
// refuse to analyse, but can no longer mistake for good data.
static void resyncIndex() {
  const uint32_t elapsed = millis() - g_stream_t0_ms;
  const uint32_t expected = (uint32_t)(((uint64_t)elapsed * cfg.odr_hz) / 1000ULL);
  if (expected > g_sample_idx) g_sample_idx = expected;
}

static bool bleSubscribed() { return g_notify_on && Bluefruit.connected(); }

static void streamTask() {
  if (!streaming()) return;

  // A packet the radio refused last time goes before anything new, so the
  // sample index the host sees stays monotonic. Held only while there is
  // still someone to deliver it to -- an earlier version stashed the packet
  // whenever notify() failed, including when nothing was connected at all,
  // and then spent every subsequent pass retrying a send that could not
  // succeed. The USB capture path stopped dead after exactly one packet.
  if (g_pending_len) {
    if (bleSubscribed() && !sendPacket(g_pending, g_pending_len)) return;
    g_pending_len = 0;
  }

  // An overrun in continuous mode means the oldest samples were discarded to
  // make room -- not that what remains is bad. Realign to a sample boundary
  // and carry on rather than flushing the whole buffer.
  if (imu.fifoOverrun()) {
    g_overruns++;
    resyncIndex();
    imu.fifoRealign();
  }

  // The FIFO depth is re-read on every pass through this loop, and never
  // carried forward.
  //
  // It used to be read once and then decremented locally as packets were
  // drained. That is wrong in a way that is almost invisible: fifoRealign()
  // discards a word or two to get back onto a sample boundary, which the
  // local count knew nothing about, so the next read asked for more words
  // than the FIFO held. Reading past the end does not fail -- it returns
  // whatever is there and advances the read pointer anyway -- so the pointer
  // slipped by one word and every subsequent sample came out rotated: X read
  // Y, Y read Z, Z read the next sample's gyro. The magnitude of gravity
  // stays near 1 g through a rotation like that, the packet indices stay
  // perfectly contiguous, and nothing in the link statistics moves, so it
  // survived every throughput test. It showed up only in a still-board noise
  // measurement, as 7.6% of samples with |a| in the wrong place.
  for (;;) {
    uint16_t words = imu.fifoUnread();
    if (words > g_depth_max) g_depth_max = words;
    if ((words / WORDS_PER_SAMPLE) < g_pkt_samples) return;

    // Get onto a sample boundary, then re-read the depth: realigning consumes
    // words, and that is exactly the accounting the old code got wrong.
    if (!imu.fifoRealign()) {
      g_misaligns++;
      imu.fifoReset();
      resyncIndex();
      return;
    }
    words = imu.fifoUnread();
    const uint8_t avail = (uint8_t)min((uint16_t)(words / WORDS_PER_SAMPLE),
                                       (uint16_t)g_pkt_samples);
    if (avail < g_pkt_samples) return;      // wait for a whole packet

    uint8_t  buf[sizeof(PktHeader) + MAX_SAMPLES_PER_PKT * BYTES_PER_SAMPLE];
    PktHeader* h = (PktHeader*)buf;
    int16_t* samples = (int16_t*)(buf + sizeof(PktHeader));

    const uint8_t got = imu.fifoReadSamples((uint8_t*)samples, avail);
    // A short read means an I2C transfer failed part-way through a sample, so
    // the pointer is now mid-sample. There is no way to know how far in, so
    // the only safe move is to dump the FIFO.
    if (got != avail) {
      g_misaligns++;
      imu.fifoReset();
      resyncIndex();
      return;
    }

    // Whole samples in, whole samples out: the pattern index must be back at
    // zero. If it is not, this read slipped, and the packet just assembled is
    // rotated. Checking after the fact is what turns a silent corruption into
    // a counter -- the cost is two bytes over I2C per packet.
    if (imu.fifoPatternPhase() != 0) {
      g_misaligns++;
      imu.fifoReset();
      resyncIndex();
      return;                                // drop this packet, it is suspect
    }

    h->ver  = PROTO_VERSION;
    h->n    = got;
    h->seq  = g_seq++;
    h->idx  = g_sample_idx;
    h->t_ms = millis();
    g_sample_idx += got;

    const uint16_t len = sizeof(PktHeader) + (uint16_t)got * BYTES_PER_SAMPLE;

    if (bleSubscribed() && !sendPacket(buf, len)) {
      g_tx_drops++;
      memcpy(g_pending, buf, len);
      g_pending_len = len;
      if (g_csv) emitCsv(h, samples, got);
      return;
    }
    if (g_csv) emitCsv(h, samples, got);
  }
}

// A serial console, because the board spends its life inside tape or a collar
// enclosure where RESET is not reachable. 'd' in particular: the flash path on
// this board is serial DFU, and asking the running firmware to reboot into the
// bootloader beats hunting for a button.
static void serialTask() {
  if (!Serial || !Serial.available()) return;
  int c = Serial.read();
  switch (c) {
    case 'c':
      g_csv = !g_csv;
      Serial.printf("# csv %s\n", g_csv ? "on" : "off");
      if (g_csv) {
        if (!g_notify_on) resetStream();
        Serial.printf("# scale accel_g_per_lsb=%.8f gyro_dps_per_lsb=%.6f odr=%u\n",
                      imu.accelScaleG(), imu.gyroScaleDps(), cfg.odr_hz);
        Serial.println("# idx,t_ms,ax,ay,az,gx,gy,gz   (raw int16 counts)");
      }
      break;
    case 'r':
      g_stream_req = !g_stream_req;
      if (g_stream_req) resetStream();
      Serial.printf("# stream %s\n", g_stream_req ? "on" : "off");
      publishStatus();
      break;
    case 's':
      Serial.printf("# imu=%s whoami=0x%02X odr=%u accel=+/-%ug gyro=+/-%udps "
                    "conn=%d notify=%d mtu=%u n/pkt=%u idx=%lu over=%u misalign=%u "
                    "depthmax=%u drops=%u batt=%umV\n",
                    g_imu_ok ? "ok" : "FAIL", imu.whoAmI(), cfg.odr_hz,
                    cfg.accel_fs_g, cfg.gyro_fs_dps, (int)Bluefruit.connected(),
                    (int)g_notify_on, g_mtu, g_pkt_samples, (unsigned long)g_sample_idx,
                    g_overruns, g_misaligns, g_depth_max, g_tx_drops, g_batt_mv);
      break;
    case 'i':
      i2cDiagnose();
      g_imu_tries = 0;                 // let the normal probe try again after
      g_imu_next_try = millis() + 200;
      break;
    case 'd':
      Serial.println("# rebooting into serial DFU");
      Serial.flush();
      delay(50);
      enterSerialDfu();
      break;
    case 'u':
      Serial.println("# rebooting into UF2 bootloader");
      Serial.flush();
      delay(50);
      enterUf2Dfu();
      break;
  }
}

void loop() {
  imuInitTask();
  streamTask();
  serialTask();
  ledTask();

  static uint32_t last_banner = 0;
  if (!g_csv && !streaming() && millis() < 30000 && millis() - last_banner >= 500) {
    last_banner = millis();
    Serial.printf("# dog-vitals boot: %s| whoami=0x%02X adv=%d  (s=status c=csv r=stream d=dfu)\n",
                  g_boot_log, imu.whoAmI(), (int)Bluefruit.Advertising.isRunning());
  }

  static uint32_t last_slow = 0;
  if (millis() - last_slow >= 4000) {
    last_slow = millis();
    mark_once_batt();
    updatePacketSize();
    g_batt_mv = readBatteryMv();
    blebas.write(battPercent(g_batt_mv));
    publishStatus();
  }

  // Yield so the SoftDevice and USB tasks run. At 208 Hz a full packet of 19
  // samples takes 91 ms to accumulate, so polling every millisecond is far
  // more often than needed and still leaves the FIFO 3 seconds of headroom.
  delay(1);
}
