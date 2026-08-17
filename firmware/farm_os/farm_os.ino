/*
 * Farm OS — STM32 Firmware (Stage 1: Motor Control)
 * Arduino UNO Q — STM32U585 MCU side
 *
 * Flash via Arduino IDE:
 *   Board: Arduino UNO Q (install via Board Manager → "Arduino UNO Q")
 *   Port:  select the UNO Q USB-C port
 *
 * RouterBridge exposes RPC functions callable from the Linux side.
 * D0/D1 are owned by RouterBridge — do NOT use them for anything else.
 */

#include <Arduino_RouterBridge.h>
#include <OneWire.h>            // Library Manager: "OneWire"
#include <DallasTemperature.h> // Library Manager: "DallasTemperature"
#include <Servo.h>             // seeder servos
#include <Wire.h>              // MPU-6050 gyro — Wire2 = i2c3 = A4(SDA)/A5(SCL)

// ── Soil sensor pins (Stage 2) ──────────────────────────────────────────────
#define MOIST_A0   A0   // capacitive moisture #1 (powered at 3.3V!)
#define MOIST_A1   A1   // capacitive moisture #2 (powered at 3.3V!)
#define EC_SENSE   A2   // DIY EC probe sense node (22kΩ pulldown)
#define EC_DRIVE   13   // EC AC drive (was D10 — moved off the seeder spool; D13 freed
                        // by the single-solenoid-driver decision). Soil probe + seeder
                        // are mutually-exclusive attachments.
#define TEMP_PIN    2   // DS18B20 1-Wire (D2) + 4.7kΩ pull-up to 3.3V

// ── Battery monitor ─────────────────────────────────────────────────────────
// REMOVED FROM A4 on 2026-08-17. A4/A5 are the ONLY header pins with a working I2C
// peripheral behind them on this core (i2c3 -> Wire2), and the gyro needs both — see the
// MPU-6050 block below for why the D20/D21 "SDA/SCL" pins cannot be used. The battery
// monitor lost the coin toss because it was already the least trusted reading on the
// robot: it has never been stable enough to compensate duty with (the console has run
// batt_comp=False throughout, and field runs pass `nobatt`), whereas the turn has no
// other way to know what it actually did.
//
// To put it back: set BATT_PRESENT 1, refit the 10k/2k divider + 100nF to A4, and move
// the gyro to the Qwiic connector (Wire1) — which needs a JST-SH cable.
#define BATT_PRESENT 0
#define BATT_PIN   A4       // 3S LiPo via a 10kΩ/2kΩ divider + 100nF at the node → A4
#define BATT_MULT  5.910f   // MEASURED on this unit: 12.53V pack / 2.12V node.
                            // Nominal (10k+2k)/2k = 6.0; the 1.5% delta is resistor
                            // tolerance. Re-measure if the divider is ever rebuilt.
#define ADC_MAX    16383.0f // 14-bit (analogReadResolution(14))
// UNO Q ADC reference is VREF+ = ~3.3V (VDD); inputs are 0–3.3V, NOT 5V-tolerant.
// Use a LOW-impedance divider (10k/2k, ~1.7kΩ source) — the earlier 100k/20k
// (~17kΩ) was too weak for this ADC and got loaded down (read ~half the true node).
// Fine-tune per unit: ADC_VREF_new = ADC_VREF_old × (meter_volts ÷ reported_volts).
// ONLY calibrate once the reading is STABLE. A wandering reading is a wiring fault,
// and calibrating it just bakes in whichever state it happened to be in. Seen
// 2026-08-11: readings drifted 94% -> 61% of the true node within one session,
// traced to the A4 lead being the one friction-fit link in an otherwise soldered
// divider (the header is female, so solder the wire to a male pin and strain-relieve
// it). Symptom to recognise: low AND unstable together, while a 10M-ohm meter at the
// node reads correctly because it doesn't load it.
#define ADC_VREF   3.475f   // MEASURED on this unit (2026-08-12): the meter reads 2.120V
                            // at the node while the ADC reports 2.0133V, so VREF+ is ~5%
                            // ABOVE the 3.3V nominal. 3.3 is the datasheet figure, not
                            // this board's. (2.66 was an old band-aid on a loaded reading.)

// Averaged battery ADC, with a few samples thrown away first: getMoisture/getEC
// read other analog pins, and the first conversion after a channel switch can
// still carry residue from the previous one. Cheap insurance on a shared ADC.
#if BATT_PRESENT
static float readBattRaw() {
  const int WARMUP = 4, N = 32;
  for (int i = 0; i < WARMUP; i++) (void)analogRead(BATT_PIN);
  long acc = 0;
  for (int i = 0; i < N; i++) acc += analogRead(BATT_PIN);
  return acc / (float)N;
}
#endif

OneWire           ec_oneWire(TEMP_PIN);
DallasTemperature tempSensor(&ec_oneWire);

// ── Seeder pins (per uno-q-wiring.md v2 pinout) ─────────────────────────────
#define SPOOL_PIN  10   // S3003 spool drive (1:1 gear / direct)
#define DRUM_PIN   11   // SG90 drum metering (servos need a DIGITAL pin on UNO Q)
#define PUNCH_PIN  A3   // solenoid MOSFET gate — ONE driver fires BOTH punches (on/off)
// OPTIONAL gate sense — DISABLED 2026-08-17, and it must stay that way while the gyro
// is on A4/A5. It was never wired (it read floating noise), and analogRead() RE-MUXES
// the pad before sampling, so a single gateTest() call would have taken A5 away from
// i2c3 and killed the bus mid-run. The gate node has no other free pin to move to.
#define GATE_SENSE 0
#define SENSE_PIN  A5      // = the gyro's SCL now. Do NOT analogRead it.

Servo spoolServo;
Servo drumServo;

// Drum angles — bench-tuned 2026-07: fill=pocket up under feed, drop=pocket over outlet.
const int DRUM_FILL = 0;
const int DRUM_DROP = 165;
// Plant sequence — SEED FIRST, then punch (corrected 2026-08-14).
// The drum releases the seed, then the tip drives it into the soil. The tip is NOT
// making a hole for the seed to fall through; the seed is already at the spot when
// the punch fires, so the punch is what puts it to depth.
//
//   t = 0                drum -> DROP, seed leaves the pocket
//   + DROP_MS            seed clear of the drum, at the tip
//   + PUNCH_DELAY_MS     punch ON   <- keep this SHORT
//   + PUNCH_HOLD_MS      punch OFF, spring retracts
//   + FILL_MS            drum back to fill, tip already up
//
// COIL-ON is now PUNCH_HOLD_MS alone = 500 ms (the JF-0530B is pulse-rated). That is
// down from 1300 ms in the original punch-then-drop order.
const int DROP_MS        = 500;  // drum travel to DROP + seed released
const int PUNCH_DELAY_MS = 100;  // drop -> punch. SHORT on purpose: the seed must be
                                 // driven in before it can roll or bounce off the spot.
const int PUNCH_HOLD_MS  = 500;  // coil-on — tip held at depth, then spring return
const int FILL_MS        = 400;  // drum back to fill — punch already retracted

// ── IBT-2 Pin Assignments (identical to Nano wiring — no rewiring needed) ──
#define L_RPWM  3   // IBT-2 #1 RPWM  — left  forward
#define L_LPWM  5   // IBT-2 #1 LPWM  — left  reverse
#define R_RPWM  6   // IBT-2 #2 RPWM  — right forward
#define R_LPWM  9   // IBT-2 #2 LPWM  — right reverse
#define L_R_EN  7   // IBT-2 #1 R_EN  — left  enable
#define L_L_EN  8   // IBT-2 #1 L_EN  — left  enable
#define R_R_EN  4   // IBT-2 #2 R_EN  — right enable
#define R_L_EN  12  // IBT-2 #2 L_EN  — right enable

static int s_left_pwm  = 0;
static int s_right_pwm = 0;

// ── Motor current sense (IBT-2 R_IS/L_IS) — OPTIONAL, off until wired ───────
// Tie R_IS+L_IS together on each driver (only one half-bridge conducts at a
// time, so one pin covers both directions), then IS_RESISTOR to GND, node → ADC.
//
// BTS7960: the IS pin sources IL/8500. Vis = IL/8500 × R.  SIZE THE RESISTOR:
// the UNO Q ADC is 0–3.3V and NOT 5V-tolerant, and IS jumps to ~4.5mA in a FAULT
// (overtemp/short). 3.3V / 4.5mA ≈ 733Ω, so keep R ≤ 680Ω and even a fault stays
// in range. (Many IBT-2 boards ship 1k or 10k here — 10k would push >3.3V at only
// ~2.8A and damage the input. Check yours.)  At 680Ω: 10A → 0.80V.
#define CURRENT_SENSE 0        // ← set to 1 once the IS pins are actually wired
#define IS_L_PIN      A5       // left  driver IS node (A5 is the free analog pin)
#define IS_R_PIN      A1       // right driver IS node (frees up: moisture #2)
#define IS_RESISTOR   680.0f   // ohms, IS node → GND
#define IS_RATIO      8500.0f  // BTS7960 k_ILIS: load amps per sense amp

// ── MPU-6050 gyro (yaw) — closing the loop on TURNS ────────────────────────
// WHY: the pivot is timed (tstartup/tdps), which assumes a constant deg/s. Skid breaks
// that assumption differently on every surface and every run — a 72 deg command has
// landed anywhere from a few degrees to the full amount. A gyro measures what the
// CHASSIS actually did, so the turn can be closed on the achieved angle instead.
//
// Deliberately raw register access, no MPU6050 library: sketch.yaml pins library
// versions to what this board has, and adding a dependency there has broken the build
// before. Six registers is not worth that risk.
//
// WIRING: module on A4 (=SDA) / A5 (=SCL), powered from +3V3, AD0 -> GND (addr 0x68).
//
// NOT the pins labelled SDA/SCL on the header. Those are D20/D21 = PB10/PB11, and this
// core's board overlay hands those pads to USART3:
//     &usart3 { status = "okay"; pinctrl-0 = <&usart3_tx_pb10 &usart3_rx_pb11>; };
//     &i2c2   { zephyr,deferred-init; };        <- no status, no pinctrl: DISABLED
// So `Wire` scans a peripheral that was never enabled and finds nothing, however
// correctly the module is wired. Confirmed the hard way 2026-08-17, and it matches
// arduino/ArduinoCore-zephyr#301. The pads still work as plain GPIO, which is why
// gateTest(20)/gateTest(21) pass and mislead.
//
// The Arduino objects map (overlay: `i2cs = <&i2c2>, <&i2c4>, <&i2c3>;`):
//     Wire  -> i2c2  D20/D21     DISABLED — do not use
//     Wire1 -> i2c4  Qwiic only  works, needs a JST-SH cable
//     Wire2 -> i2c3  A4/A5       works, and is solderable — so this is the one.
// Editing the overlay is not an option: it is compiled into the prebuilt Zephyr ELF we
// flash, so changing that text does nothing without rebuilding Zephyr from source.
// AD0 low -> 0x68, AD0 high or floating -> 0x69. Both are tried, because a floating
// AD0 is the single most common reason a correctly-wired module does not answer.
static uint8_t s_mpu_addr = 0x68;
#define MPU_ADDR       s_mpu_addr
#define MPU_SMPLRT_DIV 0x19
#define MPU_CONFIG     0x1A
#define MPU_GYRO_CFG   0x1B
#define MPU_GYRO_ZOUT  0x47
#define MPU_PWR_MGMT_1 0x6B
#define MPU_WHO_AM_I   0x75
// +/-500 deg/s full scale. The pivot runs near 45 deg/s, so this leaves headroom for
// vibration spikes: a clipped reading is ASYMMETRIC noise and integrates into a real
// error, whereas unclipped vibration averages to nothing.
// 0x08 = FS_SEL 1 (+/-500 dps) with bits[1:0] = 0.
// THOSE LOW BITS MATTER ON THIS BOARD: the vendor states it "will function as MPU6500",
// and on the 6500 GYRO_CONFIG[1:0] is FCHOICE_B (reserved on a real 6050) — non-zero
// BYPASSES the DLPF and runs the gyro at 8/32kHz, which would undo the anti-aliasing the
// filter is there for. Keep them zero.
#define MPU_GYRO_FS_500  0x08
// SENSITIVITY IS READ BACK, NEVER ASSUMED. This module answers WHO_AM_I = 0x74, so it is
// not a genuine MPU-6050 (0x68) — it is register-compatible but its FS_SEL write does not
// necessarily stick. The first closed-loop turn came out at ~45 deg for a 90 deg command:
// a clean factor of 2, i.e. the chip was running at +/-250 (131 LSB/dps) while this code
// divided by the +/-500 figure, so every rate read twice as fast as reality and the
// integrator "arrived" halfway. Whatever the chip says it is doing is what we scale by.
static const float MPU_LSB[4] = {131.0f, 65.5f, 32.8f, 16.4f};   // FS_SEL 0..3
static float s_lsb_per_dps = 65.5f;
static int   s_gyro_cfg    = -1;
// DLPF_CFG=3 -> 42Hz gyro bandwidth, filtered ON-CHIP i.e. BEFORE sampling. Without
// this, motor/gearbox vibration aliases down into what looks like genuine slow rotation.
#define MPU_DLPF_44HZ    0x03

// WHICH BUS. The header SDA/SCL and the Qwiic socket are different I2C peripherals, and
// which Arduino object maps to which is a property of the core, not something to assume:
// the first attempt scanned `Wire` and found an empty bus. So probe both and keep the one
// that answers. s_bus_name is reported, so the log says where the sensor actually is.
static TwoWire *s_bus = &Wire;
static const char *s_bus_name = "Wire";

static bool  s_imu_ok    = false;
static int   s_imu_who   = -1;
static float s_imu_bias  = 0.0f;    // deg/s at rest, sampled just before each pivot

static bool mpuWrite(uint8_t reg, uint8_t val) {
  s_bus->beginTransmission(MPU_ADDR);
  s_bus->write(reg);
  s_bus->write(val);
  return s_bus->endTransmission() == 0;
}

static int mpuRead8(uint8_t reg) {
  s_bus->beginTransmission(MPU_ADDR);
  s_bus->write(reg);
  if (s_bus->endTransmission(false) != 0) return -1;
  if (s_bus->requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) != 1) return -1;
  return s_bus->read();
}

// Raw yaw rate, deg/s, bias NOT removed.
static bool mpuGyroZ(float &dps) {
  s_bus->beginTransmission(MPU_ADDR);
  s_bus->write(MPU_GYRO_ZOUT);
  if (s_bus->endTransmission(false) != 0) return false;
  if (s_bus->requestFrom((uint8_t)MPU_ADDR, (uint8_t)2) != 2) return false;
  int16_t raw = (int16_t)((s_bus->read() << 8) | s_bus->read());
  dps = raw / s_lsb_per_dps;
  return true;
}

// Probe every legal 7-bit address. Answers the only question that matters when a new
// sensor is silent: is ANYTHING on this bus? Nothing at all points at wiring or power;
// something at an unexpected address points at AD0 or a different part.
static int i2cScanBus(TwoWire &w, uint8_t *found, int max_found) {
  w.begin();
  int n = 0;
  for (uint8_t a = 0x08; a <= 0x77 && n < max_found; a++) {
    w.beginTransmission(a);
    if (w.endTransmission() == 0) found[n++] = a;
    delay(2);
  }
  return n;
}

static int i2cScan(uint8_t *found, int max_found) {
  return i2cScanBus(*s_bus, found, max_found);
}

static String addrList(uint8_t *a, int n) {
  String s = "[";
  for (int i = 0; i < n; i++) { if (i) s += ","; s += "\"0x" + String(a[i], HEX) + "\""; }
  return s + "]";
}

String rpc_i2cScan() {
  uint8_t a[16];
  int n0 = i2cScanBus(Wire, a, 16);
  String s = "{\"Wire_d20d21_disabled\":{\"count\":" + String(n0) + ",\"addrs\":" + addrList(a, n0) + "}";
  int n1 = i2cScanBus(Wire1, a, 16);
  s += ",\"Wire1_qwiic\":{\"count\":" + String(n1) + ",\"addrs\":" + addrList(a, n1) + "}";
  int n2 = i2cScanBus(Wire2, a, 16);
  s += ",\"Wire2_a4a5\":{\"count\":" + String(n2) + ",\"addrs\":" + addrList(a, n2) + "}";
  s += ",\"note\":\"Wire=i2c2 D20/D21 is DISABLED in this core's overlay (usart3 owns the pads)\"}";
  return s;
}

// i2cLines(): are SDA and SCL sitting HIGH when idle?
// The single most useful measurement on a dead I2C bus, and it needs no meter. Both lines
// are open-drain and pulled up by the module, so IDLE MUST READ 1/1. A 0 means something
// is holding that line down — on this robot the prime suspect is leftover hardware on the
// pad (the battery divider's 2k leg to GND and its 100nF would both do it), not the MPU.
// Reads as plain GPIO inputs, then hands the pads back to i2c3 via begin(), which
// re-applies the pinctrl state.
// Read a pad floating, then with the MCU's own pull-up. The PAIR is what identifies the
// fault, which a single reading cannot:
//   float=1, pullup=1  -> a real external pull-up is present: the module IS connected
//   float=0, pullup=1  -> NO external pull-up: that wire is not reaching the module's pin
//   float=0, pullup=0  -> the pad is held down: short to GND, or something else on the pin
static void lineTest(int pin, int &floating, int &pulled) {
  pinMode(pin, INPUT);
  delay(20);                       // 4.7k into a few hundred pF settles in microseconds;
  floating = digitalRead(pin);     // 20ms is generous so a slow line cannot fool us
  pinMode(pin, INPUT_PULLUP);
  delay(20);
  pulled = digitalRead(pin);
  pinMode(pin, INPUT);             // leave it high-Z, not driving the bus
}

String rpc_i2cLines() {
  int sda_f, sda_p, scl_f, scl_p;
  lineTest(A4, sda_f, sda_p);
  lineTest(A5, scl_f, scl_p);
  Wire2.begin();                   // hand the pads back to i2c3 (re-applies pinctrl)

  const char *v_sda = sda_f ? "external pull-up present — connected"
                    : sda_p ? "NO external pull-up — A4 is not reaching the module's SDA pin"
                            : "HELD LOW — short to GND or something else on A4";
  const char *v_scl = scl_f ? "external pull-up present — connected"
                    : scl_p ? "NO external pull-up — A5 is not reaching the module's SCL pin"
                            : "HELD LOW — short to GND or something else on A5";
  return "{\"sda_a4\":{\"float\":" + String(sda_f) + ",\"pullup\":" + String(sda_p) +
         ",\"verdict\":\"" + String(v_sda) + "\"}" +
         ",\"scl_a5\":{\"float\":" + String(scl_f) + ",\"pullup\":" + String(scl_p) +
         ",\"verdict\":\"" + String(v_scl) + "\"}}";
}

static bool imuBegin() {
  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin();
  Wire2.begin();
  s_imu_who = -1;
  // Wire2 first: it is where the sensor is meant to be. The others are probed anyway so
  // a moved sensor still works and the log says which bus answered.
  TwoWire *buses[3] = {&Wire2, &Wire1, &Wire};
  const char *names[3] = {"Wire2 (A4/A5)", "Wire1 (Qwiic)", "Wire (D20/D21, disabled)"};
  for (int b = 0; b < 3 && s_imu_who < 0; b++) {
    for (int i = 0; i < 2 && s_imu_who < 0; i++) {      // 0x68 then 0x69
      s_bus = buses[b]; s_bus_name = names[b];
      s_mpu_addr = (i == 0) ? 0x68 : 0x69;
      s_imu_who = mpuRead8(MPU_WHO_AM_I);
    }
  }
  // 0x68 is the MPU-6050; MPU-6500/9250 clones answer 0x70/0x71/0x73. All of them
  // have the same gyro registers, so accept any sane reply rather than one value.
  if (s_imu_who < 0 || s_imu_who == 0xFF) { s_imu_ok = false; return false; }
  s_imu_ok  = mpuWrite(MPU_PWR_MGMT_1, 0x00);           // wake from sleep
  delay(50);
  s_imu_ok &= mpuWrite(MPU_CONFIG,   MPU_DLPF_44HZ);
  s_imu_ok &= mpuWrite(MPU_GYRO_CFG, MPU_GYRO_FS_500);
  s_imu_ok &= mpuWrite(MPU_SMPLRT_DIV, 0x04);           // 1kHz/(1+4) = 200Hz
  delay(20);
  // Trust the DEVICE, not the write: read FS_SEL back and scale by what it actually is.
  s_gyro_cfg = mpuRead8(MPU_GYRO_CFG);
  if (s_gyro_cfg >= 0) s_lsb_per_dps = MPU_LSB[(s_gyro_cfg >> 3) & 0x03];
  return s_imu_ok;
}

// Zero-rate offset, measured NOW. Bias drifts with temperature, and it cannot be
// measured while the motors are running — so every pivot re-samples it standing still
// in the moment before it moves.
static float imuSampleBias(int n = 200) {
  float sum = 0; int got = 0, d;
  for (int i = 0; i < n; i++) {
    float dps;
    if (mpuGyroZ(dps)) { sum += dps; got++; }
    delay(2);
  }
  (void)d;
  return got ? sum / got : 0.0f;
}

// ── Diagnostics (getDiag) ──────────────────────────────────────────────────
// Handlers only touch plain globals — no bridge traffic inside an RPC, which
// would add a round-trip to the very drive times the Linux side is measuring.
static uint32_t s_cmd_count  = 0;   // setMotors calls since boot
static uint32_t s_stop_count = 0;
static uint32_t s_last_cmd_ms = 0;
static int s_req_left = 0, s_req_right = 0;             // as RECEIVED over the bridge
static int s_pin_l_r = 0, s_pin_l_l = 0;                // as WRITTEN to the pins
static int s_pin_r_r = 0, s_pin_r_l = 0;
// Latched copy of the last non-stop command. stop() zeroes the live values, so
// without this a caller that polls getDiag *after* a move (the only safe moment —
// polling mid-move would corrupt the drive timing it is measuring) sees all zeros
// and can't verify what the move was actually commanded with.
static int s_mv_req_l = 0, s_mv_req_r = 0, s_mv_app_l = 0, s_mv_app_r = 0;
static int s_mv_pin_lr = 0, s_mv_pin_ll = 0, s_mv_pin_rr = 0, s_mv_pin_rl = 0;
// per-move current accumulators, reset on each setMotors, sampled in loop()
static float s_amp_l_sum = 0, s_amp_r_sum = 0, s_amp_l_max = 0, s_amp_r_max = 0;
static uint32_t s_amp_n = 0;

// Monitor logging — never block on it: if nothing is attached the robot must
// still run. operator bool() costs a bridge call when disconnected, so the
// result is cached and only re-probed occasionally.
static bool     s_mon_ok = false;
static uint32_t s_mon_check_ms = 0;
static bool monitorReady() {
  uint32_t now = millis();
  if (!s_mon_ok && (now - s_mon_check_ms) > 5000) {
    s_mon_check_ms = now;
    s_mon_ok = (bool)Monitor;
  }
  return s_mon_ok;
}

// ── RPC: ping ──────────────────────────────────────────────────────────────
// Linux side calls bridge.call("ping") → expects "pong"
String rpc_ping() {
  return "pong";
}

// ── RPC: setMotors(left, right) ────────────────────────────────────────────
// left, right: -255 to 255
//   positive = forward, negative = reverse, 0 = coast
// Drive one side and record what was actually written, so getDiag reports the
// pin duty rather than a second copy that could drift from it.
static void writeSide(int rpwm_pin, int lpwm_pin, int pwm, int &rec_r, int &rec_l) {
  rec_r = (pwm >= 0) ?  pwm : 0;
  rec_l = (pwm >= 0) ?  0   : -pwm;
  analogWrite(rpwm_pin, rec_r);
  analogWrite(lpwm_pin, rec_l);
}

int rpc_setMotors(int left, int right) {
  s_req_left  = left;                 // keep the raw request — a clamped value
  s_req_right = right;                // still proves what the bridge delivered
  s_left_pwm  = constrain(left,  -255, 255);
  s_right_pwm = constrain(right, -255, 255);

  writeSide(L_RPWM, L_LPWM, s_left_pwm,  s_pin_l_r, s_pin_l_l);
  writeSide(R_RPWM, R_LPWM, s_right_pwm, s_pin_r_r, s_pin_r_l);

  if (s_left_pwm != 0 || s_right_pwm != 0) {   // latch real moves, not the stop
    s_mv_req_l = s_req_left;  s_mv_req_r = s_req_right;
    s_mv_app_l = s_left_pwm;  s_mv_app_r = s_right_pwm;
    s_mv_pin_lr = s_pin_l_r;  s_mv_pin_ll = s_pin_l_l;
    s_mv_pin_rr = s_pin_r_r;  s_mv_pin_rl = s_pin_r_l;
  }
  s_cmd_count++;
  s_last_cmd_ms = millis();
  s_amp_l_sum = s_amp_r_sum = s_amp_l_max = s_amp_r_max = 0;   // new move, new window
  s_amp_n = 0;
  return 1;
}

// ── RPC: stop ──────────────────────────────────────────────────────────────
// NOTE: both PWM inputs low with the enables HIGH shorts the motor terminals —
// this is dynamic BRAKING, not coasting.
int rpc_stop() {
  s_left_pwm  = 0;
  s_right_pwm = 0;
  s_req_left  = 0;   // a stop IS a request for zero — keeps "req" meaning the last
  s_req_right = 0;   // requested state, so the Diag view can compare it directly
  writeSide(L_RPWM, L_LPWM, 0, s_pin_l_r, s_pin_l_l);
  writeSide(R_RPWM, R_LPWM, 0, s_pin_r_r, s_pin_r_l);
  s_stop_count++;
  s_last_cmd_ms = millis();
  return 1;
}

// ── RPC: getMotorState ─────────────────────────────────────────────────────
// Returns JSON string: {"left":150,"right":150}
String rpc_getMotorState() {
  return "{\"left\":" + String(s_left_pwm) + ",\"right\":" + String(s_right_pwm) + "}";
}

// ── RPC: getDiag ───────────────────────────────────────────────────────────
// One snapshot of what the MCU actually saw. Call it BETWEEN moves (it builds a
// String and reads the ADC — cheap, but not something to do inside a timed hop).
// Fields:
//   up_ms                MCU uptime — a reset mid-run shows up as this going backwards
//   cmd.n / stops        call counters: proves commands are arriving at all
//   cmd.ms_ago           age of the last motor command
//   cmd.req_l/req_r      values as RECEIVED over the bridge (pre-clamp)
//   pins.*               duty as WRITTEN to each IBT-2 input (post-clamp/direction)
//   batt.raw             averaged ADC counts, next to the derived volts, so a
//                        divider/wiring fault is distinguishable from bad math
//   amps.*               per-move average/peak motor current, if CURRENT_SENSE
String rpc_getDiag() {
  // NO analogRead(A4) HERE while the gyro owns it. analogRead() re-muxes the pad to
  // analog BEFORE sampling (see troubleshooting.md, [Seeder] gateTest), so this one line
  // silently stole SDA from i2c3 — and because the console's battery logger polls
  // getDiag every 2 SECONDS, the bus died again immediately after every Wire2.begin().
  // Symptom: a correctly-wired MPU-6050 that scans clean exactly zero times.
#if BATT_PRESENT
  float raw   = readBattRaw();
  float volts = raw / ADC_MAX * ADC_VREF * BATT_MULT;
#else
  float raw = 0, volts = 0;
#endif

  String s = "{\"up_ms\":" + String(millis());
  s += ",\"cmd\":{\"n\":" + String(s_cmd_count) +
       ",\"ms_ago\":" + String(millis() - s_last_cmd_ms) +
       ",\"req_l\":" + String(s_req_left) + ",\"req_r\":" + String(s_req_right) +
       ",\"app_l\":" + String(s_left_pwm) + ",\"app_r\":" + String(s_right_pwm) + "}";
  s += ",\"pins\":{\"l_rpwm\":" + String(s_pin_l_r) + ",\"l_lpwm\":" + String(s_pin_l_l) +
       ",\"r_rpwm\":" + String(s_pin_r_r) + ",\"r_lpwm\":" + String(s_pin_r_l) + "}";
  // last real move, latched — still valid after the stop that ended it
  s += ",\"move\":{\"req_l\":" + String(s_mv_req_l) + ",\"req_r\":" + String(s_mv_req_r) +
       ",\"app_l\":" + String(s_mv_app_l) + ",\"app_r\":" + String(s_mv_app_r) +
       ",\"l_rpwm\":" + String(s_mv_pin_lr) + ",\"l_lpwm\":" + String(s_mv_pin_ll) +
       ",\"r_rpwm\":" + String(s_mv_pin_rr) + ",\"r_lpwm\":" + String(s_mv_pin_rl) + "}";
  s += ",\"stops\":" + String(s_stop_count);
  s += ",\"batt\":{\"raw\":" + String(raw, 1) + ",\"volts\":" + String(volts, 2) + "}";
#if CURRENT_SENSE
  float ln = s_amp_n ? s_amp_l_sum / s_amp_n : 0.0f;
  float rn = s_amp_n ? s_amp_r_sum / s_amp_n : 0.0f;
  s += ",\"amps\":{\"l_avg\":" + String(ln, 2) + ",\"l_max\":" + String(s_amp_l_max, 2) +
       ",\"r_avg\":" + String(rn, 2) + ",\"r_max\":" + String(s_amp_r_max, 2) +
       ",\"n\":" + String(s_amp_n) + "}";
#endif
  s += "}";
  return s;
}

// ── RPC: getMoisture ─────────────────────────────────────────────────────────
// Returns JSON: {"a0":8200,"a1":7900}  (14-bit ADC, 0–16383; wetter = lower)
String rpc_getMoisture() {
  int a0 = analogRead(MOIST_A0);
  int a1 = analogRead(MOIST_A1);
  return "{\"a0\":" + String(a0) + ",\"a1\":" + String(a1) + "}";
}

// ── RPC: getTemperature ──────────────────────────────────────────────────────
// Returns Celsius as a string, 2 dp. "-127.00" = no DS18B20 found (check the
// 4.7kΩ pull-up — 1-Wire won't read without it).
String rpc_getTemperature() {
  tempSensor.requestTemperatures();
  return String(tempSensor.getTempCByIndex(0), 2);
}

// ── RPC: getEC ───────────────────────────────────────────────────────────────
// DIY EC: pulse the drive pin, read the divider node. v_high − v_low cancels
// ADC offset/ambient. Bigger = more conductive soil. Relative value (calibrate).
int rpc_getEC() {
  digitalWrite(EC_DRIVE, HIGH);
  delayMicroseconds(500);
  int v_high = analogRead(EC_SENSE);
  digitalWrite(EC_DRIVE, LOW);     // equal HIGH/LOW time keeps net DC low
  delayMicroseconds(500);
  int v_low = analogRead(EC_SENSE);
  return v_high - v_low;
}

// ── RPC: getBattery ──────────────────────────────────────────────────────────
// 3S LiPo on A4 via a 100kΩ/20kΩ divider. Averages the ADC (beats motor/PWM
// noise, complements the 100nF node cap), scales node → pack volts (×6.0), then
// maps to % via a per-cell LiPo discharge curve. Returns {"volts":11.82,"pct":58}.
static float battCellToPct(float v) {          // per-cell voltage → %, interpolated
  static const float T[][2] = {
    {3.30, 0}, {3.40, 3}, {3.55, 8}, {3.65, 15}, {3.70, 25}, {3.75, 35},
    {3.80, 45}, {3.85, 55}, {3.90, 65}, {4.00, 80}, {4.10, 90}, {4.20, 100}
  };
  const int N = sizeof(T) / sizeof(T[0]);
  if (v <= T[0][0])   return 0;
  if (v >= T[N-1][0]) return 100;
  for (int i = 1; i < N; i++) {
    if (v < T[i][0]) {
      float f = (v - T[i-1][0]) / (T[i][0] - T[i-1][0]);
      return T[i-1][1] + f * (T[i][1] - T[i-1][1]);
    }
  }
  return 100;
}

String rpc_getBattery() {
#if !BATT_PRESENT
  // A4 is the gyro's SDA now. A floating ADC pin reads plausible-looking noise, and a
  // plausible-looking wrong voltage is worse than none — so say so explicitly instead.
  return "{\"volts\":0,\"pct\":0,\"present\":false,\"why\":\"divider removed: A4/A5 are the gyro's I2C3\"}";
#else
  float vnode = readBattRaw() / ADC_MAX * ADC_VREF;
  float volts = vnode * BATT_MULT;
  int   pct   = (int)(battCellToPct(volts / 3.0f) + 0.5f);
  return "{\"volts\":" + String(volts, 2) + ",\"pct\":" + String(pct) +
         ",\"present\":true}";
#endif
}

// ── Gyro RPCs ────────────────────────────────────────────────────────────────
// getGyro(): is it alive, and what does it read RIGHT NOW. Bias is sampled on every
// call so a bench check shows the residual after removing it — the number that decides
// whether integration over a 2s turn is trustworthy.
String rpc_getGyro() {
  if (!s_imu_ok && !imuBegin()) {
    uint8_t found[16];
    int n = i2cScan(found, 16);
    String s = "{\"ok\":false,\"whoami\":" + String(s_imu_who) +
               ",\"i2c_devices\":" + String(n) + ",\"addrs\":[";
    for (int i = 0; i < n; i++) { if (i) s += ","; s += "\"0x" + String(found[i], HEX) + "\""; }
    s += "],\"err\":\"no MPU at 0x68 or 0x69 on I2C2\"}";
    return s;
  }
  float bias = imuSampleBias(100);
  float dps = 0, peak = 0, sum = 0;
  int n = 0;
  for (int i = 0; i < 100; i++) {                 // 100 samples of the residual
    if (mpuGyroZ(dps)) {
      float r = dps - bias;
      sum += r;
      if (fabsf(r) > fabsf(peak)) peak = r;
      n++;
    }
    delay(2);
  }
  s_imu_bias = bias;
  return "{\"ok\":true,\"whoami\":" + String(s_imu_who) +
         ",\"bus\":\"" + String(s_bus_name) + "\",\"addr\":\"0x" + String(s_mpu_addr, HEX) + "\"" +
         ",\"gyro_cfg\":\"0x" + String(s_gyro_cfg, HEX) + "\",\"fs_sel\":" +
         String(s_gyro_cfg >= 0 ? ((s_gyro_cfg >> 3) & 0x03) : -1) +
         ",\"lsb_per_dps\":" + String(s_lsb_per_dps, 1) +
         ",\"dps_full_scale\":" + String((int)(32768.0f / s_lsb_per_dps + 0.5f)) +
         ",\"bias_dps\":" + String(bias, 3) +
         ",\"mean_dps\":" + String(n ? sum / n : 0.0f, 3) +
         ",\"peak_dps\":" + String(peak, 3) +
         ",\"samples\":" + String(n) + "}";
}

// gyroIntegrate(ms): integrate yaw for a window, motors untouched.
// The scale-factor test. Turn the robot BY HAND through a known angle — a full 360 is
// far easier to call accurately than 90 — and compare. It isolates sensitivity from
// everything else that muddies a driven turn: no skid, no coast, no PWM, no early
// release. reported/actual is the correction, directly.
String rpc_gyroIntegrate(int ms) {
  if (!s_imu_ok && !imuBegin()) return "{\"ok\":false,\"err\":\"no gyro\"}";
  ms = constrain(ms, 500, 30000);
  float bias = imuSampleBias(150);          // hold still until this returns
  float signed_a = 0, abs_a = 0, peak = 0;
  uint32_t t0 = millis(), last = micros();
  int n = 0;
  while (millis() - t0 < (uint32_t)ms) {
    float dps;
    uint32_t now = micros();
    float dt = (now - last) / 1000000.0f;
    last = now;
    if (mpuGyroZ(dps)) {
      float r = dps - bias;
      signed_a += r * dt;
      abs_a    += fabsf(r) * dt;
      if (fabsf(r) > fabsf(peak)) peak = r;
      n++;
    }
  }
  return "{\"ok\":true,\"signed_deg\":" + String(signed_a, 1) +
         ",\"abs_deg\":" + String(abs_a, 1) +
         ",\"peak_dps\":" + String(peak, 1) +
         ",\"lsb_per_dps\":" + String(s_lsb_per_dps, 1) +
         ",\"bias_dps\":" + String(bias, 3) +
         ",\"samples\":" + String(n) + ",\"ms\":" + String(ms) + "}";
}

// setGyroScale(lsb_milli): override LSB-per-deg/s at RUNTIME (value x1000).
// Deliberately not persistent — it exists so calibration costs one RPC instead of a
// five-minute compile-and-flash cycle. Once a number is confirmed, bake it into
// MPU_LSB / imuBegin so it survives a reboot.
String rpc_setGyroScale(int lsb_milli) {
  if (lsb_milli >= 1000 && lsb_milli <= 500000) s_lsb_per_dps = lsb_milli / 1000.0f;
  return "{\"lsb_per_dps\":" + String(s_lsb_per_dps, 3) +
         ",\"dps_full_scale\":" + String((int)(32768.0f / s_lsb_per_dps + 0.5f)) + "}";
}

// pivotDeg(deg, pwm, turn_right): spin until the GYRO says we got there.
// Returns the angle actually achieved plus the time it took, so the caller can log
// commanded-vs-achieved — i.e. measure the skid instead of guessing at it.
//
// STOPS EARLY, deliberately: braking is not instant, so it releases the motors at
// COAST_FRAC of the target and lets the remainder carry it in. The coast is then
// measured for another moment and included in the returned angle.
//
// INTEGRATES SIGNED RATE, NOT |RATE|. The first version accumulated fabsf(dps - bias),
// which RECTIFIES vibration: judder swinging the rate back and forth adds a positive
// contribution in BOTH directions, so a shaky driven pivot banks angle the chassis never
// turned. Even a smooth hand rotation showed the gap (signed 354.6 vs abs 364.9 on a true
// 360); a motor pivot on concrete is far worse. `abs_deg` is still reported next to
// `achieved` precisely so that gap stays visible — if the two diverge, the robot is
// juddering and no amount of gyro accuracy will save the turn.
// COAST IS MEASURED, NOT GUESSED. It was a fixed 0.80 x target, which is the wrong shape:
// coast is a roughly constant number of DEGREES (set by speed and inertia), not a
// proportion of however far you asked to turn. At 90 deg that fraction released at 72 and
// the robot arrived at 80.1 — an err of -9.9 that is simply the ~8 deg of coast the
// fraction failed to account for, and it would have been -3.5 at 30 deg and -22 at 200.
//
// So: release at (target - coast_estimate), then keep integrating through the stop to see
// what the coast ACTUALLY was, and fold that into the estimate for next time. The number
// then tracks the surface — which is the whole reason for fitting a gyro, since concrete
// measured 64.6 deg/s against the 45.2 calibrated on soil.
// MEASURED, on both surfaces (2026-08-17, four consecutive 90 deg turns each):
//     soil      coast ~3.3 deg, turn rate 45-61 deg/s, worst error +/-1.7 deg
//     concrete  coast ~8.1 deg, turn rate 68-70 deg/s, worst error +/-0.3 deg
// Init to the SOIL figure: the field is where the first row change of a run actually
// happens, and starting from the concrete value cost -4.4 deg on the first soil turn while
// the estimate walked down. On concrete it now starts low and learns up instead, which
// only affects bench testing. Note this resets on every reboot and reflash, so the very
// first turn after one always carries this number.
#define PIVOT_COAST_INIT 3.3f
#define PIVOT_COAST_GAIN 0.5f    // EMA weight on each new measurement
#define PIVOT_COAST_MAX  45.0f   // sanity clamp, and never release before half the target
#define PIVOT_TIMEOUT_MS 4000
#define PIVOT_SETTLE_MS  400
// ONE estimate, shared by every caller — so only ever call pivotDeg at the standard
// turn_pwm. Coast scales with duty, so interleaving a 90 deg turn at pwm 120 with small
// nudges at a lower pwm would drag this value between two different truths and ruin both.
// Small vision-guided corrections are done with open-loop pulses on the Linux side instead.
static float s_coast_deg = PIVOT_COAST_INIT;

String rpc_pivotDeg(int deg, int pwm, int turn_right) {
  if (!s_imu_ok && !imuBegin()) {
    return "{\"ok\":false,\"err\":\"no gyro — caller must fall back to a timed pivot\"}";
  }
  float target = fabsf((float)deg);
  int p = constrain(abs(pwm), 0, 255);
  // SETTLE FIRST. Bias must be measured on a chassis that is genuinely still. In the drip
  // flow this is called moments after a creep-and-stop, and on soft ground the robot keeps
  // rocking for a beat afterwards — sampling into that rocking bakes a false zero into the
  // whole turn. Cheap insurance: it costs 250ms out of a two-second manoeuvre.
  rpc_stop();
  delay(250);
  s_imu_bias = imuSampleBias(150);                // standing still, right now

  float signed_a = 0, abs_a = 0, peak = 0, at_release = 0;
  // Release point: target minus what we expect to coast, floored at half the target so a
  // wild estimate can never make the robot barely move.
  float coast_used = constrain(s_coast_deg, 0.0f, PIVOT_COAST_MAX);
  float release_at = max(target - coast_used, target * 0.5f);
  uint32_t t0 = millis(), last = micros();
  bool released = false;
  rpc_setMotors(turn_right ? p : -p, turn_right ? -p : p);
  while (millis() - t0 < PIVOT_TIMEOUT_MS) {
    float dps;
    uint32_t now = micros();
    float dt = (now - last) / 1000000.0f;
    last = now;
    if (mpuGyroZ(dps)) {
      float r = dps - s_imu_bias;
      signed_a += r * dt;                 // SIGNED: judder cancels, as it should
      abs_a    += fabsf(r) * dt;          // kept only to expose judder in the log
      if (fabsf(r) > fabsf(peak)) peak = r;
    }
    if (!released && fabsf(signed_a) >= release_at) {
      rpc_stop();
      at_release = fabsf(signed_a);
      released = true;
      break;
    }
  }
  if (!released) rpc_stop();

  // keep integrating through the coast so the reported angle is the real one
  uint32_t t1 = millis();
  while (millis() - t1 < PIVOT_SETTLE_MS) {
    float dps;
    uint32_t now = micros();
    float dt = (now - last) / 1000000.0f;
    last = now;
    if (mpuGyroZ(dps)) {
      float r = dps - s_imu_bias;
      signed_a += r * dt;
      abs_a    += fabsf(r) * dt;
      if (fabsf(r) > fabsf(peak)) peak = r;
    }
  }

  float angle = fabsf(signed_a);          // direction is commanded, magnitude is measured
  // What the coast actually was this time -> carry it forward.
  float coast_seen = released ? (angle - at_release) : 0.0f;
  if (released && coast_seen >= 0.0f && coast_seen <= PIVOT_COAST_MAX) {
    s_coast_deg += PIVOT_COAST_GAIN * (coast_seen - s_coast_deg);
  }
  uint32_t took = millis() - t0;
  return "{\"ok\":true,\"lsb_per_dps\":" + String(s_lsb_per_dps, 1) +
         ",\"target\":" + String(target, 1) +
         ",\"achieved\":" + String(angle, 1) +
         ",\"err_deg\":" + String(angle - target, 1) +
         ",\"abs_deg\":" + String(abs_a, 1) +
         ",\"judder_deg\":" + String(abs_a - angle, 1) +
         ",\"coast_used\":" + String(coast_used, 1) +
         ",\"coast_seen\":" + String(coast_seen, 1) +
         ",\"coast_next\":" + String(s_coast_deg, 1) +
         ",\"peak_dps\":" + String(peak, 1) +
         ",\"avg_dps\":" + String(took ? angle / (took / 1000.0f) : 0.0f, 1) +
         ",\"ms\":" + String(took) +
         ",\"bias_dps\":" + String(s_imu_bias, 3) +
         ",\"timeout\":" + String(released ? "false" : "true") + "}";
}

// ── Seeder RPCs ──────────────────────────────────────────────────────────────
// S3003 travel calibration — measured on the arm 2026-08-14 with `field_test.py spool`.
//
// VERIFIED OPERATING POINTS — and these are the only two the drip cross ever uses:
//     physical   0 deg  <-  servo cmd  0   (rest position, the 0 reference)
//     physical  90 deg  <-  servo cmd 64   (bench-confirmed: `spool go 75` under the
//                                           previous 0.8571 constant emitted 64)
// SPOOL_A is set so indexSpool(90) emits exactly 64.
//
// ⚠️ DO NOT trust this model far from those points. Two readings were taken:
//       cmd 90 -> phys 105   (uncalibrated firmware)
//       cmd 64 -> phys  90   (this calibration)
//   A line through them has slope 0.58 phys/cmd and intercept +53 deg, which
//   contradicts cmd 0 resting at phys 0. So the response is either non-linear across
//   the range or one reading was eyeballed rather than measured. It does not affect
//   the 4-seed cross, which only ever asks for 0 and 90 — both pinned above — but
//   re-run `spool sweep` + `spool solve` before relying on 45 or anything else.
//   (firmware/s3003_cycle.ino's "write(90) reached only ~60 deg" note is closer to
//   that 0.58 slope than to a proportional fit, so it may have been right all along.)
//   commanded = SPOOL_A * physical + SPOOL_B
const float SPOOL_A = 0.7111f;
const float SPOOL_B = 0.0f;

// indexSpool(deg): rotate the spool to an absolute PHYSICAL angle. Callers (the drip
// cross, the console's _drip["angles"]) ask for the arm angle they actually want and
// the calibration above converts it to the servo command — so the correction lives in
// exactly one place. Returns the raw servo angle written, so the mapping is visible.
int rpc_indexSpool(int deg) {
  // +0.5f rather than lroundf(): this Zephyr build's libm has no lroundf (link error),
  // and the argument here is never negative, so plain rounding is exact.
  int cmd = (int)(SPOOL_A * constrain(deg, 0, 210) + SPOOL_B + 0.5f);
  spoolServo.write(constrain(cmd, 0, 180));
  return cmd;
}

// dropSeed(): meter one drop — drum fill(0) → drop(165) → back to fill.
int rpc_dropSeed() {
  drumServo.write(DRUM_DROP); delay(DROP_MS);
  drumServo.write(DRUM_FILL); delay(FILL_MS);
  return 1;
}

// punch()/retract(): drive the tip into the soil / release (spring return).
int rpc_punch()   { digitalWrite(PUNCH_PIN, HIGH); return 1; }
int rpc_retract() { digitalWrite(PUNCH_PIN, LOW);  return 1; }

// gateTest(pin, ms): drive ANY pin as a plain GPIO output — HIGH for ms, then LOW —
// and read the pad back at each step. Built to answer one question without a meter:
// "is this pin actually driving?", after `punch` (A3) stopped firing the solenoid
// while both servos kept working.
//
// Returned fields:
//   dr_hi/dr_lo — digitalRead of the pad itself while driven high / low. On STM32
//                 this reads the input data register, so it reflects the REAL pad
//                 level, not just what we asked for. 1/0 = the pin drives.
//   an_sense    — analogRead of SENSE_PIN, the gate node, once that jumper exists.
//
// There is deliberately NO analogRead of the DRIVEN pin: analogRead() re-muxes the pad
// to analog mode BEFORE sampling, so it measures the pin after the output driver has
// already been disconnected. Tried it 2026-08-14 — three pins in identical states read
// 1261 / 2109 / 1631. It is noise, and it left the pin needing a re-arm. Removed.
//
// Pin numbers are the Arduino enum: D0..D13 = 0..13, A0..A5 = 14..19 (so A3 = 17).
//
// ⚠️ SERVO PINS ARE REFUSED. The first version detached the servo, ran the test and
// re-attached — and detach()/attach() on this Zephyr core HUNG THE MCU stone dead
// (2026-08-14: gateTest(10) wedged the bridge; every later RPC, getBattery included,
// timed out until the MCU was reflashed). The Servo library owns a hardware counter
// on these pins and does not survive being taken away and given back at runtime.
// Diagnose a servo with the Servo API (indexSpool / dropSeed), never by re-muxing
// its pin. The refusal is the whole point of this guard — do not "improve" it.
String rpc_gateTest(int pin, int ms) {
  if (pin == SPOOL_PIN || pin == DRUM_PIN) {
    return String("{\"err\":\"servo pin - refused, detach() hangs the MCU\",\"pin\":")
           + String(pin) + "}";
  }
  ms = constrain(ms, 10, 3000);            // never leave a coil energised on a typo

  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(ms);
  int dr_hi    = digitalRead(pin);
  // A5 is i2c3 SCL now; analogRead() would re-mux it out from under the gyro.
  int an_sense = GATE_SENSE ? analogRead(SENSE_PIN) : -1;

  digitalWrite(pin, LOW);
  delay(120);
  int dr_lo       = digitalRead(pin);
  int an_sense_lo = GATE_SENSE ? analogRead(SENSE_PIN) : -1;

  pinMode(PUNCH_PIN, OUTPUT);              // leave the real punch pin safe + released
  digitalWrite(PUNCH_PIN, LOW);

  String s = "{\"pin\":" + String(pin) + ",\"ms\":" + String(ms);
  s += ",\"dr_hi\":" + String(dr_hi) + ",\"dr_lo\":" + String(dr_lo);
  s += ",\"an_sense\":" + String(an_sense) + ",\"an_sense_lo\":" + String(an_sense_lo);
  s += ",\"adc_max\":" + String(ADC_MAX, 0) + ",\"vref\":" + String(ADC_VREF, 3) + "}";
  if (s_mon_ok) Monitor.println("gateTest " + s);
  return s;
}

// plantSeed(): one full planting at the current spot —
//   punch tip into soil → drop seed through the tip → retract.
// The drum steps are inlined rather than calling rpc_dropSeed(), because the punch has
// to fire BETWEEN the drop and the fill — see the sequence note above. rpc_dropSeed()
// stays as-is for the standalone dropSeed RPC (drum only, never touches the punch).
int rpc_plantSeed() {
  drumServo.write(DRUM_DROP);      // 1. meter one seed out of the drum
  delay(DROP_MS);
  delay(PUNCH_DELAY_MS);           // 2. seed reaches the tip / the spot
  digitalWrite(PUNCH_PIN, HIGH);   // 3. drive it into the soil
  delay(PUNCH_HOLD_MS);
  digitalWrite(PUNCH_PIN, LOW);    // 4. spring retracts — COIL OFF (500 ms total)
  drumServo.write(DRUM_FILL);      // 5. drum re-fills, tip already up
  delay(FILL_MS);
  return 1;
}

// ──────────────────────────────────────────────────────────────────────────
void setup() {
  // IBT-2 enable pins — tie HIGH permanently (drivers always enabled)
  pinMode(L_R_EN, OUTPUT); digitalWrite(L_R_EN, HIGH);
  pinMode(L_L_EN, OUTPUT); digitalWrite(L_L_EN, HIGH);
  pinMode(R_R_EN, OUTPUT); digitalWrite(R_R_EN, HIGH);
  pinMode(R_L_EN, OUTPUT); digitalWrite(R_L_EN, HIGH);

  // PWM output pins (L/R RPWM+LPWM = 3,5,6,9 — the UNO Q's PWM-capable pins).
  // UNO Q / Zephyr 0.52 core BUG: calling pinMode() on a PWM pin *before*
  // analogWrite() breaks PWM on that pin (outputs ~0V) — analogWrite() must own
  // the pin config. So we do NOT pinMode() these; rpc_stop() below (analogWrite 0)
  // configures them. Ref: forum.arduino.cc/t/...1419189
  analogWriteResolution(8);   // 255 = full duty (explicit; core default is 8-bit)

  // Safe start — motors stopped
  rpc_stop();

  // ── Soil sensors (Stage 2) ──
  analogReadResolution(14);           // 0–16383 (matches plan.md / 14-bit ADC)
  pinMode(EC_DRIVE, OUTPUT);
  digitalWrite(EC_DRIVE, LOW);
  tempSensor.begin();

  // ── Seeder ──
  pinMode(PUNCH_PIN, OUTPUT);
  digitalWrite(PUNCH_PIN, LOW);       // punch released
  spoolServo.attach(SPOOL_PIN, 500, 2500);   // wide pulse -> full travel
  drumServo.attach(DRUM_PIN,  500, 2500);
  spoolServo.write(0);
  drumServo.write(DRUM_FILL);

  // ── Gyro (yaw) ──
  // Failure is NOT fatal: imuBegin() returning false leaves s_imu_ok clear, getGyro
  // reports why, and pivotDeg refuses so the caller keeps using the timed pivot. The
  // robot must still drive with the sensor unplugged.
  imuBegin();

  // RouterBridge init — must come AFTER pin setup
  bool bridge_ok = Bridge.begin();

  // Monitor = the UNO Q's Serial replacement (D0/D1 are the bridge's UART, so
  // classic Serial isn't available). Read it headless with `arduino-app-cli monitor`.
  // Deliberately NOT `while (!Monitor)` like the library example — that would hang
  // the robot whenever nothing is attached.
  Monitor.begin(115200);
  s_mon_check_ms = millis();
  s_mon_ok = (bool)Monitor;
  if (s_mon_ok) {
    Monitor.println("--- farm_os boot ---");
    Monitor.println(String("build: ") + __DATE__ + " " + __TIME__);
    Monitor.println(String("bridge: ") + (bridge_ok ? "ok" : "FAILED"));
    Monitor.println(String("motors: L_RPWM=") + L_RPWM + " L_LPWM=" + L_LPWM +
                    " R_RPWM=" + R_RPWM + " R_LPWM=" + R_LPWM);
    Monitor.println(String("current sense: ") +
                    (CURRENT_SENSE ? "ENABLED" : "off (see CURRENT_SENSE)"));
  }

  Bridge.provide("ping",           rpc_ping);
  Bridge.provide("setMotors",      rpc_setMotors);
  Bridge.provide("stop",           rpc_stop);
  Bridge.provide("getMotorState",  rpc_getMotorState);
  Bridge.provide("getMoisture",    rpc_getMoisture);
  Bridge.provide("getTemperature", rpc_getTemperature);
  Bridge.provide("getEC",          rpc_getEC);
  Bridge.provide("getBattery",     rpc_getBattery);
  Bridge.provide("indexSpool",     rpc_indexSpool);
  Bridge.provide("dropSeed",       rpc_dropSeed);
  Bridge.provide("punch",          rpc_punch);
  Bridge.provide("retract",        rpc_retract);
  Bridge.provide("plantSeed",      rpc_plantSeed);
  Bridge.provide("gateTest",       rpc_gateTest);
  Bridge.provide("getDiag",        rpc_getDiag);
  Bridge.provide("getGyro",        rpc_getGyro);
  Bridge.provide("i2cScan",        rpc_i2cScan);
  Bridge.provide("i2cLines",       rpc_i2cLines);
  Bridge.provide("pivotDeg",       rpc_pivotDeg);
  Bridge.provide("gyroIntegrate",  rpc_gyroIntegrate);
  Bridge.provide("setGyroScale",   rpc_setGyroScale);

  if (s_mon_ok) Monitor.println("RPCs registered — ready");
}

void loop() {
  // RouterBridge handles incoming calls automatically.
#if CURRENT_SENSE
  // Sample motor current HERE, not in an RPC: the Linux side times its moves by
  // the gap between setMotors and stop, so it must not have to call in mid-hop to
  // see current. loop() accumulates while the motors are on; getDiag reports it after.
  if (s_left_pwm != 0 || s_right_pwm != 0) {
    float l = analogRead(IS_L_PIN) / ADC_MAX * ADC_VREF / IS_RESISTOR * IS_RATIO;
    float r = analogRead(IS_R_PIN) / ADC_MAX * ADC_VREF / IS_RESISTOR * IS_RATIO;
    s_amp_l_sum += l; s_amp_r_sum += r; s_amp_n++;
    if (l > s_amp_l_max) s_amp_l_max = l;
    if (r > s_amp_r_max) s_amp_r_max = r;

    // The IS signal is chopped by the PWM, so single reads land wherever the duty
    // cycle happens to be. 137us is deliberately not a neat fraction of the PWM
    // period, so successive samples walk across the phase instead of aliasing onto
    // one point — the running mean then converges on the true duty-weighted current.
    delayMicroseconds(137);

    // A side commanded but drawing no current = the drive never reached the motor
    // (loose IBT-2 signal/power lead). Rate-limited so it can't flood the monitor.
    static uint32_t warn_ms = 0;
    if (s_amp_n > 200 && (millis() - warn_ms) > 2000) {
      bool l_dead = (s_left_pwm  != 0) && (s_amp_l_sum / s_amp_n) < 0.05f;
      bool r_dead = (s_right_pwm != 0) && (s_amp_r_sum / s_amp_n) < 0.05f;
      if ((l_dead || r_dead) && monitorReady()) {
        warn_ms = millis();
        Monitor.println(String("WARN: commanded but no current — ") +
                        (l_dead ? "LEFT " : "") + (r_dead ? "RIGHT " : "") +
                        "(check IBT-2 wiring)");
      }
    }
  }
#endif
}
