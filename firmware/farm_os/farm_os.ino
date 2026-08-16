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

// ── Soil sensor pins (Stage 2) ──────────────────────────────────────────────
#define MOIST_A0   A0   // capacitive moisture #1 (powered at 3.3V!)
#define MOIST_A1   A1   // capacitive moisture #2 (powered at 3.3V!)
#define EC_SENSE   A2   // DIY EC probe sense node (22kΩ pulldown)
#define EC_DRIVE   13   // EC AC drive (was D10 — moved off the seeder spool; D13 freed
                        // by the single-solenoid-driver decision). Soil probe + seeder
                        // are mutually-exclusive attachments.
#define TEMP_PIN    2   // DS18B20 1-Wire (D2) + 4.7kΩ pull-up to 3.3V

// ── Battery monitor ─────────────────────────────────────────────────────────
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
static float readBattRaw() {
  const int WARMUP = 4, N = 32;
  for (int i = 0; i < WARMUP; i++) (void)analogRead(BATT_PIN);
  long acc = 0;
  for (int i = 0; i < N; i++) acc += analogRead(BATT_PIN);
  return acc / (float)N;
}

OneWire           ec_oneWire(TEMP_PIN);
DallasTemperature tempSensor(&ec_oneWire);

// ── Seeder pins (per uno-q-wiring.md v2 pinout) ─────────────────────────────
#define SPOOL_PIN  10   // S3003 spool drive (1:1 gear / direct)
#define DRUM_PIN   11   // SG90 drum metering (servos need a DIGITAL pin on UNO Q)
#define PUNCH_PIN  A3   // solenoid MOSFET gate — ONE driver fires BOTH punches (on/off)
// OPTIONAL gate sense: jumper the driver board's GATE node to A5 (the only free
// GPIO) and gateTest reports the actual gate voltage instead of guessing. Reads
// floating noise until that wire is fitted — an unwired sense is not a fault.
#define SENSE_PIN  A5

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
  float raw   = readBattRaw();
  float volts = raw / ADC_MAX * ADC_VREF * BATT_MULT;

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
  float vnode = readBattRaw() / ADC_MAX * ADC_VREF;
  float volts = vnode * BATT_MULT;
  int   pct   = (int)(battCellToPct(volts / 3.0f) + 0.5f);
  return "{\"volts\":" + String(volts, 2) + ",\"pct\":" + String(pct) + "}";
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
  int an_sense = analogRead(SENSE_PIN);    // a DIFFERENT pin, so the driven pad is untouched

  digitalWrite(pin, LOW);
  delay(120);
  int dr_lo       = digitalRead(pin);
  int an_sense_lo = analogRead(SENSE_PIN);

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
