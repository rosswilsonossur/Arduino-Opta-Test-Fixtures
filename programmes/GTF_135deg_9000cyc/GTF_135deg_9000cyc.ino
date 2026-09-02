/*
 * ============================================================================
 *  Generic Test Fixture — 135 DEGREE OSCILLATION, 9,000 CYCLES
 * ============================================================================
 *  Status            : Generated against GTF-CONOPS-001 Rev L [SW-08]
 *  Adapted from      : reference-implementation.ino (Rev K baseline).
 *                      Deviations from the reference: CONFIGURATION BLOCK
 *                      values only, plus MOVE_ARM_MS 1000 -> 500 ms because
 *                      the 1 s stroke equals the reference arming delay and
 *                      the guard must be shorter than the shortest move.
 *                      No structural changes.
 *  Target            : Arduino Opta WiFi (Arduino IDE)            [SW-01]
 *  Libraries         : ArduinoRS485, ArduinoModbus                [SW-02]
 *  Drive             : StepperOnline CL57RS, Modbus RTU slave ID 1 (core)
 *  Stack light       : Waveshare Modbus RTU Relay 4CH, slave ID 2 (optional)
 *                      ch1/2/3 = Green/Amber/Red, FC05 coilWrite only [SW-13]
 *  Serial bus        : 9600 8N1, shared trunk, both slaves (COM-02)
 *
 *  This build (interrogation of 2026-08-27, confirmed):
 *    - Stroke: 135 deg = 3,750 pulses @ 23 rpm (0.98 s/stroke), relative,
 *      alternating sign, first stroke clockwise (+)
 *    - Dwells: 0 after outward stroke, 2,000 ms after return (SW-10)
 *    - Cycles: 9,000, dead reckoning (CTL-09) - ~3.96 s/cycle, ~9.9 h total
 *    - CTL-11 Stop: all stack lights flash 3x at 1 s, then Amber (idle)
 *    - Panel Start/Pause/Stop on I3/I4/I5 (standard hardware, CTL-03/05/11)
 *    - Stack light fitted (relay module ID 2); absence is non-fatal [SW-12]
 *    - Guard / restart: not fitted in this build
 *    - Peak current: 3.0 A written to Pr5.00 and verified at setup (SW-09)
 *
 *  Safety (ConOps §3 — do not modify):
 *    SAF-02  E-stop channel 2 on I1, HIGH = healthy (NC chain closed)
 *    SAF-04  I1 LOW -> immediate quick-stop (0x6002 = 0x0040), E-STOPPED
 *    SAF-05  Recovery requires E-stop reset AND a fresh start press;
 *            restart is always from the beginning, never resume
 *    AI-04   Software-forced enable (register 0x000F) is NOT used
 *
 *  NOTE on software E-stop latency: Modbus transactions block; with a 100 ms
 *  timeout and one retry a failing read can stall a loop pass ~200 ms before
 *  I1 is next sampled. This is acceptable ONLY because E-stop channel 1
 *  removes drive enable in hardware (SAF-01) — the software channel is
 *  supervisory. Do not raise MODBUS_TIMEOUT_MS without reviewing this.
 *
 *  Heartbeat [SW-11]: the onboard LED_USER (above the RESET button) blinks
 *  every 500 ms unconditionally, placed first in loop(). This is the only
 *  proof the control loop is genuinely alive — a spinning motor in velocity/
 *  RUNNING states is NOT proof, since a triggered move continues under the
 *  drive's own control even if the Opta's loop has hung. If the heartbeat
 *  ever stops, treat it as a hang and use the hardware E-stop / power off.
 * ============================================================================
 */

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

/* ============================================================================
 *  CONFIGURATION BLOCK  [SW-04]
 *  Retune the fixture by editing this block only. Every constant here is
 *  referenced by the code below — do not add placebo values. [SW-10]
 * ============================================================================
 */
namespace Config {

  // ---- Motion ---------------------------------------------------------
  constexpr int32_t  STROKE_PULSES       = 3750L;    // 135 deg = 3750 of 10000 P/R
  constexpr int32_t  FIRST_STROKE_SIGN   = +1;       // +1 first stroke CW, -1 CCW
  constexpr uint16_t SPEED_RPM           = 23;       // 135 deg in ~1 s (22.5 rpm exact; register is integer rpm, 23 -> 0.978 s/stroke)
  constexpr uint16_t ACCEL_MS_PER_KRPM   = 50;       // Pr9.04
  constexpr uint16_t DECEL_MS_PER_KRPM   = 50;       // Pr9.05
  constexpr uint32_t DWELL_AFTER_OUT_MS  = 0;        // no dwell at the far end [SW-10]
  constexpr uint32_t DWELL_AFTER_BACK_MS = 2000UL;   // 2 s hold after each return [SW-10]

  // ---- Cycles ---------------------------------------------------------
  constexpr uint32_t CYCLE_TARGET        = 9000;     // out-and-back pairs (~9.9 h at ~3.96 s/cycle)

  // ---- Drive ----------------------------------------------------------
  constexpr uint16_t PEAK_CURRENT_01A    = 30;       // Pr5.00, units 0.1 A (30 = 3.0 A) [SW-09]
  constexpr uint16_t PULSES_PER_REV      = 10000;    // Pr0.00 default (informational)

  // ---- Timing ---------------------------------------------------------
  constexpr uint32_t DEBOUNCE_MS         = 30;       // CTL-04 (millis()-based, never delay())
  constexpr uint32_t BLINK_FAULT_MS      = 2000;     // E-STOPPED / FAULT: 2 s blink
  constexpr uint32_t BLINK_STACK_AMBER_MS = 2000;    // PAUSED: stack-light amber 2 s blink
  constexpr uint8_t  STACK_FLASH_COUNT    = 3;       // CTL-11 reset annunciation: ON pulses
  constexpr uint32_t STACK_FLASH_INTERVAL_MS = 1000; // CTL-11 reset annunciation: edge interval
  constexpr uint32_t BLINK_PAUSE_MS = BLINK_STACK_AMBER_MS; // PAUSED: pilot blink TIED to the
                                                     // stack amber (ConOps 7.1 Rev K) so the two
                                                     // lamps toggle in step - do not split them
  constexpr uint32_t POLL_MS             = 100;      // completion-poll interval while moving
  constexpr uint32_t ALARM_POLL_MS       = 500;      // alarm/status poll interval while moving
  constexpr uint32_t HEARTBEAT_MS        = 500;      // LED_USER blink - proves the loop is alive [SW-11]
  constexpr uint32_t MOVE_ARM_MS         = 500;      // ignore "complete" until this long after
                                                     // trigger (stale-status race guard). Reduced
                                                     // from the reference 1000 ms: strokes here
                                                     // are 1 s and this MUST stay shorter.

  // ---- Modbus ---------------------------------------------------------
  constexpr uint8_t  SLAVE_ID            = 1;        // COM-01 (CL57RS, core)
  constexpr uint8_t  RELAY_SLAVE_ID      = 2;        // COM-01/COM-06 (relay module, optional)
  constexpr uint32_t BAUD                = 9600;     // COM-02 - locked to the relay module's
                                                     // factory default; its baud changes do NOT
                                                     // survive power cycles (COM-06). CL57RS set
                                                     // to match: Pr5.22=2 saved to EEPROM.
  constexpr uint32_t MODBUS_TIMEOUT_MS   = 100;      // per transaction; see latency note above

  // ---- Stack light ----------------------------------------------------
  constexpr bool     STACKLIGHT_FITTED   = true;     // set false on builds without the relay module
}

/* ============================================================================
 *  CL57RS register addresses (ConOps §6 ONLY — never invent addresses)
 * ============================================================================
 */
namespace Reg {
  constexpr uint16_t PEAK_CURRENT  = 0x0191; // Pr5.00, 0.1 A units
  constexpr uint16_t PR0_BASE      = 0x6200; // Pr9.00..Pr9.07, all 8 in one FC10
  constexpr uint16_t TRIGGER       = 0x6002; // write 0x0040 quick stop; read 0x0000 complete
  constexpr uint16_t MOTION_STATUS = 0x1003; // bit0 fault, bit2 running
  constexpr uint16_t ALARM_CODE    = 0x2203; // 0 = no alarm
  constexpr uint16_t CONTROL_WORD  = 0x1801; // write 0x1111 reset current alarm
}

namespace RelayCoil { // relay module, slave ID 2 (ConOps 6.5) - FC05 coilWrite ONLY [SW-13]
  constexpr uint16_t GREEN = 0x0000;
  constexpr uint16_t AMBER = 0x0001;
  constexpr uint16_t RED   = 0x0002;
}
constexpr uint16_t RELAY_REG_VERSION = 0x8000; // FC03 liveness read for the SW-12 boot check

constexpr uint16_t PR0_MODE_REL_POS = 0x0041; // relative position mode (MOT-03)
constexpr uint16_t QUICK_STOP_CMD   = 0x0040; // SAF-04
constexpr uint16_t ALARM_RESET_CMD  = 0x1111;

/* ============================================================================
 *  I/O map (ConOps §4)
 * ============================================================================
 */
constexpr pin_size_t PIN_ESTOP = A0; // I1 - E-stop ch.2, HIGH = healthy
constexpr pin_size_t PIN_START = A2; // I3 - panel Start (green), HIGH = pressed (NO)
constexpr pin_size_t PIN_PAUSE = A3; // I4 - panel Pause (amber), HIGH = pressed (NO)
constexpr pin_size_t PIN_STOP  = A4; // I5 - panel Stop (red),   HIGH = pressed (NO) - CTL-11
constexpr pin_size_t PIN_PILOT = D0; // Relay 1 - pilot light, energised = ON
// BTN_USER (onboard) — active LOW when pressed — must ALWAYS be implemented

/* ============================================================================
 *  State machine (ConOps §7.1). Unfitted features (PAUSED, GUARD_PAUSED,
 *  RESTART_DWELL) are omitted from this reference build.
 * ============================================================================
 */
enum class SysState : uint8_t { IDLE, RUNNING, PAUSED, COMPLETE, E_STOPPED, FAULT };
enum class RunPhase : uint8_t { TRIGGER_OUT, MOVING_OUT, DWELL_A,
                                TRIGGER_BACK, MOVING_BACK, DWELL_B };

SysState    state       = SysState::IDLE;
RunPhase    phase       = RunPhase::TRIGGER_OUT;
uint32_t    cycleCount  = 0;
uint16_t    lastAlarm   = 0;
const char* faultReason = "";

uint32_t phaseT0 = 0, lastPoll = 0, lastAlarmPoll = 0;
bool     pilotOn = false;
uint32_t pilotT0 = 0;
bool     heartbeatOn = false;
uint32_t lastHeartbeat = 0; // [SW-11]

/* ---- Buttons: debounced, edge-triggered, millis()-based [CTL-04, SW-03] --*/
struct Button {
  pin_size_t pin;
  bool activeLow;
  bool stable;
  bool lastRead;
  uint32_t lastChange;
  bool pressedEvent; // true for exactly one loop pass on a debounced press edge
};
Button btnUser  = { BTN_USER,  true,  false, false, 0, false };
Button btnStart = { PIN_START, false, false, false, 0, false };
Button btnPause = { PIN_PAUSE, false, false, false, 0, false };
Button btnStop  = { PIN_STOP,  false, false, false, 0, false };

/* ============================================================================
 *  Serial logging  [SW-07]
 * ============================================================================
 */
const char* stateName(SysState s) {
  switch (s) {
    case SysState::IDLE:      return "IDLE";
    case SysState::RUNNING:   return "RUNNING";
    case SysState::PAUSED:    return "PAUSED";
    case SysState::COMPLETE:  return "COMPLETE";
    case SysState::E_STOPPED: return "E-STOPPED";
    case SysState::FAULT:     return "FAULT";
  }
  return "?";
}

void logStatus(const char* note) {
  Serial.print(F("[t="));
  Serial.print(millis());
  Serial.print(F(" ms] state="));
  Serial.print(stateName(state));
  Serial.print(F(" cycles="));
  Serial.print(cycleCount);
  Serial.print(F("/"));
  Serial.print(Config::CYCLE_TARGET);
  if (lastAlarm != 0) { Serial.print(F(" alarm=0x")); Serial.print(lastAlarm, HEX); }
  Serial.print(F(" — "));
  Serial.println(note);
}

// Single state-entry point: guards against re-entry side effects.
// Also the ONLY place stack-light writes happen: on state change, never
// every loop pass [SW-13].
void updateStackLight(); // fwd
uint8_t stackFlashEdgesLeft = 0; // CTL-11 reset-flash overlay; any state change cancels it
bool stackFlashPending = false;  // set by the Stop handlers BEFORE enterState, so the
                                 // flash starts directly - no amber pre-flick
void enterState(SysState s, const char* note) {
  state   = s;
  pilotT0 = millis();
  pilotOn = false;
  stackFlashEdgesLeft = 0; // cancel any CTL-11 reset flash in progress
  logStatus(note);
  updateStackLight();
}

/* ============================================================================
 *  Modbus helpers — ONLY these ArduinoModbus calls are permitted (ConOps
 *  Appendix B). Each transaction retried once, then FAULT. [SW-05, COM-05]
 * ============================================================================
 */
bool mbWriteSingle(uint16_t addr, uint16_t value) {
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    if (ModbusRTUClient.holdingRegisterWrite(Config::SLAVE_ID, addr, value)) return true;
  }
  return false;
}

bool mbWriteMulti(uint16_t addr, const uint16_t* values, uint8_t count) {
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    ModbusRTUClient.beginTransmission(Config::SLAVE_ID, HOLDING_REGISTERS, addr, count);
    for (uint8_t i = 0; i < count; i++) ModbusRTUClient.write(values[i]);
    if (ModbusRTUClient.endTransmission()) return true;
  }
  return false;
}

bool mbReadSingle(uint16_t addr, uint16_t& out) {
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    if (ModbusRTUClient.requestFrom(Config::SLAVE_ID, HOLDING_REGISTERS, addr, 1) == 1) {
      long v = ModbusRTUClient.read();
      if (v >= 0) { out = (uint16_t)v; return true; }
    }
  }
  return false;
}

/* ============================================================================
 *  Drive commands
 * ============================================================================
 */
bool driveQuickStop() {
  return mbWriteSingle(Reg::TRIGGER, QUICK_STOP_CMD); // SAF-04
}

// Fire PR0 relative move: all 8 registers 0x6200–0x6207 in one FC10 frame (§6.2)
bool driveTriggerMove(int32_t pulses) {
  uint16_t frame[8];
  frame[0] = PR0_MODE_REL_POS;                              // Pr9.00 mode
  frame[1] = (uint16_t)(((uint32_t)pulses >> 16) & 0xFFFF); // Pr9.01 target high
  frame[2] = (uint16_t)((uint32_t)pulses & 0xFFFF);         // Pr9.02 target low
  frame[3] = Config::SPEED_RPM;                             // Pr9.03 velocity
  frame[4] = Config::ACCEL_MS_PER_KRPM;                     // Pr9.04 accel
  frame[5] = Config::DECEL_MS_PER_KRPM;                     // Pr9.05 decel
  frame[6] = 0;                                             // Pr9.06 drive pause = 0 (dwell in software; register is 16-bit ms)
  frame[7] = 0x0010;                                        // Pr9.07 trigger PR0 immediately
  return mbWriteMulti(Reg::PR0_BASE, frame, 8);
}

void goFault(const char* reason) {
  faultReason = reason;
  driveQuickStop(); // best effort; already faulting
  enterState(SysState::FAULT, reason);
}

/* ============================================================================
 *  Stack light (relay module, ID 2) - ConOps 6.5 / SW-12 / SW-13
 *  FC05 coilWrite() with plain 0/1 ONLY. FC0F is prohibited (COM-03): it
 *  draws an Illegal-function exception on this hardware/library combo.
 *  Sequential writes ordered OFF-then-ON so an interrupted sequence can
 *  leave the light dimmer than intended but never two colours at once.
 *  Failures are logged but non-fatal (COM-07): the light is an optional
 *  extra, the pilot light remains authoritative.
 * ============================================================================
 */
bool stackLightOk = false; // set by the SW-12 boot check; false = writes suppressed
bool stackAmberOn = false;         // paused-state amber blink phase
uint32_t stackAmberT0 = 0;         // last amber toggle time
bool stackFlashOn = false;         // CTL-11 reset-flash phase
uint32_t stackFlashT0 = 0;         // CTL-11 reset-flash last edge
void startStackResetFlash();       // fwd (called from updateStackLight on pending)

bool relayCoilWrite(uint16_t coil, bool on) {
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    if (ModbusRTUClient.coilWrite(Config::RELAY_SLAVE_ID, coil, on ? 1 : 0)) return true;
  }
  return false;
}

// ConOps 7.1 Rev J scheme:
//   IDLE     -> Amber solid            RUNNING -> Green solid
//   PAUSED   -> Green + Amber 2 s blink (amber toggled in stackLightTick)
//   COMPLETE -> Green + Amber solid    E-STOPPED / FAULT -> Red solid
// No state is all-off: a dark stack light means unpowered or module absent.
void updateStackLight() {
  // Stop-abort path: start the CTL-11 flash directly instead of writing the
  // IDLE colours first - avoids the ~75 ms amber-only flick before all-on.
  if (stackFlashPending) {
    stackFlashPending = false;
    startStackResetFlash();
    return;
  }
  if (!Config::STACKLIGHT_FITTED || !stackLightOk) return;
  bool green = (state == SysState::RUNNING || state == SysState::PAUSED ||
                state == SysState::COMPLETE);
  bool amber = (state == SysState::IDLE    || state == SysState::PAUSED ||
                state == SysState::COMPLETE);
  bool red   = (state == SysState::E_STOPPED || state == SysState::FAULT);
  stackAmberOn = amber;      // paused blink starts from ON
  bool ok = true;
  // OFF first, then ON [SW-13]
  if (!green) ok &= relayCoilWrite(RelayCoil::GREEN, false);
  if (!amber) ok &= relayCoilWrite(RelayCoil::AMBER, false);
  if (!red)   ok &= relayCoilWrite(RelayCoil::RED,   false);
  if (green)  ok &= relayCoilWrite(RelayCoil::GREEN, true);
  if (amber)  ok &= relayCoilWrite(RelayCoil::AMBER, true);
  if (red)    ok &= relayCoilWrite(RelayCoil::RED,   true);
  stackAmberT0 = millis(); // after the writes: first paused-blink interval full length
  if (!ok) logStatus("stack light write failed (non-fatal, COM-07) - pilot light remains authoritative");
}

void writeAllStack(bool on) {
  bool ok = true;
  ok &= relayCoilWrite(RelayCoil::GREEN, on);
  ok &= relayCoilWrite(RelayCoil::AMBER, on);
  ok &= relayCoilWrite(RelayCoil::RED,   on);
  if (!ok) logStatus("stack flash write failed (non-fatal, COM-07)");
}

// CTL-11 reset annunciation: all three lights flash together 3 times at a
// 1 s interval, then the state colours restore. Non-blocking overlay - no
// delay() [SW-03] - driven from stackLightTick(); a bounded burst, so it
// remains within SW-13's permitted writes. Cancelled by any state change.
// NOTE: the interval timer is stamped AFTER the writes complete - each FC05
// is a ~25 ms round trip at 9600, so stamping first would shave the write
// time off the FIRST interval only, making the first blink visibly short.
void startStackResetFlash() {
  if (!Config::STACKLIGHT_FITTED || !stackLightOk) return;
  stackFlashOn = true;
  stackFlashEdgesLeft = Config::STACK_FLASH_COUNT * 2 - 1; // edges after this initial ON
  writeAllStack(true);
  stackFlashT0 = millis(); // after the writes: interval measured lamp-to-lamp
}

// Periodic stack-light service: CTL-11 flash overlay first (it owns the
// light while active), then the PAUSED amber blink - the only periodic
// writes permitted by SW-13.
void stackLightTick(uint32_t now) {
  if (!Config::STACKLIGHT_FITTED || !stackLightOk) return;

  if (stackFlashEdgesLeft > 0) {
    if (now - stackFlashT0 >= Config::STACK_FLASH_INTERVAL_MS) {
      stackFlashT0 = now;
      stackFlashEdgesLeft--;
      if (stackFlashEdgesLeft == 0) { updateStackLight(); return; } // restore state colours
      stackFlashOn = !stackFlashOn;
      writeAllStack(stackFlashOn);
    }
    return;
  }

  if (state != SysState::PAUSED) return;
  if (now - stackAmberT0 >= Config::BLINK_STACK_AMBER_MS) {
    stackAmberT0 = now;
    stackAmberOn = !stackAmberOn;
    if (!relayCoilWrite(RelayCoil::AMBER, stackAmberOn))
      logStatus("stack light amber toggle failed (non-fatal, COM-07)");
  }
}

/* ============================================================================
 *  Buttons — debounced, edge-triggered  [CTL-04] — NO delay() [SW-03]
 * ============================================================================
 */
void updateButton(Button& b, uint32_t now) {
  bool raw = digitalRead(b.pin);
  bool pressed = b.activeLow ? (raw == LOW) : (raw == HIGH);
  if (pressed != b.lastRead) { b.lastRead = pressed; b.lastChange = now; }
  b.pressedEvent = false;
  if ((now - b.lastChange) >= Config::DEBOUNCE_MS && pressed != b.stable) {
    b.stable = pressed;
    if (pressed) b.pressedEvent = true; // rising edge of debounced press only
  }
}

// CTL-03: remote button mirrors BTN_USER exactly. Edge events, never levels.
bool startPressed() { return btnUser.pressedEvent || btnStart.pressedEvent; }

// The drive's enable bit (0x1003 bit1) must be verified before any trigger.
// After an E-stop, hardware channel 1 (DI1) can still be open - or restore a
// beat later than the I1 chain - even though I1 already reads healthy.
// Triggering while disabled either leaves the trigger pending forever (sketch
// waits on a move that never runs) or silently discards it (phantom cycle
// counts). Refuse loudly instead; the operator presses start again once the
// drive reports enabled.
bool driveEnabledOk();  // fwd (uses mbReadSingle, defined later)

/* ============================================================================
 *  Pilot light (§7.1 - RUNNING is solid on, E_STOPPED/FAULT blink, all
 *  other states off. IDLE and COMPLETE are both off, intentionally: off
 *  means safe to approach, solid-on unambiguously means "cycling now".)
 *  [SW-03]
 * ============================================================================
 */
void updatePilot(uint32_t now) {
  uint32_t period = 0;
  bool solid = false, off = false;
  switch (state) {
    case SysState::IDLE:      off = true;                      break;
    case SysState::RUNNING:   solid = true;                    break;
    case SysState::PAUSED:    period = Config::BLINK_PAUSE_MS; break;
    case SysState::COMPLETE:  off = true;                      break;
    case SysState::E_STOPPED:
    case SysState::FAULT:     period = Config::BLINK_FAULT_MS; break;
  }
  if (off)        pilotOn = false;
  else if (solid) pilotOn = true;
  else if (now - pilotT0 >= period) { pilotOn = !pilotOn; pilotT0 = now; }
  digitalWrite(PIN_PILOT, pilotOn ? HIGH : LOW);
}

/* ============================================================================
 *  Run-programme helpers
 * ============================================================================
 */
int32_t strokeForPhase(RunPhase p) {
  int32_t s = Config::STROKE_PULSES * Config::FIRST_STROKE_SIGN;
  return (p == RunPhase::TRIGGER_BACK || p == RunPhase::MOVING_BACK) ? -s : s;
}

bool driveEnabledOk() {
  uint16_t status;
  if (!mbReadSingle(Reg::MOTION_STATUS, status)) {
    logStatus("enable check: Modbus read failed - start refused");
    return false;
  }
  if ((status & 0x0002) == 0) {
    logStatus("start refused - drive reports DISABLED (E-stop channel 1 / DI1 not restored?)");
    return false;
  }
  return true;
}

void startProgramme() {
  if (!driveEnabledOk()) return;  // stays in current state; press start again
  cycleCount = 0;
  lastAlarm  = 0;
  phase      = RunPhase::TRIGGER_OUT;
  enterState(SysState::RUNNING, "programme start (from beginning)");
}

// On pause during a move, rewind the phase to that move's trigger so resume
// re-fires the stroke (see the envelope-shift note at the PAUSED case).
// Dwell phases resume in place with their timer restarted.
void pauseProgramme() {
  if      (phase == RunPhase::MOVING_OUT)  phase = RunPhase::TRIGGER_OUT;
  else if (phase == RunPhase::MOVING_BACK) phase = RunPhase::TRIGGER_BACK;
  else                                     phaseT0 = millis();
}

// Completion = 0x6002 reads 0x0000 AND 0x1003 running bit clear, after the
// arming delay. Dual-source check guards the stale-status race.
// Returns true if a fault was raised (caller bails out of RUNNING logic).
bool pollDriveWhileMoving(uint32_t now) {
  if (now - lastPoll >= Config::POLL_MS) {
    lastPoll = now;
    uint16_t trig, status;
    if (!mbReadSingle(Reg::TRIGGER, trig) ||
        !mbReadSingle(Reg::MOTION_STATUS, status)) {
      goFault("Modbus failure reading 0x6002/0x1003 (COM-05)");
      return true;
    }
    bool running = (status & 0x0004) != 0; // bit2
    if (trig == 0x0000 && !running && (now - phaseT0) >= Config::MOVE_ARM_MS) {
      if (phase == RunPhase::MOVING_OUT) {
        phase = RunPhase::DWELL_A; phaseT0 = now;
        logStatus("outward stroke complete -> dwell A");
      } else if (phase == RunPhase::MOVING_BACK) {
        cycleCount++; // CTL-09: count on return-move complete
        phase = RunPhase::DWELL_B; phaseT0 = now;
        logStatus("return stroke complete -> cycle counted -> dwell B");
      }
    }
  }
  if (now - lastAlarmPoll >= Config::ALARM_POLL_MS) {
    lastAlarmPoll = now;
    uint16_t alarm, status;
    if (!mbReadSingle(Reg::ALARM_CODE, alarm) ||
        !mbReadSingle(Reg::MOTION_STATUS, status)) {
      goFault("Modbus failure reading status/alarm (COM-05)");
      return true;
    }
    if (alarm != 0 || (status & 0x0001)) {
      lastAlarm = alarm;
      goFault("drive alarm / status fault bit set");
      return true;
    }
  }
  return false;
}

void runProgramme(uint32_t now) {
  switch (phase) {

    case RunPhase::TRIGGER_OUT:
    case RunPhase::TRIGGER_BACK: {
      if (!driveTriggerMove(strokeForPhase(phase))) {
        goFault("Modbus failure triggering PR0 move (COM-05)");
        return;
      }
      phaseT0 = now; lastPoll = now; lastAlarmPoll = now;
      if (phase == RunPhase::TRIGGER_OUT) {
        phase = RunPhase::MOVING_OUT;  logStatus("outward stroke triggered");
      } else {
        phase = RunPhase::MOVING_BACK; logStatus("return stroke triggered");
      }
      break;
    }

    case RunPhase::MOVING_OUT:
    case RunPhase::MOVING_BACK:
      if (pollDriveWhileMoving(now)) return;
      break;

    case RunPhase::DWELL_A: // after outward stroke; 0 ms passes straight through
      if (now - phaseT0 >= Config::DWELL_AFTER_OUT_MS) phase = RunPhase::TRIGGER_BACK;
      break;

    case RunPhase::DWELL_B: // after return stroke; 0 ms passes straight through
      if (now - phaseT0 >= Config::DWELL_AFTER_BACK_MS) {
        if (cycleCount >= Config::CYCLE_TARGET) {
          enterState(SysState::COMPLETE, "all cycles complete");
        } else {
          phase = RunPhase::TRIGGER_OUT;
          logStatus("dwell complete -> next cycle");
        }
      }
      break;
  }
}

/* ============================================================================
 *  Setup — first Modbus transaction is the SW-09 current write, so a wiring
 *  fault announces itself as FAULT at boot, before any start press.
 * ============================================================================
 */
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* bounded wait for monitor */ }

  pinMode(PIN_ESTOP, INPUT);
  pinMode(PIN_START, INPUT);
  pinMode(PIN_PAUSE, INPUT);
  pinMode(PIN_STOP,  INPUT);
  pinMode(BTN_USER,  INPUT);
  pinMode(PIN_PILOT, OUTPUT);
  digitalWrite(PIN_PILOT, LOW);
  pinMode(LED_USER,  OUTPUT);
  digitalWrite(LED_USER, HIGH); // [SW-11] proof of power-on before anything else runs

  // RS-485 timing for the Opta transceiver (standard Opta Modbus pattern)
  const float bitduration = 1.0f / (float)Config::BAUD;
  const uint32_t preDelay  = (uint32_t)(bitduration * 9.6f * 3.5f * 1e6f);
  const uint32_t postDelay = (uint32_t)(bitduration * 9.6f * 3.5f * 1e6f);
  RS485.setDelays(preDelay, postDelay);

  if (!ModbusRTUClient.begin(Config::BAUD, SERIAL_8N1)) {   // COM-02
    enterState(SysState::FAULT, "ModbusRTUClient.begin() failed");
    return;
  }
  ModbusRTUClient.setTimeout(Config::MODBUS_TIMEOUT_MS);

  // SW-09: write and verify peak current before any motion is possible.
  uint16_t rb = 0;
  if (!mbWriteSingle(Reg::PEAK_CURRENT, Config::PEAK_CURRENT_01A) ||
      !mbReadSingle(Reg::PEAK_CURRENT, rb) ||
      rb != Config::PEAK_CURRENT_01A) {
    enterState(SysState::FAULT, "peak current write/verify failed (SW-09)");
    return;
  }
  logStatus("peak current set and verified (Pr5.00)");

  // SW-12 boot check, relay-module half. (The CL57RS half is the SW-09
  // write-and-verify above - its failure has already latched FAULT.)
  // Relay module is an OPTIONAL extra: absence warns and continues with
  // stack-light writes suppressed; everything else runs normally (COM-07).
  if (Config::STACKLIGHT_FITTED) {
    if (ModbusRTUClient.requestFrom(Config::RELAY_SLAVE_ID, HOLDING_REGISTERS,
                                    RELAY_REG_VERSION, 1) == 1 &&
        ModbusRTUClient.read() >= 0) {
      stackLightOk = true;
      logStatus("stack light detected (relay module ID 2)");
    } else {
      logStatus("stack light NOT detected - continuing without it (SW-12, non-fatal)");
    }
  }

  enterState(SysState::IDLE, "power-up OK — press start when ready (I1 must be healthy)");
}

/* ============================================================================
 *  Main loop — E-stop check first, always  [SW-06]
 * ============================================================================
 */
void loop() {
  uint32_t now = millis();

  // [SW-11] Heartbeat: proves the loop is executing, independent of system
  // state and independent of serial connection. Placed first, before even
  // the E-stop check, so ANY hang anywhere else in loop() shows up as the
  // heartbeat stopping. A running motor is NOT proof of a live loop: once
  // triggered, motion continues under the drive's own control regardless.
  if (now - lastHeartbeat >= Config::HEARTBEAT_MS) {
    lastHeartbeat = now;
    heartbeatOn = !heartbeatOn;
    digitalWrite(LED_USER, heartbeatOn ? HIGH : LOW);
  }

  // SAF-04: E-stop channel 2, before any other logic. Guarded against
  // re-entry so a held E-stop does not spam the log or the bus.
  bool estopHealthy = (digitalRead(PIN_ESTOP) == HIGH);
  if (!estopHealthy && state != SysState::E_STOPPED) {
    driveQuickStop();
    enterState(SysState::E_STOPPED, "E-STOP: I1 LOW — quick stop commanded");
  }

  updateButton(btnUser,  now);
  updateButton(btnStart, now);
  updateButton(btnPause, now);
  updateButton(btnStop,  now);

  switch (state) {

    case SysState::IDLE:
      if (startPressed()) {
        if (estopHealthy) startProgramme();                 // SAF-02
        else logStatus("start ignored — E-stop chain not healthy (I1 LOW)");
      }
      break;

    case SysState::RUNNING:
      // CTL-02: start does nothing while RUNNING
      if (btnStop.pressedEvent) {                      // CTL-11: abort to IDLE
        driveQuickStop();
        stackFlashPending = true;                      // CTL-11 reset annunciation
        enterState(SysState::IDLE, "STOP pressed - programme abandoned (restart is from the beginning)");
        break;
      }
      if (btnPause.pressedEvent) {                     // CTL-05
        driveQuickStop();
        pauseProgramme();
        enterState(SysState::PAUSED, "PAUSE pressed - motion quick-stopped");
        break;
      }
      runProgramme(now);
      break;

    case SysState::PAUSED:
      // CTL-05: start resumes. NOTE (ConOps 7.2): a stroke interrupted
      // mid-move is re-triggered in FULL on resume - relative moves lose
      // the aborted remainder, shifting the oscillation envelope. Harmless
      // for free rotation; a travel-limit consideration otherwise.
      if (btnStop.pressedEvent) {                      // CTL-11
        stackFlashPending = true;                      // CTL-11 reset annunciation
        enterState(SysState::IDLE, "STOP pressed from PAUSED - programme abandoned");
        break;
      }
      if (startPressed() && estopHealthy) {
        if (driveEnabledOk()) enterState(SysState::RUNNING, "resumed from PAUSED");
      }
      break;

    case SysState::COMPLETE:
      if (startPressed() && estopHealthy) startProgramme(); // OI-4: fresh run
      break;

    case SysState::E_STOPPED:
      // SAF-05: reset alone does NOT restart; requires reset + fresh press,
      // and always from the beginning. NOTE (ConOps §7.2): relative moves
      // resume from wherever the shaft stopped — envelope shifts after abort.
      if (estopHealthy && startPressed()) startProgramme();
      break;

    case SysState::FAULT:
      if (startPressed() && estopHealthy) {
        // Attempt alarm reset (0x1801 = 0x1111), VERIFY 0x2203 == 0, then run.
        uint16_t alarm = 0xFFFF;
        if (mbWriteSingle(Reg::CONTROL_WORD, ALARM_RESET_CMD) &&
            mbReadSingle(Reg::ALARM_CODE, alarm) && alarm == 0) {
          logStatus("fault cleared — restarting from beginning");
          startProgramme();
        } else {
          lastAlarm = alarm;
          logStatus("fault NOT cleared — investigate drive (some alarms need a power cycle)");
        }
      }
      break;
  }

  updatePilot(now);
  stackLightTick(now); // PAUSED amber blink [SW-13]
}
