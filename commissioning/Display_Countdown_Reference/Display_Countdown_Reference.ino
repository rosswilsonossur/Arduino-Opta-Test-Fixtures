/*
 * ============================================================================
 *  Display_Countdown_Reference — YK-LED4006-RS485 countdown display,
 *  standalone OFFLINE validation sketch
 * ============================================================================
 *  Status   : Reference implementation for bench validation, per
 *             docs/Display_Integration_Plan.md (GTF-DIP-001) against
 *             GTF-CONOPS-001 Rev L. Validate on the bench (checklist §7 of
 *             the plan) BEFORE lifting the display driver into the GTF
 *             reference implementation.
 *  Hardware : Arduino Opta WiFi + YK-LED4006-RS485 6-digit display ONLY.
 *             No motor, no relay module, no panel buttons required.
 *             Opta RS-485 A/B -> display A/B, common 0 V, display powered
 *             from bench 24 V (confirm the unit's marked range first, V1).
 *  Bus      : 9600 8N1 (COM-02 - the fixture trunk baud; also the display's
 *             factory default). The baud register 0x001C is never written.
 *  Address  : DISPLAY_ID below defaults to 3 - run the one-time tool
 *             commissioning/Display_OneTime_SetAddr3 first (recommended:
 *             validates the address the display will use on the real trunk).
 *             For a quick first-light on a factory-fresh unit, set
 *             DISPLAY_ID = 1 instead - but NEVER connect an ID-1 display
 *             to the fixture trunk (collides with the CL57RS).
 *
 *  WHAT THIS PROVES (the two display modes requested for the GTF):
 *    - COUNTDOWN CYCLES : display shows remaining cycles, decrementing on
 *      each (bench-simulated) cycle-complete event, exactly where the GTF
 *      hooks its cycleCount++ (CTL-09, return-stroke complete).
 *    - COUNTDOWN MINUTES: display shows remaining time to complete as
 *      minutes.seconds (e.g. 90.00 -> 89.59 -> ... -> 0.00), one write
 *      per visible second at most.
 *    Selected by the single global flag Config::MODE below.
 *
 *  Behaviour:
 *    boot  -> detect + initialise display, show the full target
 *    USER button (BTN_USER) -> start countdown
 *    USER during countdown  -> restart from the top
 *    countdown reaches 0    -> display blinks all digits; USER resets
 *    display unplugged/power-cycled at ANY point -> logged once, countdown
 *    keeps running, background 1 Hz probe re-detects, RE-INITIALISES (an
 *    unsaved display reverts to power-on defaults - float mode - on a power
 *    cycle) and restores the current value + blink state.  [V7/V8]
 *
 *  Contract: follows ConOps Appendix A even though this is a bench tool -
 *  permitted ArduinoModbus API only, one retry per transaction, RS485
 *  delays set before begin(), no delay(), all timing millis()-based,
 *  LED_USER heartbeat first in loop() [SW-11], bounded Serial wait.
 *  Serial monitor: 115200 baud.
 *
 *  The sections marked [LIFT] are the display driver intended to move
 *  verbatim into the GTF reference implementation after validation
 *  (integration hook points: plan §8).
 * ============================================================================
 */

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

enum class CountdownMode : uint8_t { CYCLES, MINUTES };

/* ============================================================================
 *  CONFIGURATION BLOCK  [SW-04]
 *  Every constant here is referenced by the code below.
 * ============================================================================
 */
namespace Config {

  // ---- THE GLOBAL MODE FLAG - pick one --------------------------------
  constexpr CountdownMode MODE = CountdownMode::CYCLES;   // or ::MINUTES

  // ---- Countdown targets ----------------------------------------------
  // Bench-friendly defaults so DONE is reachable while you watch; set the
  // real programme values (e.g. 9000 cycles / 90 minutes) when lifting
  // into the GTF, or for the V10 soak test.
  constexpr uint32_t CYCLE_TARGET        = 25;   // cycles mode: cycles to count down
  constexpr uint32_t MINUTES_TARGET      = 2;    // minutes mode: duration to count down

  // ---- Bench cycle simulator (cycles mode only) -----------------------
  // Stand-in for the fixture's real "return stroke complete" event
  // (CTL-09). One simulated cycle completes per period.
  constexpr uint32_t SIM_CYCLE_PERIOD_MS = 2000;

  // ---- Display / bus --------------------------------------------------
  constexpr uint8_t  DISPLAY_ID          = 3;    // commissioned address (factory ships at 1!)
  constexpr uint32_t BAUD                = 9600; // COM-02 - fixture trunk baud, display factory default
  constexpr uint32_t MODBUS_TIMEOUT_MS   = 100;  // per transaction
  constexpr uint16_t BRIGHTNESS          = 4;    // 0 (off) .. 6 (max)
  constexpr uint16_t BLINK_INTERVAL_CODE = 5;    // 1..10 -> 100..1000 ms (5 = 500 ms)
  constexpr uint16_t BLINK_MASK_ALL      = 0x003F; // all 6 digits (one bit per digit) [V6]
  constexpr uint32_t DISPLAY_RETRY_MS    = 1000; // reconnect probe period while offline

  // ---- Misc -----------------------------------------------------------
  constexpr uint32_t DEBOUNCE_MS         = 30;   // CTL-04 style, millis()-based
  constexpr uint32_t HEARTBEAT_MS        = 500;  // LED_USER blink [SW-11]
}

// 6-digit display limits: integer mode 999999 max; minutes shown as M*100+SS
static_assert(Config::CYCLE_TARGET >= 1 && Config::CYCLE_TARGET <= 999999UL,
              "CYCLE_TARGET must fit a 6-digit display");
static_assert(Config::MINUTES_TARGET >= 1 && Config::MINUTES_TARGET <= 9999UL,
              "MINUTES_TARGET must fit a 6-digit M.SS display");

/* ============================================================================
 *  [LIFT] YK-LED4006 register addresses (plan §2 ONLY - never invent)
 * ============================================================================
 */
namespace DispReg {
  constexpr uint16_t DECIMAL_POINT  = 0x0000; // decimal places; start of the FC10 value frame
  //                 0x0001, 0x0002           // 32-bit signed value, HIGH word first -
  //                                          // written in the same FC10 frame as 0x0000
  constexpr uint16_t DISPLAY_MODE   = 0x0017; // 0 = integer mode (this driver)
  constexpr uint16_t BRIGHTNESS     = 0x0018; // 0..6
  constexpr uint16_t BLINK_MASK     = 0x0019; // bit per digit, 0 = no blink
  constexpr uint16_t BLINK_INTERVAL = 0x001A; // 1..10 -> 100..1000 ms
  constexpr uint16_t VERSION        = 0x001F; // read-only - liveness check
}
constexpr uint16_t DISP_MODE_INTEGER = 0;

/* ============================================================================
 *  Bench state machine
 * ============================================================================
 */
enum class RefState : uint8_t { WAIT_START, COUNTING, DONE };
RefState state = RefState::WAIT_START;

uint32_t remainingCycles = Config::CYCLE_TARGET;         // cycles mode
uint32_t runStartMs      = 0;                            // minutes mode
uint32_t lastRemainS     = Config::MINUTES_TARGET * 60;  // minutes mode
uint32_t lastSimCycle    = 0;                            // cycles mode

bool     heartbeatOn   = false;
uint32_t lastHeartbeat = 0; // [SW-11]

/* ---- USER button: debounced, edge-triggered, millis()-based ------------- */
struct Button {
  pin_size_t pin;
  bool activeLow;
  bool stable;
  bool lastRead;
  uint32_t lastChange;
  bool pressedEvent; // true for exactly one loop pass on a debounced press edge
};
Button btnUser = { BTN_USER, true, false, false, 0, false };

void updateButton(Button& b, uint32_t now) {
  bool raw = digitalRead(b.pin);
  bool pressed = b.activeLow ? (raw == LOW) : (raw == HIGH);
  if (pressed != b.lastRead) { b.lastRead = pressed; b.lastChange = now; }
  b.pressedEvent = false;
  if ((now - b.lastChange) >= Config::DEBOUNCE_MS && pressed != b.stable) {
    b.stable = pressed;
    if (pressed) b.pressedEvent = true;
  }
}

/* ============================================================================
 *  Serial logging  [SW-07 style]
 * ============================================================================
 */
const char* stateName(RefState s) {
  switch (s) {
    case RefState::WAIT_START: return "WAIT_START";
    case RefState::COUNTING:   return "COUNTING";
    case RefState::DONE:       return "DONE";
  }
  return "?";
}

void printRemaining() {
  if (Config::MODE == CountdownMode::CYCLES) {
    Serial.print(F(" remaining="));
    Serial.print(remainingCycles);
    Serial.print(F("cyc"));
  } else {
    Serial.print(F(" remaining="));
    Serial.print(lastRemainS / 60);
    Serial.print(':');
    if ((lastRemainS % 60) < 10) Serial.print('0');
    Serial.print(lastRemainS % 60);
  }
}

void logLine(const char* note) {
  Serial.print(F("[t="));
  Serial.print(millis());
  Serial.print(F(" ms] state="));
  Serial.print(stateName(state));
  printRemaining();
  Serial.print(F(" — "));
  Serial.println(note);
}

/* ============================================================================
 *  [LIFT] Modbus helpers for the display - permitted ArduinoModbus API only
 *  (ConOps Appendix B.1). One retry per transaction; a final failure marks
 *  the display OFFLINE - never FAULT: the display is a non-fatal peripheral
 *  (COM-07 pattern, like the relay module), a cosmetic readout must never
 *  stop a test.
 * ============================================================================
 */
bool dispWriteSingle(uint16_t addr, uint16_t value) {
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    if (ModbusRTUClient.holdingRegisterWrite(Config::DISPLAY_ID, addr, value)) return true;
  }
  return false;
}

// One FC10 frame: [decimal places, value high, value low] at 0x0000..0x0002 -
// the combined write the vendor sheet recommends; atomic on the display.
bool dispWriteValue(int32_t value, uint16_t decimals) {
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    ModbusRTUClient.beginTransmission(Config::DISPLAY_ID, HOLDING_REGISTERS,
                                      DispReg::DECIMAL_POINT, 3);
    ModbusRTUClient.write(decimals);
    ModbusRTUClient.write((uint16_t)(((uint32_t)value >> 16) & 0xFFFF)); // high first
    ModbusRTUClient.write((uint16_t)((uint32_t)value & 0xFFFF));
    if (ModbusRTUClient.endTransmission()) return true;
  }
  return false;
}

bool dispDetect(uint16_t& versionOut) {
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    if (ModbusRTUClient.requestFrom(Config::DISPLAY_ID, HOLDING_REGISTERS,
                                    DispReg::VERSION, 1) == 1) {
      long v = ModbusRTUClient.read();
      if (v >= 0) { versionOut = (uint16_t)v; return true; }
    }
  }
  return false;
}

/* ============================================================================
 *  [LIFT] Display driver - desired-state model
 *  The application only sets desired{Value,Decimals,Blink}; displaySync()
 *  pushes differences to the device, and if the display drops off the bus
 *  it re-detects at 1 Hz, RE-INITIALISES (nothing is saved to the display's
 *  NVM, so a power-cycled display reverts to power-on defaults - float
 *  mode, brightness 4) and restores the full desired state.  [V7/V8]
 * ============================================================================
 */
bool     dispOnline      = false;
uint32_t dispLastAttempt = 0;
int32_t  desiredValue    = 0;
uint16_t desiredDecimals = 0;
uint16_t desiredBlink    = 0;
bool     shownValid      = false; // what the device currently displays is known
int32_t  shownValue      = 0;
uint16_t shownDecimals   = 0;
uint16_t shownBlink      = 0;

void dispMarkOffline(const char* reason) {
  if (dispOnline) {
    logLine(reason); // log the transition once, not every retry
    logLine("display OFFLINE - countdown continues, probing at 1 Hz (non-fatal, COM-07 pattern)");
  }
  dispOnline      = false;
  shownValid      = false;
  dispLastAttempt = millis();
}

// Full bring-up: detect, initialise, restore desired value + blink.
// Used at boot and by every reconnect. All-or-nothing: any failed write
// leaves the display offline for the next probe.
bool displayBringUp() {
  uint16_t version = 0;
  if (!dispDetect(version)) return false;
  if (!dispWriteSingle(DispReg::DISPLAY_MODE,   DISP_MODE_INTEGER))          return false;
  if (!dispWriteSingle(DispReg::BRIGHTNESS,     Config::BRIGHTNESS))         return false;
  if (!dispWriteSingle(DispReg::BLINK_INTERVAL, Config::BLINK_INTERVAL_CODE)) return false;
  if (!dispWriteValue(desiredValue, desiredDecimals))                        return false;
  if (!dispWriteSingle(DispReg::BLINK_MASK,     desiredBlink))               return false;
  shownValid    = true;
  shownValue    = desiredValue;
  shownDecimals = desiredDecimals;
  shownBlink    = desiredBlink;
  dispOnline    = true;
  Serial.print(F("[t="));
  Serial.print(millis());
  Serial.print(F(" ms] display ONLINE at ID "));
  Serial.print(Config::DISPLAY_ID);
  Serial.print(F(" - version register 0x"));
  Serial.print(version, HEX);
  Serial.println(F(", initialised, value restored"));
  return true;
}

void displaySync(uint32_t now) {
  if (!dispOnline) {
    if (now - dispLastAttempt >= Config::DISPLAY_RETRY_MS) {
      dispLastAttempt = now;
      displayBringUp(); // silent on failure - absence was already logged
    }
    return;
  }
  if (!shownValid || shownValue != desiredValue || shownDecimals != desiredDecimals) {
    if (!dispWriteValue(desiredValue, desiredDecimals)) {
      dispMarkOffline("display value write failed (regs 0x0000-0x0002) after retry");
      return;
    }
    shownValid = true; shownValue = desiredValue; shownDecimals = desiredDecimals;
  }
  if (shownBlink != desiredBlink) {
    if (!dispWriteSingle(DispReg::BLINK_MASK, desiredBlink)) {
      dispMarkOffline("display blink write failed (reg 0x0019) after retry");
      return;
    }
    shownBlink = desiredBlink;
  }
}

/* ============================================================================
 *  Countdown application - what feeds the display
 * ============================================================================
 */
int32_t minSecValue(uint32_t seconds) { // 5399 s -> 8959, shown "89.59" with 2 decimals
  return (int32_t)((seconds / 60) * 100 + (seconds % 60));
}

// Desired display content for the current remaining amount, per mode.
void setDesiredFromRemaining() {
  if (Config::MODE == CountdownMode::CYCLES) {
    desiredValue    = (int32_t)remainingCycles;
    desiredDecimals = 0;
  } else {
    desiredValue    = minSecValue(lastRemainS);
    desiredDecimals = 2; // minutes.seconds - confirms the 0x0000 semantics [V5]
  }
}

void startCountdown(uint32_t now, const char* why) {
  remainingCycles = Config::CYCLE_TARGET;
  lastRemainS     = Config::MINUTES_TARGET * 60;
  runStartMs      = now;
  lastSimCycle    = now;
  desiredBlink    = 0; // clear any DONE blink [V9]
  setDesiredFromRemaining();
  state = RefState::COUNTING;
  logLine(why);
}

void enterDone() {
  setDesiredFromRemaining();               // 0 or 0.00
  desiredBlink = Config::BLINK_MASK_ALL;   // attention: all digits blink [V6]
  state = RefState::DONE;
  logLine("countdown COMPLETE - display blinking; press USER to reset");
}

void tickCycles(uint32_t now) {
  if (now - lastSimCycle < Config::SIM_CYCLE_PERIOD_MS) return;
  lastSimCycle = now;
  // In the GTF this decrement sits exactly where cycleCount++ lives:
  // pollDriveWhileMoving(), return-stroke-complete branch (CTL-09).
  if (remainingCycles > 0) remainingCycles--;
  setDesiredFromRemaining();
  if (remainingCycles == 0) { enterDone(); return; }
  logLine("simulated cycle complete");
}

void tickMinutes(uint32_t now) {
  uint32_t totalS   = Config::MINUTES_TARGET * 60;
  uint32_t elapsedS = (now - runStartMs) / 1000;
  uint32_t remainS  = (elapsedS >= totalS) ? 0 : (totalS - elapsedS);
  if (remainS == lastRemainS) return;
  lastRemainS = remainS;
  setDesiredFromRemaining(); // at most one display write per visible second
  if (remainS == 0) { enterDone(); return; }
  if (remainS % 60 == 0 || remainS <= 10) logLine("countdown tick"); // minute marks + final 10 s
}

/* ============================================================================
 *  Setup
 * ============================================================================
 */
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* bounded wait for monitor */ }

  pinMode(BTN_USER, INPUT);
  pinMode(LED_USER, OUTPUT);
  digitalWrite(LED_USER, HIGH); // [SW-11] proof of power-on

  // RS-485 timing for the Opta transceiver - BEFORE begin() (Appendix B.2)
  const float bitduration = 1.0f / (float)Config::BAUD;
  const uint32_t turnaround = (uint32_t)(bitduration * 9.6f * 3.5f * 1e6f);
  RS485.setDelays(turnaround, turnaround);

  if (!ModbusRTUClient.begin(Config::BAUD, SERIAL_8N1)) {
    Serial.println(F("FATAL: ModbusRTUClient.begin() failed"));
    while (true) { /* bench tool: nothing sensible to do without a bus */ }
  }
  ModbusRTUClient.setTimeout(Config::MODBUS_TIMEOUT_MS);

  Serial.println(F("=== YK-LED4006 countdown display - offline validation reference ==="));
  Serial.print(F("mode="));
  Serial.print(Config::MODE == CountdownMode::CYCLES ? F("COUNTDOWN_CYCLES, target=")
                                                     : F("COUNTDOWN_MINUTES, target="));
  Serial.print(Config::MODE == CountdownMode::CYCLES ? Config::CYCLE_TARGET
                                                     : Config::MINUTES_TARGET);
  Serial.print(Config::MODE == CountdownMode::CYCLES ? F(" cycles (simulated every ")
                                                     : F(" minutes (display ID "));
  if (Config::MODE == CountdownMode::CYCLES) {
    Serial.print(Config::SIM_CYCLE_PERIOD_MS);
    Serial.print(F(" ms, display ID "));
  }
  Serial.print(Config::DISPLAY_ID);
  Serial.println(F(", bus 9600 8N1)"));

  setDesiredFromRemaining(); // show the full target while waiting
  if (!displayBringUp()) {
    Serial.println(F("display NOT detected - continuing, probing at 1 Hz "
                     "(check A/B polarity, power, and that the address matches "
                     "DISPLAY_ID - factory-fresh units answer at 1, run "
                     "Display_OneTime_SetAddr3)"));
    dispLastAttempt = millis();
  }
  logLine("ready - press USER button (BTN_USER) to start the countdown");
}

/* ============================================================================
 *  Main loop
 * ============================================================================
 */
void loop() {
  uint32_t now = millis();

  // [SW-11] Heartbeat first: proves the loop is alive whatever else happens.
  if (now - lastHeartbeat >= Config::HEARTBEAT_MS) {
    lastHeartbeat = now;
    heartbeatOn = !heartbeatOn;
    digitalWrite(LED_USER, heartbeatOn ? HIGH : LOW);
  }

  updateButton(btnUser, now);

  switch (state) {

    case RefState::WAIT_START:
      if (btnUser.pressedEvent) startCountdown(now, "countdown started");
      break;

    case RefState::COUNTING:
      if (btnUser.pressedEvent) {                       // [V9] restart from the top
        startCountdown(now, "USER pressed mid-count - countdown RESTARTED from the top");
        break;
      }
      if (Config::MODE == CountdownMode::CYCLES) tickCycles(now);
      else                                       tickMinutes(now);
      break;

    case RefState::DONE:
      if (btnUser.pressedEvent) startCountdown(now, "reset from DONE - countdown restarted");
      break;
  }

  displaySync(now); // push desired state to the display; handles reconnects
}
