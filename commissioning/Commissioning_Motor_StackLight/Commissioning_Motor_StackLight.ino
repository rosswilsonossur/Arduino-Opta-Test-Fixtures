/*  =====================================================================
    Commissioning_Motor_StackLight.ino

    Purpose : Commissioning sketch combining the stack light (Waveshare
              Modbus RTU Relay 4CH) with the CL57RS closed-loop stepper
              drive on the shared RS-485 trunk. Three panel buttons
              (I3/I4/I5) act as momentary Start / Pause / Stop controls:
              each press fires ONE action (not held-down like the
              earlier light-only test), matching real control-panel
              behaviour rather than the wiring-test behaviour of the
              previous sketch.

              START  -> triggers ONE test move on the motor, Green light
              PAUSE  -> quick-stops the motor, Amber light
              STOP   -> quick-stops the motor, Red light

              SCOPE: this proves the button -> Modbus -> motor -> light
              chain works end to end with a single conservative test
              move. It is NOT the full oscillating production cycle -
              that's the next sketch, built on top of this once the
              chain is proven.

    IMPORTANT - READ BEFORE RUNNING:
    - motorSlaveAddr below is a PLACEHOLDER. Confirm the CL57RS's real
      address from its physical DIP switches (SW1-SW5) before running -
      table is below. Do NOT guess. An address collision with the relay
      board (confirmed address 1) will cause the exact silent-timeout
      symptoms we spent a long session debugging on the relay board.
    - TEST_MOVE_PULSES / TEST_MOVE_RPM below default to a small,
      conservative single move. Confirm these are safe for your actual
      mechanical setup (travel limits, coupling, whatever's attached)
      before the first press of START.
    - This drives real motion. Confirm the E-stop / DI1 fail-safe wiring
      established earlier in the project is intact and tested before
      running this on the bench.
    - Built from the CL57RS's official Modbus manual (register map
      matches the "Immediate Trigger" method, section 5.5.4) - NOT
      cross-checked against your ConOps Appendix A permitted-API list
      or the actual v4 reference sketch, since neither was available in
      this session. Verify against both before treating this as
      anything more than a bench commissioning tool.

    CL57RS Slave ID DIP switches (SW1-SW5, off=1 on=0):
      ID 1 (default): on on on on on   | ID 1 (factory): off on on on on
      ID 2: on off on on on            | ID 3: off off on on on
      ID 4: on on off on on            | ID 5: off on off on on
      (see full table in the manual if ID is higher than 5)
      Note: if SW1-SW5 are ALL ON, the ID is set by STEPPERONLINE
      MotionStudio instead - check there if the DIP switches don't
      resolve it.

    Target  : Arduino Opta WiFi (AFX00002)
    Library : ArduinoRS485 + ArduinoModbus (Modbus RTU master)

    Author  : Ross
    Rev     : -  (new sketch, builds on RelayBoard_Commissioning.ino
              Rev G - stack light portion carried over as proven working)
    Status  : Draft for hardware bring-up - NOT production logic

    Usage   : Open Serial Monitor at 115200 baud, line ending = Newline.
              Type HELP and press Enter for the command list. Buttons
              work independently of the serial console.
    ===================================================================== */

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// ============================== USER CONFIGURATION ==============================

// ---- RS-485 bus parameters (shared by both slaves) ----
// Reverted to 9600. The relay board's baud/address changes aren't
// documented as EEPROM-saved (unlike the CL57RS's explicit save step),
// and confirmed by testing: it reverted to 9600 after a power cycle
// that the CL57RS survived cleanly at its own saved settings. Moving
// the CL57RS to 9600 instead - via CL57RS_OneTime_Config.ino - so the
// whole bus matches the relay board's real, reliably-persistent state
// rather than repeating this failure on every power-up.
const long MODBUS_BAUD = 9600;

// ---- Stack light relay board (Waveshare Modbus RTU Relay 4CH) ----
// Address 2 - moved from factory 1 via RelayBoard_SetAddr2.ino so the
// CL57RS (core hardware) owns address 1. Persistence across power
// cycles verified by that tool's STEP2 before this value is trusted.
int relaySlaveAddr = 2;
const int COIL_GREEN = 0;   // Channel 1
const int COIL_AMBER = 1;   // Channel 2
const int COIL_RED   = 2;   // Channel 3
const int COIL_SPARE = 3;   // Channel 4 - unused

// ---- CL57RS motor drive ----
// Address 1 (factory DIP switch position) - core hardware gets the
// zero-configuration address; the optional relay board is the device
// that carries the software-configured one.
int motorSlaveAddr = 1;

// Test move parameters - PR0, relative position, single direction.
// TUNABLE - confirm safe for your actual mechanical setup.
long  TEST_MOVE_PULSES = 10000;   // default microstep is 10000 P/R = 1 revolution
int   TEST_MOVE_RPM    = 60;      // conservative - production profile runs at 180 RPM
int   TEST_MOVE_ACCEL  = 200;     // ms per 1000rpm
int   TEST_MOVE_DECEL  = 200;     // ms per 1000rpm

// CL57RS register addresses (from official Modbus manual, section 5.5.4 / 5.5.3)
const int MOTOR_REG_PR0_BASE   = 0x6200; // Pr9.00: mode, position H/L, vel, acc, dec, pause, trigger (8 regs)
const int MOTOR_REG_TRIGGER    = 0x6002; // Pr8.02: trigger/quick-stop register
const int MOTOR_REG_STATUS     = 0x1003; // Motion status (read-only)
const int MOTOR_REG_ALARM      = 0x2203; // Current alarm code (read-only)

const uint16_t PR0_MODE_RELATIVE = 0x0041; // bit0-3=1 (position mode) | bit6=1 (relative)
const uint16_t PR0_TRIGGER_NOW   = 0x0010; // trigger PR0 immediately as part of the same write
const uint16_t QUICK_STOP_CMD    = 0x0040; // write to MOTOR_REG_TRIGGER to halt immediately

// ---- Button test mode ----
// Opta terminal I3 = A2, I4 = A3, I5 = A4.
// Confirmed sinking-input readings from prior commissioning: idle=HIGH,
// pressed=LOW. VERIFY against RAW print below before trusting this.
const int BUTTON_ACTIVE_STATE = HIGH; // per last confirmed hardware test - re-check if backwards
const unsigned long DEBOUNCE_MS = 30;

struct ButtonChannel {
  int pin;
  int diagLed;
  const char* name;
  int lastReading;
  int stableReading;
  unsigned long lastChangeTime;
};

// index 0=START(I3), 1=PAUSE(I4), 2=STOP(I5)
ButtonChannel buttons[3] = {
  { A2, LED_D0, "START", LOW, LOW, 0 },
  { A3, LED_D1, "PAUSE", LOW, LOW, 0 },
  { A4, LED_D2, "STOP",  LOW, LOW, 0 }
};

const unsigned long RAW_PRINT_INTERVAL_MS = 500;
unsigned long lastRawPrintTime = 0;

// Poll the motor's fault status this often, independent of button presses.
const unsigned long STATUS_POLL_INTERVAL_MS = 500;
unsigned long lastStatusPollTime = 0;
bool motorFaultLatched = false;

// ==================================================================================

enum FixtureState { STATE_STOPPED, STATE_RUNNING, STATE_PAUSED, STATE_FAULT };
FixtureState currentState = STATE_STOPPED;

const char* HELP_TEXT =
  "\r\n"
  "----------------------------------------------------------------\r\n"
  " Motor + stack light commissioning commands\r\n"
  "----------------------------------------------------------------\r\n"
  "  Buttons (momentary, act on press only):\r\n"
  "    I3/A2 START -> trigger one test move, Green\r\n"
  "    I4/A3 PAUSE -> quick-stop, Amber\r\n"
  "    I5/A4 STOP  -> quick-stop, Red\r\n"
  "\r\n"
  "  START   - same as pressing the Start button\r\n"
  "  PAUSE   - same as pressing the Pause button\r\n"
  "  STOP    - same as pressing the Stop button\r\n"
  "  MSTATUS - read motor status register (fault/enable/running bits)\r\n"
  "  MALARM  - read current alarm code register\r\n"
  "  LVER    - read relay board software version\r\n"
  "  LSTATUS - read relay board coil status\r\n"
  "  ALLOFF  - all stack light channels off\r\n"
  "----------------------------------------------------------------\r\n";

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  for (int i = 0; i < 3; i++) {
    pinMode(buttons[i].pin, INPUT);
    pinMode(buttons[i].diagLed, OUTPUT);
    digitalWrite(buttons[i].diagLed, LOW);
  }

  // RS-485 turnaround delay per Modbus RTU framing spec - required on
  // Opta, see RelayBoard_Commissioning.ino Rev G notes.
  float bitDurationUs = 1.0e6f / MODBUS_BAUD;
  float turnaroundDelayUs = bitDurationUs * 9.6f * 3.5f;
  RS485.setDelays(turnaroundDelayUs, turnaroundDelayUs);

  if (!ModbusRTUClient.begin(MODBUS_BAUD, SERIAL_8N1)) {
    Serial.println("FATAL: failed to start Modbus RTU client.");
    while (1) { ; }
  }

  Serial.println(HELP_TEXT);
  Serial.print("relaySlaveAddr = "); Serial.println(relaySlaveAddr);
  Serial.print("motorSlaveAddr = "); Serial.println(motorSlaveAddr);
  if (relaySlaveAddr == motorSlaveAddr) {
    Serial.println("FATAL: relaySlaveAddr and motorSlaveAddr are the same. Fix this before continuing.");
    while (1) { ; }
  }

  setLights(false, false, false);

  // Boot-time verification: confirm both devices answer at their
  // expected addresses before declaring ready. The relay board is
  // treated as an OPTIONAL extra - its absence is reported but is not
  // fatal; the motor is core hardware and its absence is a hard stop
  // on the buttons doing anything useful.
  Serial.println("Boot check: motor at address 1...");
  if (ModbusRTUClient.requestFrom(motorSlaveAddr, HOLDING_REGISTERS, MOTOR_REG_STATUS, 1)) {
    ModbusRTUClient.read();
    Serial.println("  motor answers.");
  } else {
    Serial.println("  MOTOR NOT ANSWERING - check DIP switches at factory ID-1 position, power, wiring.");
  }
  Serial.println("Boot check: relay board at address 2...");
  if (ModbusRTUClient.requestFrom(relaySlaveAddr, HOLDING_REGISTERS, 0x8000, 1)) {
    ModbusRTUClient.read();
    Serial.println("  relay board answers.");
  } else {
    Serial.println("  RELAY BOARD NOT ANSWERING at 2 - stack light disabled.");
    Serial.println("  If it has reverted to address 1 it is COLLIDING with the motor -");
    Serial.println("  disconnect the motor from the trunk and re-run RelayBoard_SetAddr2.ino.");
  }

  Serial.println("Ready.");
}

void loop() {
  rawButtonDiagnostics();
  handleButtons();
  pollMotorStatus();

  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  String cmd = line;
  cmd.toUpperCase();

  if (cmd == "HELP") {
    Serial.println(HELP_TEXT);
  } else if (cmd == "START") {
    doStart();
  } else if (cmd == "PAUSE") {
    doPause();
  } else if (cmd == "STOP") {
    doStop();
  } else if (cmd == "MSTATUS") {
    readMotorStatus(true);
  } else if (cmd == "MALARM") {
    readMotorAlarm();
  } else if (cmd == "LVER") {
    readRelayVersion();
  } else if (cmd == "LSTATUS") {
    readRelayStatus();
  } else if (cmd == "ALLOFF") {
    setLights(false, false, false);
  } else {
    Serial.print("Unrecognised command: ");
    Serial.println(line);
    Serial.println("Type HELP for the command list.");
  }
}

// ------------------------------------------------------------------
// Button handling - momentary, fires once per press (not per hold)
// ------------------------------------------------------------------

void rawButtonDiagnostics() {
  int raw[3];
  for (int i = 0; i < 3; i++) {
    raw[i] = digitalRead(buttons[i].pin);
    digitalWrite(buttons[i].diagLed, raw[i] == BUTTON_ACTIVE_STATE ? HIGH : LOW);
  }
  unsigned long now = millis();
  if (now - lastRawPrintTime >= RAW_PRINT_INTERVAL_MS) {
    lastRawPrintTime = now;
    Serial.print("RAW  I3(A2)="); Serial.print(raw[0]);
    Serial.print("  I4(A3)="); Serial.print(raw[1]);
    Serial.print("  I5(A4)="); Serial.println(raw[2]);
  }
}

void handleButtons() {
  unsigned long now = millis();
  for (int i = 0; i < 3; i++) {
    int reading = digitalRead(buttons[i].pin);
    if (reading != buttons[i].lastReading) {
      buttons[i].lastChangeTime = now;
      buttons[i].lastReading = reading;
    }
    if ((now - buttons[i].lastChangeTime) > DEBOUNCE_MS
        && reading != buttons[i].stableReading) {
      buttons[i].stableReading = reading;
      bool pressed = (reading == BUTTON_ACTIVE_STATE);
      if (pressed) {
        if (i == 0) doStart();
        else if (i == 1) doPause();
        else if (i == 2) doStop();
      }
    }
  }
}

// ------------------------------------------------------------------
// Start / Pause / Stop actions
// ------------------------------------------------------------------

void doStart() {
  if (motorFaultLatched) {
    Serial.println("START ignored - motor fault latched. Clear the fault first (see MALARM / MSTATUS).");
    return;
  }
  Serial.println("START pressed -> triggering test move");
  bool ok = triggerTestMove();
  if (ok) {
    currentState = STATE_RUNNING;
    setLights(true, false, false);
  } else {
    Serial.println("Test move write FAILED - check motorSlaveAddr and wiring. Not changing state.");
  }
}

void doPause() {
  Serial.println("PAUSE pressed -> quick-stop");
  bool ok = quickStopMotor();
  if (ok) {
    currentState = STATE_PAUSED;
    setLights(false, true, false);
  } else {
    Serial.println("Quick-stop write FAILED - check motorSlaveAddr and wiring.");
  }
}

void doStop() {
  Serial.println("STOP pressed -> quick-stop");
  bool ok = quickStopMotor();
  if (ok) {
    currentState = STATE_STOPPED;
    setLights(false, false, true);
  } else {
    Serial.println("Quick-stop write FAILED - check motorSlaveAddr and wiring.");
  }
}

// ------------------------------------------------------------------
// Motor control (CL57RS)
// ------------------------------------------------------------------

// Configures PR0 as a relative-position move AND triggers it immediately,
// as one atomic FC10 block write - the "Immediate Trigger" method from
// the CL57RS manual, section 5.5.4. Single direction, single move -
// this is a commissioning proof, not the oscillating production cycle.
bool triggerTestMove() {
  long pulses = TEST_MOVE_PULSES;
  uint16_t posHigh = (uint16_t)((pulses >> 16) & 0xFFFF);
  uint16_t posLow  = (uint16_t)(pulses & 0xFFFF);

  ModbusRTUClient.beginTransmission(motorSlaveAddr, HOLDING_REGISTERS, MOTOR_REG_PR0_BASE, 8);
  ModbusRTUClient.write(PR0_MODE_RELATIVE);  // Pr9.00 mode
  ModbusRTUClient.write(posHigh);            // Pr9.01 position high
  ModbusRTUClient.write(posLow);             // Pr9.02 position low
  ModbusRTUClient.write(TEST_MOVE_RPM);      // Pr9.03 velocity
  ModbusRTUClient.write(TEST_MOVE_ACCEL);    // Pr9.04 accel
  ModbusRTUClient.write(TEST_MOVE_DECEL);    // Pr9.05 decel
  ModbusRTUClient.write(0);                  // Pr9.06 pause time - none, single move
  ModbusRTUClient.write(PR0_TRIGGER_NOW);    // Pr9.07 -> Pr8.02, triggers immediately
  bool ok = ModbusRTUClient.endTransmission();

  if (!ok) {
    Serial.print("Motor write error: ");
    Serial.println(ModbusRTUClient.lastError());
  }
  return ok;
}

bool quickStopMotor() {
  bool ok = ModbusRTUClient.holdingRegisterWrite(motorSlaveAddr, MOTOR_REG_TRIGGER, QUICK_STOP_CMD);
  if (!ok) {
    Serial.print("Motor write error: ");
    Serial.println(ModbusRTUClient.lastError());
  }
  return ok;
}

// Bit0=Fault, Bit1=Enable, Bit2=Running, Bit4=Command complete,
// Bit5=Path complete, Bit6=Homing complete (per manual, register 0x1003)
void readMotorStatus(bool verbose) {
  if (!ModbusRTUClient.requestFrom(motorSlaveAddr, HOLDING_REGISTERS, MOTOR_REG_STATUS, 1)) {
    Serial.print("MSTATUS read failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  int val = ModbusRTUClient.read();
  bool fault   = val & 0x01;
  bool enabled = val & 0x02;
  bool running = val & 0x04;

  if (verbose) {
    Serial.print("Motor status = 0x"); Serial.print(val, HEX);
    Serial.print("  fault="); Serial.print(fault);
    Serial.print(" enabled="); Serial.print(enabled);
    Serial.print(" running="); Serial.println(running);
  }

  if (fault && !motorFaultLatched) {
    motorFaultLatched = true;
    currentState = STATE_FAULT;
    setLights(false, false, true);
    Serial.println("MOTOR FAULT detected - stack light forced to FAULT/Red. Run MALARM for the code.");
    readMotorAlarm();
  } else if (!fault && motorFaultLatched) {
    motorFaultLatched = false;
    Serial.println("Motor fault bit has cleared. Lights NOT auto-restored - press STOP or START to resume control.");
  }
}

void readMotorAlarm() {
  if (!ModbusRTUClient.requestFrom(motorSlaveAddr, HOLDING_REGISTERS, MOTOR_REG_ALARM, 1)) {
    Serial.print("MALARM read failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  int code = ModbusRTUClient.read();
  Serial.print("Current alarm code = 0x"); Serial.println(code, HEX);
  // See CL57RS manual section 4.4.2 for the full code table -
  // 0x08 (encoder cable error) is already documented in the ConOps.
}

void pollMotorStatus() {
  unsigned long now = millis();
  if (now - lastStatusPollTime >= STATUS_POLL_INTERVAL_MS) {
    lastStatusPollTime = now;
    readMotorStatus(false);
  }
}

// ------------------------------------------------------------------
// Stack light (Waveshare relay board) - single atomic multi-coil write
// ------------------------------------------------------------------

// FC 0x0F (multi-coil write) proved unreliable via ArduinoModbus's
// beginTransmission/write/endTransmission path on this hardware - same
// "Illegal function" result whether coils were packed into one write()
// or sent as separate write() calls, which points at the library
// itself rather than anything in how it was being called. Reverted to
// sequential FC 0x05 single-coil writes - proven reliable from the
// individual RELAY n ON/OFF testing.
//
// Ordered OFF-then-ON so a failure or interruption mid-sequence can
// only ever leave things dimmer than intended, never showing two
// colours at once.
void setLights(bool green, bool amber, bool red) {
  if (!green) writeLightCoil(COIL_GREEN, false, "GREEN");
  if (!amber) writeLightCoil(COIL_AMBER, false, "AMBER");
  if (!red)   writeLightCoil(COIL_RED, false, "RED");
  if (green)  writeLightCoil(COIL_GREEN, true, "GREEN");
  if (amber)  writeLightCoil(COIL_AMBER, true, "AMBER");
  if (red)    writeLightCoil(COIL_RED, true, "RED");
}

bool writeLightCoil(int coil, bool on, const char* label) {
  bool ok = ModbusRTUClient.coilWrite(relaySlaveAddr, coil, on ? 1 : 0);
  if (!ok) {
    Serial.print("Light write FAILED for ");
    Serial.print(label);
    Serial.print(": ");
    Serial.println(ModbusRTUClient.lastError());
  }
  return ok;
}

void readRelayVersion() {
  if (!ModbusRTUClient.requestFrom(relaySlaveAddr, HOLDING_REGISTERS, 0x8000, 1)) {
    Serial.print("LVER read failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  int raw = ModbusRTUClient.read();
  Serial.print("Relay board software version = V");
  Serial.println(raw / 100.0, 2);
}

void readRelayStatus() {
  if (!ModbusRTUClient.requestFrom(relaySlaveAddr, COILS, 0x0000, 4)) {
    Serial.print("LSTATUS read failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  int g = ModbusRTUClient.read();
  int a = ModbusRTUClient.read();
  int r = ModbusRTUClient.read();
  int s = ModbusRTUClient.read();
  Serial.print("GREEN="); Serial.print(g ? "ON" : "OFF");
  Serial.print("  AMBER="); Serial.print(a ? "ON" : "OFF");
  Serial.print("  RED="); Serial.print(r ? "ON" : "OFF");
  Serial.print("  SPARE="); Serial.println(s ? "ON" : "OFF");
}
