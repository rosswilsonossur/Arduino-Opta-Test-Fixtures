/*  =====================================================================
    RelayBoard_Commissioning.ino

    Purpose : Standalone hardware bring-up / test tool for the Waveshare
              "Modbus RTU Relay 4CH" board that drives the 3-light
              stack light (Green/Amber/Red) on the GTF test fixture.

              Live test mode: three panel buttons wired to the Opta's
              digital inputs directly actuate the matching stack light
              colour for as long as they're held - lets you confirm
              button wiring and relay/light wiring together, by hand,
              before any state-machine logic is involved.

              Run this BEFORE the relay board is added to the shared
              RS-485 trunk with the CL57RS drive. Wire the Opta and the
              relay board point-to-point (A-A, B-B) so there is only one
              slave on the bus - this avoids any address collision while
              you confirm the board's identity and set its final address.

    Target  : Arduino Opta WiFi (AFX00002)
    Library : ArduinoRS485 + ArduinoModbus (Modbus RTU master)

    Author  : Ross
    Rev     : G - added RS485.setDelays() before ModbusRTUClient.begin(),
              matching every official Arduino Opta Modbus RTU example.
              Missing this is a documented cause of persistent
              "Connection timed out" on Opta even when wiring, common
              ground, and addressing are all correct. Also made the
              8N1 serial config explicit in begin() to match the relay
              board's factory default format.
    Status  : Draft for hardware bring-up

    Usage   : Open Serial Monitor at 115200 baud, line ending = Newline.
              Type HELP and press Enter for the command list.
              Button test mode runs continuously in the background -
              no serial command needed, just press the buttons.
    ===================================================================== */

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// ============================== USER CONFIGURATION ==============================
// Everything you're likely to need to change lives here.

// Modbus RTU serial parameters - must match the relay board's configured
// comms format. Board ships as 9600, N, 8, 1 from the factory - and per
// direct testing, reverts to this after any power loss, since its
// baud-change command isn't EEPROM-saved the way the CL57RS's is. Left
// at the reliable factory default rather than fighting that.
const long MODBUS_BAUD = 9600;

// Slave address to talk to for all "known address" commands below.
// CHANGE THIS once you've set the board's real address with SETADDR.
// NOTE: confirm this does not collide with the CL57RS's configured
// Modbus address before this board ever joins the shared trunk.
int relaySlaveAddr = 1;

// Coil addresses are fixed by the Waveshare register map - do not change.
const int COIL_GREEN = 0;   // Channel 1
const int COIL_AMBER = 1;   // Channel 2
const int COIL_RED   = 2;   // Channel 3
const int COIL_SPARE = 3;   // Channel 4 - unused for now

// Timing for the WALK sequential-channel test.
const unsigned long WALK_STEP_MS = 400;

// ---- Button test mode ----
// Opta terminal I3 = A2, I4 = A3, I5 = A4 (per Arduino's Opta pin map).
// All three buttons are normally open.
const int BUTTON_ACTIVE_STATE = HIGH;

const unsigned long DEBOUNCE_MS = 30;

struct ButtonChannel {
  int pin;
  int coil;
  int diagLed;
  const char* name;
  int lastReading;
  int stableReading;
  unsigned long lastChangeTime;
};

// LED_D0/D1/D2 are Opta's onboard status LEDs (normally paired with its
// own built-in relay outputs RELAY1-4, which this project doesn't use -
// see ConOps). Borrowed here purely as a raw hardware-level debug
// indicator: each LED mirrors its button pin's electrical state
// DIRECTLY, with no debounce and no dependency on Modbus or serial.
// If a button press doesn't light its LED, the fault is in the wiring
// or the input pin, not in this sketch's logic.
ButtonChannel buttons[3] = {
  { A2, COIL_GREEN, LED_D0, "GREEN", LOW, LOW, 0 },   // I3
  { A3, COIL_AMBER, LED_D1, "AMBER", LOW, LOW, 0 },   // I4
  { A4, COIL_RED,   LED_D2, "RED",   LOW, LOW, 0 }    // I5
};

const unsigned long RAW_PRINT_INTERVAL_MS = 500;
unsigned long lastRawPrintTime = 0;

// ==================================================================================

const char* HELP_TEXT =
  "\r\n"
  "----------------------------------------------------------------\r\n"
  " Relay board commissioning commands (type, then press Enter)\r\n"
  "----------------------------------------------------------------\r\n"
  "  Live button test is always running: I3/A2->GREEN, I4/A3->AMBER,\r\n"
  "  I5/A4->RED. Hold a button, matching light turns on; release, off.\r\n"
  "\r\n"
  "  Raw diagnostics (always on, no setup needed):\r\n"
  "  - Onboard LED_D0/D1/D2 mirror the raw I3/I4/I5 pin state directly.\r\n"
  "    Press a button, its LED should light immediately - no serial\r\n"
  "    monitor needed. If it doesn't, the fault is upstream of this\r\n"
  "    sketch (wiring, terminal, or the input itself).\r\n"
  "  - A 'RAW I3(A2)=.. I4(A3)=.. I5(A4)=..' line prints every 500ms\r\n"
  "    regardless of button state, so you can confirm serial is alive\r\n"
  "    and watch the raw 0/1 values change as you press.\r\n"
  "----------------------------------------------------------------\r\n"
  "  HELP             - show this menu\r\n"
  "  VER              - read software version register (0x8000)\r\n"
  "  ADDR             - read device address using relaySlaveAddr\r\n"
  "  IDENTIFY         - read device address via broadcast\r\n"
  "                     (ONLY safe with a single device on the bus)\r\n"
  "  SETADDR <n>      - set device address to n (1-255)\r\n"
  "                     (ONLY safe with a single device on the bus)\r\n"
  "  STATUS           - read back all 4 coil states\r\n"
  "  GREEN ON|OFF     - drive channel 1 directly\r\n"
  "  AMBER ON|OFF     - drive channel 2 directly\r\n"
  "  RED ON|OFF       - drive channel 3 directly\r\n"
  "  SPARE ON|OFF     - drive channel 4 directly\r\n"
  "  ALLOFF           - all relays off in one atomic write (=COMPLETE)\r\n"
  "  RUN              - drive RUNNING pattern  (green solid)\r\n"
  "  PAUSE            - drive PAUSED pattern   (amber solid)\r\n"
  "  FAULT            - drive FAULT/E-STOP pattern (red solid)\r\n"
  "  WALK             - sequential channel walking test 1->2->3->4\r\n"
  "----------------------------------------------------------------\r\n";

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  for (int i = 0; i < 3; i++) {
    pinMode(buttons[i].pin, INPUT);
    pinMode(buttons[i].diagLed, OUTPUT);
    digitalWrite(buttons[i].diagLed, LOW);
  }

  // RS-485 turnaround delays per Modbus RTU framing spec (3.5 character
  // times), computed from the configured baud rate. Every official
  // Arduino Opta Modbus RTU example sets this explicitly before
  // ModbusRTUClient.begin() - omitting it is a well-documented cause of
  // "Connection timed out" on Opta specifically, independent of wiring,
  // grounding, or addressing.
  float bitDurationUs = 1.0e6f / MODBUS_BAUD;
  float turnaroundDelayUs = bitDurationUs * 9.6f * 3.5f;
  RS485.setDelays(turnaroundDelayUs, turnaroundDelayUs);
  Serial.print("RS485 turnaround delay set to ");
  Serial.print(turnaroundDelayUs);
  Serial.println(" us");

  if (!ModbusRTUClient.begin(MODBUS_BAUD, SERIAL_8N1)) {
    Serial.println("FATAL: failed to start Modbus RTU client.");
    while (1) { ; }
  }

  Serial.println(HELP_TEXT);
  Serial.print("Current relaySlaveAddr = ");
  Serial.println(relaySlaveAddr);
  Serial.println("Ready. Button test mode is live.");
}

void loop() {
  rawButtonDiagnostics();
  handleButtons();

  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String cmd = line;
  cmd.toUpperCase();

  if (cmd == "HELP") {
    Serial.println(HELP_TEXT);

  } else if (cmd == "VER") {
    readSoftwareVersion();

  } else if (cmd == "ADDR") {
    readDeviceAddress(relaySlaveAddr);

  } else if (cmd == "IDENTIFY") {
    Serial.println("Broadcasting address query - make sure only ONE device is on the bus.");
    readDeviceAddress(0);

  } else if (cmd.startsWith("SETADDR")) {
    int newAddr = parseTrailingInt(cmd);
    if (newAddr < 1 || newAddr > 255) {
      Serial.println("Usage: SETADDR <1-255>. Make sure only ONE device is on the bus.");
    } else {
      setDeviceAddress(newAddr);
    }

  } else if (cmd == "STATUS") {
    printStatus();

  } else if (cmd == "GREEN ON") {
    writeCoil(COIL_GREEN, true, "GREEN");
  } else if (cmd == "GREEN OFF") {
    writeCoil(COIL_GREEN, false, "GREEN");
  } else if (cmd == "AMBER ON") {
    writeCoil(COIL_AMBER, true, "AMBER");
  } else if (cmd == "AMBER OFF") {
    writeCoil(COIL_AMBER, false, "AMBER");
  } else if (cmd == "RED ON") {
    writeCoil(COIL_RED, true, "RED");
  } else if (cmd == "RED OFF") {
    writeCoil(COIL_RED, false, "RED");
  } else if (cmd == "SPARE ON") {
    writeCoil(COIL_SPARE, true, "SPARE");
  } else if (cmd == "SPARE OFF") {
    writeCoil(COIL_SPARE, false, "SPARE");

  } else if (cmd == "ALLOFF") {
    drivePattern(false, false, false, "ALLOFF / COMPLETE");

  } else if (cmd == "RUN") {
    drivePattern(true, false, false, "RUNNING");

  } else if (cmd == "PAUSE") {
    drivePattern(false, true, false, "PAUSED");

  } else if (cmd == "FAULT") {
    drivePattern(false, false, true, "FAULT / E-STOP");

  } else if (cmd == "WALK") {
    walkTest();

  } else {
    Serial.print("Unrecognised command: ");
    Serial.println(line);
    Serial.println("Type HELP for the command list.");
  }
}

// ------------------------------------------------------------------
// Raw hardware diagnostics - no debounce, no Modbus, no dependency on
// anything else in the sketch. Ground truth for what the pins are
// actually seeing.
// ------------------------------------------------------------------

void rawButtonDiagnostics() {
  int raw[3];
  for (int i = 0; i < 3; i++) {
    raw[i] = digitalRead(buttons[i].pin);
    // LED lit = pressed (raw reading matches the confirmed active state),
    // not a raw mirror of the pin - now that polarity is known this is
    // the more intuitive sense to watch.
    digitalWrite(buttons[i].diagLed, raw[i] == BUTTON_ACTIVE_STATE ? HIGH : LOW);
  }

  unsigned long now = millis();
  if (now - lastRawPrintTime >= RAW_PRINT_INTERVAL_MS) {
    lastRawPrintTime = now;
    Serial.print("RAW  I3(A2)=");
    Serial.print(raw[0]);
    Serial.print("  I4(A3)=");
    Serial.print(raw[1]);
    Serial.print("  I5(A4)=");
    Serial.println(raw[2]);
  }
}

// ------------------------------------------------------------------
// Live button test mode
// ------------------------------------------------------------------

// Non-blocking per-button debounce. Each button independently drives
// its own channel - all three can be held at once if you want to
// confirm the board handles simultaneous writes cleanly.
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

      writeCoil(buttons[i].coil, pressed, buttons[i].name);
      Serial.print(buttons[i].name);
      Serial.println(pressed ? " button PRESSED -> light ON" : " button RELEASED -> light OFF");
    }
  }
}

// ------------------------------------------------------------------
// Helpers
// ------------------------------------------------------------------

int parseTrailingInt(const String& cmd) {
  int spaceIdx = cmd.lastIndexOf(' ');
  if (spaceIdx < 0) return -1;
  return cmd.substring(spaceIdx + 1).toInt();
}

void readSoftwareVersion() {
  // Holding register 0x8000, 1 register. Value/100 = version, e.g. 300 = V3.00
  if (!ModbusRTUClient.requestFrom(relaySlaveAddr, HOLDING_REGISTERS, 0x8000, 1)) {
    Serial.print("VER read failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  int raw = ModbusRTUClient.read();
  Serial.print("Software version register = ");
  Serial.print(raw);
  Serial.print("  ->  V");
  Serial.println(raw / 100.0, 2);
}

void readDeviceAddress(int queryAddr) {
  // Holding register 0x4000, 1 register.
  if (!ModbusRTUClient.requestFrom(queryAddr, HOLDING_REGISTERS, 0x4000, 1)) {
    Serial.print("Address read failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  int addr = ModbusRTUClient.read();
  Serial.print("Board reports configured address = ");
  Serial.println(addr);
}

void setDeviceAddress(int newAddr) {
  // FC06, register 0x4000. Broadcast (address 0) so this works even if
  // you don't know the board's current address - only safe with one
  // device on the bus.
  if (!ModbusRTUClient.holdingRegisterWrite(0, 0x4000, newAddr)) {
    Serial.print("SETADDR failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  Serial.print("Address set command sent for new address ");
  Serial.println(newAddr);
  Serial.println("Update relaySlaveAddr in this sketch to match, then re-upload.");
}

void printStatus() {
  if (!ModbusRTUClient.requestFrom(relaySlaveAddr, COILS, 0x0000, 4)) {
    Serial.print("STATUS read failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  int g = ModbusRTUClient.read();
  int a = ModbusRTUClient.read();
  int r = ModbusRTUClient.read();
  int s = ModbusRTUClient.read();

  Serial.println("---- Relay status ----");
  Serial.print("GREEN (ch1) : "); Serial.println(g ? "ON" : "OFF");
  Serial.print("AMBER (ch2) : "); Serial.println(a ? "ON" : "OFF");
  Serial.print("RED   (ch3) : "); Serial.println(r ? "ON" : "OFF");
  Serial.print("SPARE (ch4) : "); Serial.println(s ? "ON" : "OFF");
  Serial.println("-----------------------");
}

void writeCoil(int coil, bool on, const char* label) {
  // ModbusRTUClient.coilWrite() takes a plain 0/1 value and handles the
  // FC05 wire encoding (0xFF00/0x0000) internally.
  bool ok = ModbusRTUClient.coilWrite(relaySlaveAddr, coil, on ? 1 : 0);
  if (!ok) {
    Serial.print("Write FAILED  addr=");
    Serial.print(relaySlaveAddr);
    Serial.print("  ");
    Serial.print(label);
    Serial.print(" -> ");
    Serial.print(on ? "ON" : "OFF");
    Serial.print("   error: ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.print("Write OK      addr=");
    Serial.print(relaySlaveAddr);
    Serial.print("  ");
    Serial.print(label);
    Serial.print(" -> ");
    Serial.println(on ? "ON" : "OFF");
  }
}

// FC 0x0F (multi-coil write) proved unreliable via ArduinoModbus's
// beginTransmission/write/endTransmission path on this hardware - same
// "Illegal function" result whether coils were packed into one write()
// or sent as separate write() calls, pointing at the library itself
// rather than anything about how it was called. Reverted to sequential
// FC 0x05 single-coil writes - proven reliable from individual channel
// ON/OFF testing.
//
// Ordered OFF-then-ON so a failure or interruption mid-sequence can
// only ever leave things dimmer than intended, never showing two
// colours at once.
void drivePattern(bool green, bool amber, bool red, const char* label) {
  bool ok = true;
  if (!green) ok &= writeCoil(COIL_GREEN, false, "GREEN");
  if (!amber) ok &= writeCoil(COIL_AMBER, false, "AMBER");
  if (!red)   ok &= writeCoil(COIL_RED, false, "RED");
  if (green)  ok &= writeCoil(COIL_GREEN, true, "GREEN");
  if (amber)  ok &= writeCoil(COIL_AMBER, true, "AMBER");
  if (red)    ok &= writeCoil(COIL_RED, true, "RED");

  if (!ok) {
    Serial.print("Pattern write had at least one failure for ");
    Serial.println(label);
    return;
  }
  Serial.print("Pattern applied: ");
  Serial.println(label);
}

// Sequential 1->2->3->4 on, then 1->2->3->4 off, mirroring Waveshare's
// own demo behaviour. Useful as a first-power-on sanity check to
// confirm each physical channel switches before you trust the colour
// mapping - watch the board's own relay LEDs, or the light itself if
// it's already wired.
void walkTest() {
  Serial.println("Walking test: watch the relay board's channel LEDs.");
  int coils[4] = {COIL_GREEN, COIL_AMBER, COIL_RED, COIL_SPARE};
  const char* names[4] = {"GREEN", "AMBER", "RED", "SPARE"};

  for (int i = 0; i < 4; i++) {
    writeCoil(coils[i], true, names[i]);
    delay(WALK_STEP_MS);
  }
  for (int i = 0; i < 4; i++) {
    writeCoil(coils[i], false, names[i]);
    delay(WALK_STEP_MS);
  }
  Serial.println("Walking test complete.");
}
