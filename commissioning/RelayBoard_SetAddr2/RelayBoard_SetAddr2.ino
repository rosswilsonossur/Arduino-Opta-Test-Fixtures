/*  =====================================================================
    RelayBoard_SetAddr2.ino

    Purpose : ONE-TIME tool to move the Waveshare relay board from
              address 1 to address 2, freeing address 1 for the CL57RS
              motor drive (core hardware, DIP-switched to factory ID 1).
              Includes an explicit persistence test so the address
              change is VERIFIED to survive a power cycle, not assumed.

    Architecture this supports:
      addr 1 = CL57RS motor drive  (core, always present, DIP-switched)
      addr 2 = relay board / stack light (optional extra, software-set)

    IMPORTANT:
    - Run with the CL57RS DISCONNECTED from the RS-485 trunk (lift one
      A/B wire is enough). The address-set command here targets address
      1, and until the relay board moves, BOTH devices would otherwise
      be sitting at 1 - both would process the write and both would try
      to answer, colliding on the bus.
    - Bus runs at 9600 throughout - both devices' factory default.

    Target  : Arduino Opta WiFi (AFX00002)
    Library : ArduinoRS485 + ArduinoModbus

    Author  : Ross
    Rev     : -  (new)

    Usage   : Serial Monitor, 115200 baud, Newline line ending.
              Sequence:
                1. STEP1   (sets address 1 -> 2, verifies at 2)
                2. Power-cycle the relay board
                3. STEP2   (verifies it STILL answers at 2 - the
                            persistence proof)
                4. Reconnect the CL57RS, run STEP3 (verifies both
                   devices answer at their final addresses)
    ===================================================================== */

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

const long MODBUS_BAUD = 9600;
const int  OLD_ADDR = 1;
const int  NEW_ADDR = 2;
const int  MOTOR_ADDR = 1; // CL57RS final address (DIP-switched)

const char* HELP_TEXT =
  "\r\n"
  "----------------------------------------------------------------\r\n"
  " Relay board address migration: 1 -> 2\r\n"
  "----------------------------------------------------------------\r\n"
  "  STEP1  - set relay address 1 -> 2 and verify at 2\r\n"
  "           (CL57RS must be DISCONNECTED from the trunk)\r\n"
  "  STEP2  - after power-cycling the relay board: verify it still\r\n"
  "           answers at 2 (persistence proof)\r\n"
  "  STEP3  - after reconnecting the CL57RS: verify motor at 1 AND\r\n"
  "           relay at 2 both answer on the shared trunk\r\n"
  "  HELP   - this menu\r\n"
  "----------------------------------------------------------------\r\n";

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  float bitDurationUs = 1.0e6f / MODBUS_BAUD;
  float turnaroundDelayUs = bitDurationUs * 9.6f * 3.5f;
  RS485.setDelays(turnaroundDelayUs, turnaroundDelayUs);

  if (!ModbusRTUClient.begin(MODBUS_BAUD, SERIAL_8N1)) {
    Serial.println("FATAL: Modbus client failed to start.");
    while (1) { ; }
  }
  Serial.println(HELP_TEXT);
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  String cmd = line;
  cmd.toUpperCase();

  if (cmd == "HELP")       Serial.println(HELP_TEXT);
  else if (cmd == "STEP1") doStep1();
  else if (cmd == "STEP2") doStep2();
  else if (cmd == "STEP3") doStep3();
  else {
    Serial.print("Unrecognised: "); Serial.println(line);
  }
}

// ------------------------------------------------------------------

bool relayAnswersAt(int addr) {
  // Read the relay board's software version register as a liveness check.
  if (!ModbusRTUClient.requestFrom(addr, HOLDING_REGISTERS, 0x8000, 1)) {
    return false;
  }
  int v = ModbusRTUClient.read();
  Serial.print("  (version register reads V");
  Serial.print(v / 100.0, 2);
  Serial.println(")");
  return true;
}

void doStep1() {
  Serial.println("STEP1: confirm the CL57RS is DISCONNECTED before this runs.");
  Serial.print("Checking relay board answers at old address ");
  Serial.println(OLD_ADDR);
  if (!relayAnswersAt(OLD_ADDR)) {
    Serial.println("No answer at address 1. Is it already at 2? Checking...");
    if (relayAnswersAt(NEW_ADDR)) {
      Serial.println("Already at address 2 - STEP1 not needed. Go to STEP2 after a power cycle.");
    } else {
      Serial.println("No answer at 1 or 2. Check power/wiring before continuing.");
    }
    return;
  }

  Serial.println("Writing address change 1 -> 2 (FC06, register 0x4000)...");
  if (!ModbusRTUClient.holdingRegisterWrite(OLD_ADDR, 0x4000, NEW_ADDR)) {
    Serial.print("Write failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  delay(100);

  Serial.println("Verifying at new address 2...");
  if (relayAnswersAt(NEW_ADDR)) {
    Serial.println("SUCCESS. Now POWER-CYCLE the relay board, then run STEP2.");
  } else {
    Serial.println("No answer at 2 yet - power-cycle the board and run STEP2 anyway; some firmware applies the change on restart.");
  }
}

void doStep2() {
  Serial.println("STEP2: persistence check after power cycle.");
  if (relayAnswersAt(NEW_ADDR)) {
    Serial.println("PERSISTENCE CONFIRMED - address 2 survived the power cycle.");
    Serial.println("Reconnect the CL57RS to the trunk, then run STEP3.");
  } else {
    Serial.println("Not answering at 2 after power cycle. Checking if it fell back to 1...");
    if (relayAnswersAt(OLD_ADDR)) {
      Serial.println("It reverted to address 1 - the address genuinely does not persist on this unit.");
      Serial.println("In that case: re-run STEP1, and we add reassign-on-boot to the main sketch instead.");
    } else {
      Serial.println("No answer at 1 or 2 - check power/wiring before drawing conclusions.");
    }
  }
}

void doStep3() {
  Serial.println("STEP3: full-trunk check with both devices connected.");

  Serial.print("Motor at address "); Serial.print(MOTOR_ADDR); Serial.println("...");
  if (ModbusRTUClient.requestFrom(MOTOR_ADDR, HOLDING_REGISTERS, 0x1003, 1)) {
    int v = ModbusRTUClient.read();
    Serial.print("  answers - status register 0x");
    Serial.println(v, HEX);
  } else {
    Serial.print("  NO ANSWER: ");
    Serial.println(ModbusRTUClient.lastError());
    Serial.println("  Check DIP switches are back at the factory ID-1 position and the drive was power-cycled.");
  }

  Serial.print("Relay at address "); Serial.print(NEW_ADDR); Serial.println("...");
  if (relayAnswersAt(NEW_ADDR)) {
    Serial.println("  answers.");
  } else {
    Serial.println("  NO ANSWER - re-check STEP2 result and wiring.");
  }

  Serial.println("If both answered: migration complete. Update the commissioning sketch and re-upload.");
}
