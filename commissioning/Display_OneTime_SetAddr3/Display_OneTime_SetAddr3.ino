/*  =====================================================================
    Display_OneTime_SetAddr3.ino

    Purpose : ONE-TIME tool to move the YK-LED4006-RS485 6-digit display
              from its factory address 1 to address 3, freeing address 1
              for the CL57RS motor drive (core hardware, DIP-switched to
              factory ID 1). Includes an explicit persistence test so the
              address change is VERIFIED to survive a power cycle, not
              assumed (lesson from the relay-board commissioning: this
              display, unlike the relay board's baud, has an explicit
              save command — 0x001E = 1 — which STEP1 issues and STEP2
              proves).

    Architecture this supports:
      addr 1 = CL57RS motor drive       (core, always present, DIP-switched)
      addr 2 = relay board / stack light (optional extra, software-set)
      addr 3 = YK-LED4006 countdown display (optional extra, software-set)

    IMPORTANT:
    - NEVER connect a factory-fresh display to the fixture trunk: at
      factory address 1 it collides with the CL57RS — both devices
      would process writes to ID 1 and both would try to answer.
      Run this tool on the BENCH with ONLY the display connected.
    - Bus runs at 9600 8N1 throughout — the display's factory default
      (register 0x001C = 1) already matches the trunk (COM-02). This
      tool never touches the baud register.

    Display registers used (see docs/Display_Integration_Plan.md §2):
      0x001D  device address (1..127, 0 = broadcast)
      0x001E  save/restore   (write 1 = save params to NVM)
      0x001F  version/model  (read-only — liveness check)

    Target  : Arduino Opta WiFi (AFX00002)
    Library : ArduinoRS485 + ArduinoModbus

    Author  : Ross
    Rev     : -  (new)

    Usage   : Serial Monitor, 115200 baud, Newline line ending.
              Sequence:
                1. STEP1   (sets address 1 -> 3, saves, verifies at 3)
                2. Power-cycle the display
                3. STEP2   (verifies it STILL answers at 3 — the
                            persistence proof)
                4. Later, on the fixture trunk: STEP3 (verifies motor
                   at 1, relay at 2 AND display at 3 all answer)
              RESCUE : display at an unknown address? Broadcast the
                       address write (display ALONE on the bus only).
    ===================================================================== */

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

const long MODBUS_BAUD = 9600;
const int  OLD_ADDR    = 1;
const int  NEW_ADDR    = 3;
const int  MOTOR_ADDR  = 1; // CL57RS final address (DIP-switched)
const int  RELAY_ADDR  = 2; // relay board final address

const uint16_t REG_DEVICE_ADDR  = 0x001D;
const uint16_t REG_SAVE_RESTORE = 0x001E; // write 1 = save params
const uint16_t REG_VERSION      = 0x001F; // read-only liveness check
const uint16_t SAVE_CMD         = 1;

const char* HELP_TEXT =
  "\r\n"
  "----------------------------------------------------------------\r\n"
  " YK-LED4006 display address migration: 1 -> 3\r\n"
  " Run on the bench with ONLY the display on the RS-485 bus.\r\n"
  "----------------------------------------------------------------\r\n"
  "  STEP1  - set display address 1 -> 3, save to NVM, verify at 3\r\n"
  "  STEP2  - after power-cycling the display: verify it still\r\n"
  "           answers at 3 (persistence proof)\r\n"
  "  STEP3  - later, on the fixture trunk: verify motor at 1,\r\n"
  "           relay at 2 and display at 3 all answer\r\n"
  "  RESCUE - display at an unknown address: broadcast address=3\r\n"
  "           then save+verify (display ALONE on the bus ONLY)\r\n"
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

  if (cmd == "HELP")        Serial.println(HELP_TEXT);
  else if (cmd == "STEP1")  doStep1();
  else if (cmd == "STEP2")  doStep2();
  else if (cmd == "STEP3")  doStep3();
  else if (cmd == "RESCUE") doRescue();
  else {
    Serial.print("Unrecognised: "); Serial.println(line);
  }
}

// ------------------------------------------------------------------

bool displayAnswersAt(int addr) {
  // Read the display's version/model register as a liveness check.
  if (!ModbusRTUClient.requestFrom(addr, HOLDING_REGISTERS, REG_VERSION, 1)) {
    return false;
  }
  long v = ModbusRTUClient.read();
  Serial.print("  (version register reads 0x");
  Serial.print((uint16_t)v, HEX);
  Serial.println(")");
  return true;
}

bool saveParamsAt(int addr) {
  Serial.print("Saving parameters to NVM (0x001E = 1) at address ");
  Serial.print(addr); Serial.println("...");
  if (!ModbusRTUClient.holdingRegisterWrite(addr, REG_SAVE_RESTORE, SAVE_CMD)) {
    Serial.print("  save write failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return false;
  }
  delay(200); // let the device commit before the next transaction
  return true;
}

void doStep1() {
  Serial.println("STEP1: confirm ONLY the display is on the bus before this runs.");
  Serial.print("Checking display answers at old address ");
  Serial.println(OLD_ADDR);
  if (!displayAnswersAt(OLD_ADDR)) {
    Serial.println("No answer at address 1. Is it already at 3? Checking...");
    if (displayAnswersAt(NEW_ADDR)) {
      Serial.println("Already at address 3 - STEP1 not needed. Go to STEP2 after a power cycle.");
    } else {
      Serial.println("No answer at 1 or 3. Check power/wiring/A-B polarity, or try RESCUE.");
    }
    return;
  }

  Serial.println("Writing address change 1 -> 3 (FC06, register 0x001D)...");
  if (!ModbusRTUClient.holdingRegisterWrite(OLD_ADDR, REG_DEVICE_ADDR, NEW_ADDR)) {
    // The device may switch address mid-transaction and reply from the new
    // address (or not at all) - a reported failure here is not conclusive.
    Serial.print("  (write reported: ");
    Serial.print(ModbusRTUClient.lastError());
    Serial.println(" - may be benign if the device switched address immediately)");
  }
  delay(100);

  Serial.println("Probing new address 3...");
  if (displayAnswersAt(NEW_ADDR)) {
    if (saveParamsAt(NEW_ADDR) && displayAnswersAt(NEW_ADDR)) {
      Serial.println("SUCCESS. Now POWER-CYCLE the display, then run STEP2.");
    } else {
      Serial.println("Answered at 3 but the save could not be confirmed - power-cycle and run STEP2 to find out.");
    }
  } else if (displayAnswersAt(OLD_ADDR)) {
    Serial.println("Still answering at 1 - some firmware applies the address only after save/restart.");
    if (saveParamsAt(OLD_ADDR)) {
      Serial.println("Saved at 1. POWER-CYCLE the display, then run STEP2 (it should come back at 3).");
    }
  } else {
    Serial.println("No answer at 1 or 3 - power-cycle the display and run STEP2 anyway; some firmware applies the change on restart.");
  }
}

void doStep2() {
  Serial.println("STEP2: persistence check after power cycle.");
  if (displayAnswersAt(NEW_ADDR)) {
    Serial.println("PERSISTENCE CONFIRMED - address 3 survived the power cycle.");
    Serial.println("The display may now join the fixture trunk. Run STEP3 there.");
  } else {
    Serial.println("Not answering at 3 after power cycle. Checking if it fell back to 1...");
    if (displayAnswersAt(OLD_ADDR)) {
      Serial.println("It reverted to address 1 - the save did not take. Re-run STEP1 and watch the save result.");
    } else {
      Serial.println("No answer at 1 or 3 - check power/wiring before drawing conclusions, or try RESCUE.");
    }
  }
}

void doStep3() {
  Serial.println("STEP3: full-trunk check with all three devices connected.");

  Serial.print("Motor at address "); Serial.print(MOTOR_ADDR); Serial.println("...");
  if (ModbusRTUClient.requestFrom(MOTOR_ADDR, HOLDING_REGISTERS, 0x1003, 1)) {
    long v = ModbusRTUClient.read();
    Serial.print("  answers - status register 0x");
    Serial.println((uint16_t)v, HEX);
  } else {
    Serial.print("  NO ANSWER: ");
    Serial.println(ModbusRTUClient.lastError());
  }

  Serial.print("Relay at address "); Serial.print(RELAY_ADDR); Serial.println("...");
  if (ModbusRTUClient.requestFrom(RELAY_ADDR, HOLDING_REGISTERS, 0x8000, 1)) {
    long v = ModbusRTUClient.read();
    Serial.print("  answers - version V");
    Serial.println(v / 100.0, 2);
  } else {
    Serial.print("  NO ANSWER: ");
    Serial.println(ModbusRTUClient.lastError());
  }

  Serial.print("Display at address "); Serial.print(NEW_ADDR); Serial.println("...");
  if (displayAnswersAt(NEW_ADDR)) {
    Serial.println("  answers.");
  } else {
    Serial.println("  NO ANSWER - re-check STEP2 result and trunk wiring.");
  }

  Serial.println("If all three answered: migration complete. Record the result against V3 in docs/Display_Integration_Plan.md.");
}

void doRescue() {
  Serial.println("RESCUE: broadcast address write. The display must be ALONE on the bus");
  Serial.println("(broadcast reaches EVERY listening device - on the trunk it would re-address the lot).");
  Serial.println("Broadcasting address=3 (FC06 to address 0, register 0x001D)...");
  // Broadcast writes get NO reply by Modbus rule, so the library reports a
  // timeout here - that is EXPECTED and not a failure.
  ModbusRTUClient.holdingRegisterWrite(0, REG_DEVICE_ADDR, NEW_ADDR);
  Serial.println("  (no reply expected for broadcast - ignore the library's timeout)");
  delay(200);

  Serial.println("Probing address 3...");
  if (displayAnswersAt(NEW_ADDR)) {
    if (saveParamsAt(NEW_ADDR) && displayAnswersAt(NEW_ADDR)) {
      Serial.println("RESCUED. Power-cycle the display and run STEP2 for the persistence proof.");
    }
  } else {
    Serial.println("Still no answer at 3. Power-cycle the display and probe again (STEP2),");
    Serial.println("or check power/wiring/A-B polarity - RESCUE cannot fix a wiring fault.");
  }
}
