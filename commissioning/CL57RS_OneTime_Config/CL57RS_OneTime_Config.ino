/*  =====================================================================
    CL57RS_OneTime_Config.ino

    Purpose : ONE-TIME provisioning tool for the CL57RS motor drive.
              The drive was previously bench-tuned at a faster baud rate
              than the relay board's confirmed-working 9600, and now
              that both devices share one RS-485 trunk with one baud
              setting in the master sketch, they need to agree. This
              scans the CL57RS's supported baud rates to find which one
              it's currently listening on, then moves it to the target
              baud (default 9600, matching the relay board) and saves
              that to EEPROM so it survives a power cycle.

              Also supports changing the drive's Modbus address the same
              way (MSETADDR), in case its DIP switches are set to
              "software configurable" (all SW1-SW5 ON) rather than a
              fixed numeric ID.

    IMPORTANT:
    - The CL57RS's baud/address can only be changed over Modbus if its
      physical DIP switches allow it:
        - Baud (SW6/SW7): both OFF permits software configuration.
          Any other position, the switches force the baud in hardware -
          writing the register will have no lasting effect.
        - Address (SW1-SW5): all ON permits software configuration.
          Any other position, the switches force a fixed numeric ID.
      Check these BEFORE running SCAN - if the switches force fixed
      values, change them physically instead; this sketch can't override
      hardware DIP switches.
    - This only scans BAUD at the address already configured in
      motorSlaveAddr below - it does not brute-force unknown addresses.
      Confirm the address via DIP switches (or MotionStudio over RS232)
      first, same as advised for the main commissioning sketch.
    - Run this ONCE, isolated on the bench if possible (though it should
      be safe on the shared trunk since it only ever targts
      motorSlaveAddr). Once confirmed working at the target baud, go
      back to Commissioning_Motor_StackLight.ino for actual bring-up.

    Target  : Arduino Opta WiFi (AFX00002)
    Library : ArduinoRS485 + ArduinoModbus (Modbus RTU master)

    Author  : Ross
    Rev     : -  (new)
    Status  : Draft - one-time provisioning tool, not for ongoing use

    Usage   : Open Serial Monitor at 115200 baud, line ending = Newline.
              Sequence: SCAN -> SETBAUD -> (power-cycle the CL57RS) -> CONFIRM
              Optionally: MSETADDR <n> if the address also needs setting.
    ===================================================================== */

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// ============================== USER CONFIGURATION ==============================

// Confirmed at address 1 (DIP switches at factory ID-1 position).
int motorSlaveAddr = 1;

// Where we want the drive to end up - matching the relay board's
// confirmed-working 9600. Change here if you'd rather standardize the
// whole bus on a different baud instead (would also mean revisiting the
// relay board's baud via its own SETADDR-style command).
const long   TARGET_BAUD      = 9600;
const uint16_t TARGET_BAUD_ENUM = 2; // Pr5.22 enum: 0=2400 1=4800 2=9600 3=19200 4=38400 5=57600 6=115200

// Candidate bauds to scan, in order. The four DIP-switch-selectable
// values first (most likely), then the extra values the register
// itself supports.
const long CANDIDATE_BAUDS[] = { 115200, 38400, 19200, 9600, 57600, 4800, 2400 };
const int  NUM_CANDIDATE_BAUDS = 7;

// CL57RS register addresses (from official Modbus manual)
const int MOTOR_REG_BAUD        = 0x01BD; // Pr5.22 - RS485 baud rate
const int MOTOR_REG_ADDR        = 0x01BF; // Pr5.23 - RS485 ID
const int MOTOR_REG_SAVE_EEPROM = 0x1801; // Control word
const uint16_t EEPROM_SAVE_CMD  = 0x2211; // Write here to persist changes

// ==================================================================================

long discoveredBaud = -1;

const char* HELP_TEXT =
  "\r\n"
  "----------------------------------------------------------------\r\n"
  " CL57RS one-time config commands\r\n"
  "----------------------------------------------------------------\r\n"
  "  HELP           - show this menu\r\n"
  "  SCAN           - try each candidate baud, report which one\r\n"
  "                   motorSlaveAddr currently responds on\r\n"
  "  SETBAUD        - move the drive to TARGET_BAUD and save to EEPROM\r\n"
  "                   (run SCAN first)\r\n"
  "  CONFIRM        - reconnect at TARGET_BAUD and verify\r\n"
  "  MSETADDR <n>   - change the drive's Modbus address to n and save\r\n"
  "                   to EEPROM (uses whichever baud SCAN found)\r\n"
  "----------------------------------------------------------------\r\n";

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println(HELP_TEXT);
  Serial.print("motorSlaveAddr = "); Serial.println(motorSlaveAddr);
  Serial.print("target baud    = "); Serial.println(TARGET_BAUD);
  Serial.println("Ready. Run SCAN first.");
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  String cmd = line;
  cmd.toUpperCase();

  if (cmd == "HELP") {
    Serial.println(HELP_TEXT);
  } else if (cmd == "SCAN") {
    cmdScan();
  } else if (cmd == "SETBAUD") {
    cmdSetBaud();
  } else if (cmd == "CONFIRM") {
    cmdConfirm();
  } else if (cmd.startsWith("MSETADDR")) {
    int spaceIdx = cmd.lastIndexOf(' ');
    int newAddr = spaceIdx < 0 ? -1 : cmd.substring(spaceIdx + 1).toInt();
    cmdSetAddr(newAddr);
  } else {
    Serial.print("Unrecognised command: ");
    Serial.println(line);
    Serial.println("Type HELP for the command list.");
  }
}

// ------------------------------------------------------------------

void beginModbusAtBaud(long baud) {
  float bitDurationUs = 1.0e6f / baud;
  float turnaroundDelayUs = bitDurationUs * 9.6f * 3.5f;
  RS485.setDelays(turnaroundDelayUs, turnaroundDelayUs);
  ModbusRTUClient.begin(baud, SERIAL_8N1);
  delay(50); // let the port settle after reconfiguring
}

void cmdScan() {
  Serial.print("Scanning candidate baud rates for address ");
  Serial.print(motorSlaveAddr);
  Serial.println(" ...");
  discoveredBaud = -1;

  for (int i = 0; i < NUM_CANDIDATE_BAUDS; i++) {
    long baud = CANDIDATE_BAUDS[i];
    beginModbusAtBaud(baud);
    Serial.print("  ");
    Serial.print(baud);
    Serial.print(" baud... ");
    if (ModbusRTUClient.requestFrom(motorSlaveAddr, HOLDING_REGISTERS, MOTOR_REG_BAUD, 1)) {
      int val = ModbusRTUClient.read();
      Serial.print("RESPONSE - Pr5.22 register reads ");
      Serial.println(val);
      discoveredBaud = baud;
      break;
    } else {
      Serial.println("no response");
    }
  }

  if (discoveredBaud < 0) {
    Serial.println("No response at any candidate baud.");
    Serial.println("Check: motorSlaveAddr correct? Wiring on the correct RS485 port on CN4? A/B orientation?");
  } else {
    Serial.print("Motor currently responds at ");
    Serial.print(discoveredBaud);
    Serial.println(" baud.");
    if (discoveredBaud == TARGET_BAUD) {
      Serial.println("This already matches TARGET_BAUD - nothing to change.");
    } else {
      Serial.println("Run SETBAUD to move it to TARGET_BAUD.");
    }
  }
}

void cmdSetBaud() {
  if (discoveredBaud < 0) {
    Serial.println("Run SCAN first to find the motor's current baud.");
    return;
  }
  beginModbusAtBaud(discoveredBaud);

  Serial.print("Writing Pr5.22 = ");
  Serial.print(TARGET_BAUD_ENUM);
  Serial.print(" (");
  Serial.print(TARGET_BAUD);
  Serial.println(" baud)...");
  if (!ModbusRTUClient.holdingRegisterWrite(motorSlaveAddr, MOTOR_REG_BAUD, TARGET_BAUD_ENUM)) {
    Serial.print("Write failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }

  delay(50);
  Serial.println("Saving to EEPROM (required, or this reverts on power-off)...");
  if (!ModbusRTUClient.holdingRegisterWrite(motorSlaveAddr, MOTOR_REG_SAVE_EEPROM, EEPROM_SAVE_CMD)) {
    Serial.print("EEPROM save failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }

  Serial.println("Done. POWER-CYCLE the CL57RS now, then run CONFIRM.");
}

void cmdConfirm() {
  beginModbusAtBaud(TARGET_BAUD);
  Serial.print("Checking address ");
  Serial.print(motorSlaveAddr);
  Serial.print(" at ");
  Serial.print(TARGET_BAUD);
  Serial.println(" baud...");

  if (ModbusRTUClient.requestFrom(motorSlaveAddr, HOLDING_REGISTERS, MOTOR_REG_BAUD, 1)) {
    int val = ModbusRTUClient.read();
    Serial.print("SUCCESS - Pr5.22 reads ");
    Serial.println(val);
    if (val == TARGET_BAUD_ENUM) {
      Serial.println("Baud change confirmed and persisted. Safe to move on to the commissioning sketch.");
    } else {
      Serial.println("Responding, but register value is unexpected - worth a closer look before continuing.");
    }
  } else {
    Serial.print("No response at target baud: ");
    Serial.println(ModbusRTUClient.lastError());
    Serial.println("Did you power-cycle the drive after SETBAUD? Also re-check SW6/SW7 aren't forcing a fixed baud.");
  }
}

void cmdSetAddr(int newAddr) {
  if (newAddr < 1 || newAddr > 31) {
    Serial.println("Usage: MSETADDR <1-31> (CLRS series supports up to 31 sub-devices)");
    return;
  }
  if (discoveredBaud < 0) {
    Serial.println("Run SCAN first so this knows which baud to talk on.");
    return;
  }
  if (newAddr == /* relay board address, for reference */ 1) {
    Serial.println("WARNING: address 1 is the relay board's confirmed address. Choose a different one.");
    return;
  }

  beginModbusAtBaud(discoveredBaud);
  Serial.print("Writing Pr5.23 = ");
  Serial.print(newAddr);
  Serial.println(" ...");
  if (!ModbusRTUClient.holdingRegisterWrite(motorSlaveAddr, MOTOR_REG_ADDR, newAddr)) {
    Serial.print("Write failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }

  delay(50);
  Serial.println("Saving to EEPROM...");
  if (!ModbusRTUClient.holdingRegisterWrite(motorSlaveAddr, MOTOR_REG_SAVE_EEPROM, EEPROM_SAVE_CMD)) {
    Serial.print("EEPROM save failed: ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }

  Serial.print("Done. POWER-CYCLE the CL57RS, then update motorSlaveAddr to ");
  Serial.print(newAddr);
  Serial.println(" in both this sketch and the commissioning sketch before continuing.");
}
