/*  =====================================================================
    RelayBoard_OneTime_BaudConfig.ino

    Purpose : ONE-TIME provisioning tool to move the Waveshare relay
              board's baud rate from its confirmed-working 9600 up to
              115200, to match the CL57RS motor drive (already
              confirmed working at 115200) rather than moving the motor
              down to 9600.

              Unlike the CL57RS, the relay board's own docs don't
              mention a separate "must save to EEPROM or it reverts on
              power-off" step for baud/address changes - the change is
              expected to take effect directly. This tool verifies that
              assumption by attempting an immediate read-back at the new
              baud, and falls back to asking for a power-cycle if that
              doesn't work.

    IMPORTANT:
    - Run this ONCE, then update MODBUS_BAUD to 115200 in both
      RelayBoard_Commissioning.ino and Commissioning_Motor_StackLight.ino
      before using either of those again - they still default to 9600.
    - The CL57RS_OneTime_Config.ino sketch built earlier is no longer
      needed for baud (we're keeping the motor as-is) - its MSETADDR
      command is still valid if the motor's address ever needs changing.

    Target  : Arduino Opta WiFi (AFX00002)
    Library : ArduinoRS485 + ArduinoModbus (Modbus RTU master)

    Author  : Ross
    Rev     : -  (new)
    Status  : Draft - one-time provisioning tool, not for ongoing use

    Usage   : Open Serial Monitor at 115200 baud, line ending = Newline.
              Sequence: SETBAUD -> (if it reports success, done)
                        If SETBAUD's immediate verification fails,
                        power-cycle the relay board and run CONFIRM.
    ===================================================================== */

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// ============================== USER CONFIGURATION ==============================

int relaySlaveAddr = 1; // confirmed

const long     CURRENT_BAUD = 9600;
const long     TARGET_BAUD  = 115200;
const uint16_t TARGET_BAUD_CODE = 0x05; // per relay board register map: 0x05 = 115200
const uint16_t PARITY_NONE  = 0x00;     // matches the 8N1 framing used throughout

const int RELAY_REG_UART = 0x2000; // write (parity<<8 | baudCode) here

// ==================================================================================

const char* HELP_TEXT =
  "\r\n"
  "----------------------------------------------------------------\r\n"
  " Relay board one-time baud config\r\n"
  "----------------------------------------------------------------\r\n"
  "  HELP     - show this menu\r\n"
  "  SETBAUD  - write the new baud at 9600, then try to verify\r\n"
  "             immediately at 115200\r\n"
  "  CONFIRM  - reconnect at 115200 and verify (use after a power-cycle\r\n"
  "             if SETBAUD's immediate check didn't succeed)\r\n"
  "----------------------------------------------------------------\r\n";

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println(HELP_TEXT);
  Serial.print("relaySlaveAddr = "); Serial.println(relaySlaveAddr);
  Serial.print("current baud   = "); Serial.println(CURRENT_BAUD);
  Serial.print("target baud    = "); Serial.println(TARGET_BAUD);
  Serial.println("Ready. Run SETBAUD.");
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
  } else if (cmd == "SETBAUD") {
    cmdSetBaud();
  } else if (cmd == "CONFIRM") {
    cmdConfirm();
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
  delay(50);
}

void cmdSetBaud() {
  beginModbusAtBaud(CURRENT_BAUD);

  uint16_t writeValue = (PARITY_NONE << 8) | TARGET_BAUD_CODE;
  Serial.print("Writing register 0x2000 = 0x");
  Serial.print(writeValue, HEX);
  Serial.print(" (no parity, ");
  Serial.print(TARGET_BAUD);
  Serial.println(" baud) at current baud 9600...");

  if (!ModbusRTUClient.holdingRegisterWrite(relaySlaveAddr, RELAY_REG_UART, writeValue)) {
    Serial.print("Write failed: ");
    Serial.println(ModbusRTUClient.lastError());
    Serial.println("Board may already be unreachable at 9600 - check nothing changed since last confirmed working.");
    return;
  }
  Serial.println("Write acknowledged.");

  Serial.println("Attempting immediate verification at 115200 (no power-cycle)...");
  delay(100);
  beginModbusAtBaud(TARGET_BAUD);
  if (verifyAtCurrentBaud()) {
    Serial.println("SUCCESS - relay board is now answering at 115200 without a power-cycle.");
  } else {
    Serial.println("No response yet at 115200. Power-cycle the relay board, then run CONFIRM.");
  }
}

void cmdConfirm() {
  beginModbusAtBaud(TARGET_BAUD);
  Serial.println("Checking relay board at 115200...");
  if (verifyAtCurrentBaud()) {
    Serial.println("SUCCESS - baud change confirmed and persisted.");
    Serial.println("Update MODBUS_BAUD to 115200 in the commissioning sketches before continuing.");
  } else {
    Serial.println("Still no response at 115200.");
    Serial.println("Did the write in SETBAUD actually succeed? Try SETBAUD again, or check A/B wiring hasn't shifted.");
  }
}

bool verifyAtCurrentBaud() {
  if (!ModbusRTUClient.requestFrom(relaySlaveAddr, HOLDING_REGISTERS, 0x8000, 1)) {
    return false;
  }
  int val = ModbusRTUClient.read();
  Serial.print("Software version register reads ");
  Serial.print(val);
  Serial.print(" (V");
  Serial.print(val / 100.0, 2);
  Serial.println(")");
  return true;
}
