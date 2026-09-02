# Generic Test Fixture — Concept of Operations

**Document ID:** GTF-CONOPS-001 Rev L
**Purpose:** This document defines the hardware architecture, system states, and software requirements for a generic motor-driven test fixture. It is the single source of context to be provided to any AI code-generation tool. The AI shall read this document in full, then follow the workflow in §10 before generating any code.

---

## 1. System Overview

The fixture cycles a unit under test (UUT) by driving a stepper motor through a repeated motion profile (nominally: oscillate back and forth for a configured number of cycles, speed, and dwell). The controller is an Arduino Opta WiFi acting as a Modbus RTU master over RS-485 to a closed-loop stepper driver and, optionally, a Modbus relay module driving a 3-colour stack light. Operator interaction is via panel Start / Pause / Stop push buttons, an E-stop, a pilot light, and (where fitted) the stack light indicating system state at a distance.

The fixture is *generic*: the motion profile and I/O options (guard switches, counters, pause/restart buttons) are configured per test by regenerating the control program with an AI tool, using this document plus the interrogation workflow in §10.

## 2. Hardware

| Item | Part | Role |
|---|---|---|
| Controller | Arduino Opta WiFi | Modbus RTU master, all fixture logic |
| Motor driver | StepperOnline CL57RS (CLRS series) | Closed-loop stepper drive, Modbus RTU slave |
| E-stop button | 2-channel, NC contacts | Safety stop (see §3) |
| Pilot light | 24 V lamp | State indication via Opta relay 1 |
| Start push button (internal) | Opta BTN_USER | Programme start |
| Panel Start / Pause / Stop buttons | External NO buttons → I3 / I4 / I5 | Green/Amber/Red panel buttons; Start mirrors BTN_USER (§7.2, §7.3, §7.7) |
| Optional: stack light | 3-light 24 VDC LED tower (Green/Amber/Red) | State indication at a distance (§7.1); driven by relay module below; 24 V+ feed fused |
| Optional: relay module | Waveshare Modbus RTU Relay 4CH | Modbus RTU slave ID 2; channels 1/2/3 → Green/Amber/Red, channel 4 spare (§6.5) |
| Optional: restart button | External button → spare input | §7.4 |
| Optional: guard switch | Limit switch → I6 | §7.5 |
| Optional: counter / travel-limit switch | Limit switch → I7 | §7.6 |

Power to the system is purely electrical and is **not** controlled by software. The software cannot remove power from the motor driver.

## 3. Safety Architecture

The E-stop is **dual-channel**:

- **SAF-01** — Channel 1 (hardware): one NC E-stop contact is wired into the CL57RS enable input (DI1) circuit such that actuating the E-stop places the drive in its *disabled* state, removing torque independently of the Opta.
- **SAF-02** — Channel 2 (software): the second NC E-stop contact is wired to Opta input **I1**. I1 HIGH = healthy. The control loop shall check I1 every scan; the programme shall not start unless I1 is HIGH.
- **SAF-03** — Both channels use NC contacts so that a broken wire is equivalent to an E-stop actuation (fail-safe, wire-break detecting).
- **SAF-04** — On I1 going LOW the software shall immediately command a driver quick-stop (write `0x0040` to register `0x6002`), abandon the running programme, and enter the E-STOPPED state.
- **SAF-05** — Releasing/resetting the E-stop shall **not** restart the programme. Recovery requires an explicit start button press, which restarts the programme **from the beginning** (never resume).
- **SAF-06** — The E-stop is a functional stop for the motor only; it does not isolate system power.

> **DI1 commissioning (resolves OI-1):** The CL57RS ships with DI1 set to N.C.-type/Enable (register `0x0145` = `0x0088`), meaning the drive is enabled with **nothing wired at all** — the opposite of fail-safe. The correct fail-safe combination is: reconfigure DI1 to N.O. logical polarity by writing `0x0008` to `0x0145`, save to EEPROM (write `0x2211` to `0x1801`), power-cycle the drive, and wire a genuinely separate N.C. contact pole from the E-stop (through an external 12–24 VDC supply, return to COM) into DI1. This is a **one-time commissioning step performed directly on the drive**, not re-written by the Opta sketch at boot — SAF-01 must protect the motor independent of whether the Opta has ever run. Verify by reading `0x1003` bit 1 (enable) before and after pressing E-stop, and by disconnecting the DI1 wire to confirm a break behaves the same as a press.

## 4. I/O Map (Arduino Opta)

| Signal | Opta resource | Type | Sense |
|---|---|---|---|
| E-stop (channel 2) | I1 (pin A0) | Digital in, 12–24 V | HIGH = healthy (NC chain closed) |
| Panel Start button (green) | I3 (pin A2) | Digital in | HIGH = pressed (NO button) — mirrors BTN_USER |
| Panel Pause button (amber) | I4 (pin A3) | Digital in | HIGH = pressed (NO button) |
| Panel Stop button (red) | I5 (pin A4) | Digital in | HIGH = pressed (NO button) — CTL-11 |
| Optional restart button | Spare input | Digital in | HIGH = pressed |
| Optional guard switch | I6 (pin A5) | Digital in | HIGH = guard closed/safe |
| Optional counter switch | I7 (pin A6) | Digital in | Edge = event (see §7.6) |
| (spare) | I2 (pin A1) | — | Unassigned — was remote start prior to Rev I |
| Internal start button | BTN_USER | Onboard button | Active LOW when pressed |
| Pilot light | Relay 1 (D0), NO contact | Relay out | Relay energised = light ON |
| RS-485 to CL57RS | Opta RS-485 port (half duplex) | Modbus RTU master | — |

I1/I3/I4/I5 are fixed panel wiring; the remaining assignments are defaults — the AI shall confirm actual assignments during interrogation (§10).

## 5. Communications — Modbus RTU to CL57RS

- **COM-01** — Protocol: Modbus RTU over RS-485, Opta is master. Slaves: CL57RS at **ID 1** (core hardware, DIP switches at the factory ID-1 position — zero-configuration device), Waveshare relay module at **ID 2** (optional stack light; address software-set once during commissioning and verified persistent across power cycles).
- **COM-02** — Serial: **9600 baud, 8N1** for the whole bus. Rationale: the relay module's baud-change command does **not** survive a power cycle (commissioning-verified), so the bus is locked to the module's factory default. The CL57RS is set to match — Pr5.22 (`0x01BD`) = 2 (9600), saved to EEPROM and verified persistent, DIP switches SW6–SW7 both OFF; data format register Pr5.24 (`0x01C1`) at default value 4 (8N1). Fit the 120 Ω terminator (SW8 ON) if the drive is the last node. Do **not** raise the relay module's baud: the change silently reverts at the next power loss and takes the stack light down with it.
- **COM-03** — Function codes — CL57RS: FC03 (read holding registers), FC06 (write single), FC10 (write multiple). Relay module: FC03 (read config registers), FC01 (read coils), FC05 (write single coil), FC06 (config writes, commissioning only). **FC0F (write multiple coils) is prohibited** — it draws an *Illegal function* exception through the ArduinoModbus transmission path on this hardware/library combination (commissioning-verified); stack-light updates use sequential FC05 writes instead (SW-13).
- **COM-04** — CL57RS parameters are 32-bit values split across a high-16-bit and low-16-bit register pair; single writes normally target the low word, but multi-register reads/writes must start at the high word.
- **COM-05** — A Modbus timeout or repeated CRC/exception errors on the **CL57RS** shall be treated as a fault: quick-stop attempted, FAULT state entered. Relay-module transaction failures are logged but non-fatal (COM-07).
- **COM-06** — Relay-module configuration persistence is asymmetric: the **address** survives power cycles (verified — set once to ID 2 during commissioning via register `0x4000`); the **baud** does not. Address changes are a one-time commissioning action; baud is never changed from 9600.
- **COM-07** — Boot check: `setup()` shall verify each expected slave answers at its address before entering IDLE. The CL57RS failing the check latches FAULT (core hardware). The relay module failing the check is logged and disables stack-light writes for the session (optional extra, non-fatal); all other behaviour continues.

## 6. CL57RS Register Reference (extracted from the CLRS Series user manual)

The AI shall use **only** these addresses and shall not invent register addresses. Anything not listed here must be taken from the CLRS manual, not guessed.

### 6.1 Configuration

| Address | Param | Meaning | Notes |
|---|---|---|---|
| `0x0001` | Pr0.00 | Pulses per revolution | Default **10000 P/R** — basis of all count conversions |
| `0x000F` | Pr0.07 | Software forced enable | 1 = enable regardless of DI1 (do **not** use in this fixture — defeats SAF-01) |
| `0x0145` | Pr4.02 | DI1 function/polarity | Default 0x88 = enable, N.C type |
| `0x0191` | Pr5.00 | Peak current | Unit 0.1 A |
| `0x0233` | Pr7.01 | Encoder resolution | CPR |
| `0x1801` | — | Control word | Write `0x1111` reset current alarm; `0x2211` save params to EEPROM |

### 6.2 Motion — PR0 immediate trigger (preferred method)

Write all 8 registers `0x6200`–`0x6207` in one FC10 frame; writing `0x0010` into `0x6207` triggers PR0 immediately.

| Address | Param | Meaning | Unit |
|---|---|---|---|
| `0x6200` | Pr9.00 | PR0 mode. Bits 0–3: 1 = position, 2 = velocity, 3 = homing. Bit 6: 0 = absolute, 1 = relative | — |
| `0x6201` | Pr9.01 | Target position, high 16 bits | pulses |
| `0x6202` | Pr9.02 | Target position, low 16 bits | pulses (32-bit signed overall) |
| `0x6203` | Pr9.03 | Velocity | rpm |
| `0x6204` | Pr9.04 | Acceleration | ms/1000 rpm |
| `0x6205` | Pr9.05 | Deceleration | ms/1000 rpm |
| `0x6206` | Pr9.06 | Pause time after move | ms |
| `0x6207` | Pr9.07 | Trigger control (maps to Pr8.02) — write `0x0010` to fire PR0 | — |

### 6.3 Trigger / stop register

| Address | Write | Effect |
|---|---|---|
| `0x6002` | `0x001P` | Start PR path P (P = 0–15) |
| `0x6002` | `0x0020` | Trigger homing |
| `0x6002` | `0x0021` | Set current position as zero |
| `0x6002` | `0x0040` | **Quick stop (E-stop)** |
| `0x6002` (read) | `0x0000` | Positioning complete, ready for new command |
| `0x6002` (read) | `0x010P` | Path P running |

### 6.4 Status / monitoring (read)

| Address | Meaning |
|---|---|
| `0x1003` | Motion status. Bit0 = fault, bit1 = enabled, bit2 = running, bit4 = command complete, bit5 = path complete, bit6 = homing complete |
| `0x1014` / `0x1015` | Feedback position, high/low 16 bits (pulses) |
| `0x1046` / `0x1047` | Feedback velocity, high/low (rpm) |
| `0x2203` | Current alarm code: `0x01` over-current, `0x02` over-voltage, `0x20` position following error, `0x40` current sampling fault, `0x80` shaft-lock fail, `0x100` auto-tune fault, `0x200` EEPROM fault, `0x08` encoder cable error (expected if the drive is enabled/moved with no motor/encoder connected — not a fault condition, just the drive protecting itself), `0x09` repeated input function assignment |

### 6.5 Relay module (Waveshare Modbus RTU Relay 4CH) — slave ID 2

The AI shall use **only** these addresses for the relay module, and shall write coils with FC05 only (COM-03).

| Address | Type | Meaning |
|---|---|---|
| `0x0000` | Coil | Channel 1 — stack light **Green** |
| `0x0001` | Coil | Channel 2 — stack light **Amber** |
| `0x0002` | Coil | Channel 3 — stack light **Red** |
| `0x0003` | Coil | Channel 4 — spare, unused |
| `0x4000` | Holding reg | Device address (FC06 to set — one-time commissioning only, see COM-06) |
| `0x8000` | Holding reg | Software version × 100 (FC03 — used as the SW-12 boot-check liveness read) |

`coilWrite()` takes a plain 0/1 — never the raw wire encoding `0xFF00`/`0x0000`, which truncates to a `uint8_t` and silently turns the coil OFF (commissioning-verified). Wiring: channels 1–3 NO contacts switch the fused 24 V feed to the corresponding tower-light segment, common return to the light's common terminal.

## 7. System States and Behaviours

### 7.1 State table

Blink convention: a "T blink" means the light toggles every T (T on, T off), timed with `millis()` — never `delay()`.

| State | Pilot light | Stack light | Entry | Exit |
|---|---|---|---|---|
| IDLE | Off | Amber | Power-up; or start pressed from COMPLETE resets to run | Start pressed → RUNNING (if I1 healthy and guard closed) |
| RUNNING | Solid on | Green | Start pressed | Programme complete → COMPLETE; pause → PAUSED; stop → IDLE (CTL-11); guard open → GUARD_PAUSED; restart → RESTART_DWELL; E-stop → E-STOPPED; error → FAULT |
| PAUSED | 2 s blink (in step with stack amber) | Green + Amber 2 s blink | Pause button pressed | Start pressed → resume RUNNING (from where it paused); stop → IDLE (CTL-11) |
| GUARD_PAUSED | 2 s blink (in step with stack amber) | Green + Amber 2 s blink | Guard switch opens during RUNNING (motion quick-stopped) | Guard closes → automatically resume RUNNING |
| RESTART_DWELL | 0.25 s blink | Green + Amber 2 s blink | Restart button pressed | 5 s elapses → RUNNING from the beginning. Start button is **ignored** during the dwell |
| COMPLETE | Off | Green + Amber | Programme finishes all cycles | Start pressed → new run from the beginning |
| E-STOPPED | 2 s blink | Red | I1 goes LOW (any state) | E-stop reset **and** start pressed → RUNNING from the beginning |
| FAULT | 2 s blink | Red | Driver alarm (`0x2203` ≠ 0), Modbus timeout, or status bit0 set | Operator investigates; start after fault clear → RUNNING from the beginning |

Pilot and stack-light blinks in the paused states share one period (2 s) and are reset together at state entry, so the two lamps toggle in step rather than beating against each other. Consequence: the pilot alone now shows the same 2 s blink for PAUSED, GUARD_PAUSED, E-STOPPED, and FAULT — the stack light disambiguates (Green + blinking Amber vs solid Red); on a build without the stack light the pilot cannot distinguish paused from fault (see OI-2).

Stack light scheme: **Amber solid** = powered and idle; **Green solid** = cycling now; **Green + blinking Amber** (toggling every 2 s) = paused/dwelling of any variety; **Green + Amber solid** = complete; **Red solid** = E-stop or fault. The only blinking element is the paused-family amber — one relay toggle per 2 s, only while paused, so relay wear is negligible even on long tests. Because no state is all-off, a **dark stack light unambiguously means the fixture is unpowered or the relay module is absent/failed** (pilot light remains authoritative per COM-07); the pilot light retains its own blink coding for close-up disambiguation.

Note: IDLE and COMPLETE are both "off." This is intentional — off means "motor not running, safe to approach" whether the fixture has never run yet or has just finished; RUNNING is the only solid-on state, so a lit light unambiguously means "cycling now, do not approach."

### 7.2 Start (CTL-01…04)

- **CTL-01** — BTN_USER starts the programme from the beginning when in IDLE, COMPLETE, E-STOPPED (after reset), or FAULT (after clear).

> **Positional reference note:** relative-move programmes lose their positional reference after any abort (E-stop, guard stop, fault quick-stop) — restart-from-beginning resumes relative motion from wherever the shaft stopped, permanently shifting the oscillation envelope by the aborted remainder. This is harmless for free continuous rotation but is a travel-limit hazard on any build with finite travel. Where travel is limited, recovery homing (or hardware-signalled limits per CTL-10) shall be considered — this is interrogation topic 5 (homing trigger: "after E-stop recovery").
- **CTL-02** — While RUNNING, the start button does nothing (except resuming from PAUSED).
- **CTL-03** — The green panel Start button (NO, on I3) shall mirror BTN_USER exactly.
- **CTL-04** — All buttons shall be software-debounced (nominal 30 ms) and edge-triggered.

### 7.3 Pause (panel button)

- **CTL-05** — Pause button: programme pauses (motion quick-stopped at end of current commanded move or via `0x6002` = `0x0040`); pilot light blinks 0.5 s; start button resumes.

### 7.4 Restart (optional)

- **CTL-06** — Restart button: programme returns to the beginning, dwells 5 s with 0.25 s blink, then starts. Start presses during the dwell are ignored.

### 7.5 Guard check (optional)

- **CTL-07** — The programme shall not start unless the guard input is HIGH (guard closed).
- **CTL-08** — If the guard opens while RUNNING, motion is quick-stopped and the state becomes GUARD_PAUSED. When the guard closes again the programme **resumes automatically** from where it stopped.

### 7.6 Cycle counting (optional hardware)

Two counting strategies exist; the AI shall establish which applies during interrogation:

- **CTL-09 (dead reckoning)** — Each commanded out-and-back move pair is one cycle; the counter increments (`i++`) when the driver reports the return move complete.
- **CTL-10 (hardware-signalled)** — The programme shall not proceed to the next move until the counter/limit input changes state; the state change marks the limit of travel, at which point direction reverses, and the cycle counter increments once per complete out-and-back pair.

### 7.7 Stop (panel button)

- **CTL-11** — The red panel Stop button (I5) aborts the programme from RUNNING, PAUSED, GUARD_PAUSED, or RESTART_DWELL: motion is quick-stopped (`0x6002` = `0x0040`), the programme is abandoned, and the state returns to IDLE (lights off — safe to approach). A subsequent start begins from the beginning; the §7.2 positional-reference note applies. Stop is a *functional* stop layered above the E-stop: it does not replace either E-stop channel and shall not be wired into the safety chain. On the Stop abort, the stack light annunciates the reset: **all three lights flash together 3 times at a 1 s interval** (a bounded ~5 s burst, non-blocking), then the IDLE indication (Amber solid) restores. Any state change — including a fresh Start press — cancels the annunciation immediately in favour of the new state's colours.

## 8. Motion Definition and Unit Translation

- **MOT-01** — The user expresses motion **from the motor's perspective**: either (a) rotation (revolutions or degrees per stroke) or (b) time at a given speed (e.g. "30 s at 200 rpm each direction").
- **MOT-02** — The AI shall translate to driver units and echo the derived values back to the user for confirmation before generating code:
  - pulses per stroke = revolutions × Pr0.00 (default 10000 P/R); degrees: pulses = (deg / 360) × Pr0.00
  - time-at-speed: revolutions = rpm × t(s) / 60, then convert to pulses as above
  - velocity register = rpm directly; acceleration/deceleration in ms per 1000 rpm change
- **MOT-03** — Oscillation uses PR0 in **relative position mode** (mode `0x0041`), alternating the sign of the target each stroke, unless interrogation establishes velocity mode or homing is required.
- **MOT-04** — Configurable per test: stroke length, speed, accel/decel, dwell at each end, number of cycles, direction of first stroke.

## 9. Software Requirements (coding constraints)

- **SW-01** — Target: Arduino Opta WiFi, compiled/flashed from the Arduino IDE (toolchain setup: §11). Output is a single `.ino` sketch.
- **SW-02** — Libraries: `ArduinoRS485` and `ArduinoModbus` (`ModbusRTUClient` over the Opta RS-485 port). No other external libraries without user agreement.
- **SW-03** — Architecture: a **non-blocking state machine** implementing the §7.1 table exactly. No `delay()` anywhere; all timing (blinks, dwells, debounce, Modbus poll intervals) via `millis()`.
- **SW-04** — All user-tunable values shall live in a single, clearly commented configuration block at the top of the sketch (a `struct Config` or equivalent group of `constexpr` globals): stroke, speed, accel, decel, dwells, cycle target, blink periods, debounce time, Modbus timeout, feature enables (guard, counter, pause, restart). The user must be able to retune the fixture by editing this block only, without re-running the AI.
- **SW-05** — Every Modbus transaction shall check for success; failures are retried once then escalate to FAULT (COM-05).
- **SW-06** — The E-stop input check (SAF-04) executes every loop pass before any other logic.
- **SW-07** — State, cycle count, and fault codes shall be printed to `Serial` on every state change for debugging.
- **SW-08** — Code shall compile with no warnings at default Arduino IDE settings and include a header comment recording the ConOps revision it was generated against.
- **SW-09** — The sketch shall write the confirmed motor peak current to Pr5.00 (`0x0191`, units of 0.1 A; default 3.0 A → value 30) during `setup()`, before any motion is possible, and verify by read-back; a failed write or mismatched read-back shall latch FAULT. The value shall not be saved to drive EEPROM (rewritten every power-up; saving wears the EEPROM for no benefit).
- **SW-10** — Every quantity offered as an option during interrogation shall be independently settable in the configuration block. In particular, dwell after the outward stroke and dwell after the return stroke are separate constants (either may be zero), never a shared value.
- **SW-11** — The sketch shall blink the onboard `LED_USER` (the LED above the RESET button, independent of relays and `LED_D0`-`LED_D3`) at a fixed period (default 500 ms) unconditionally on every loop pass, regardless of system state and regardless of whether a serial monitor is connected. This is the sole proof the control loop is genuinely still executing — a running motor is not proof, since a triggered velocity-mode move continues under the drive's own control even if the Opta's loop has hung. The heartbeat toggle shall be placed first in `loop()`, before any logic that could block, so a hang anywhere else in the loop is visible as the heartbeat stopping.

- **SW-12** — Boot check (COM-07): `setup()` verifies each configured slave answers before entering IDLE. CL57RS failure latches FAULT; relay-module failure logs a warning, sets a session flag that suppresses all stack-light writes, and continues.
- **SW-13** — Stack-light updates are performed on state *change* (sequential FC05 single-coil writes ordered **OFF-then-ON**, so an interrupted sequence can leave the light dimmer than intended, never wrong-coloured), plus a single FC05 amber toggle every 2 s while in a paused/dwelling state per the §7.1 scheme, plus the bounded CTL-11 reset-flash burst (3 flashes at 1 s, all channels, ~15 writes total). Never full rewrites every loop pass. FC0F is prohibited (COM-03).

## 10. AI Code-Generation Workflow

- **AI-01** — Before writing any code, the AI shall interrogate the user using the **grill-the-user** skill (or, if the skill is unavailable, the checklist below), asking about one topic at a time. The AI shall assume the user has not read this document, shall use plain language, and shall open by listing the topics to be covered so the user knows the scope of each stage.
- **AI-02** — Minimum interrogation checklist:
  1. Motion: rotation-based or time-at-speed? Values per stroke, both directions symmetric?
  2. Speed, accel/decel.
  3. Dwells — asked structurally, offering the distinct options: (a) after each stroke/direction change, (b) once per complete cycle, (c) both, with durations for each. The AI shall restate the resulting cycle sequence in words and the total test duration before moving on.
  4. Number of cycles, the definition of "one cycle" (default: one out-and-back pair), and behaviour at completion.
  5. Homing: required at all? (default for relative oscillation: none). If required — trigger (power-up, each start press, after E-stop recovery), method (home switch / limit switch / set-current-as-zero via `0x6002` = `0x0021`), and post-homing behaviour.
  6. Guard switch fitted? Which input?
  7. Cycle counting: dead reckoning or hardware-signalled (CTL-09 vs CTL-10)? Which input?
  8. Restart button fitted? Which input? (Start/Pause/Stop are standard panel hardware on I3/I4/I5 and are always implemented.)
  9. Driver setup — stated defaults, not open questions: pulses/rev is **stated** as 10000 (Pr0.00 default), changed only if the drive has been reconfigured via the tuning software or a non-default microstep resolution is deliberately in use. Peak current defaults to **3.0 A** and is a **mandatory safety confirmation** — the user must explicitly confirm or supply the correct value (rule of thumb: motor rated phase current × 1.2–1.4); the question cannot be skipped or silently defaulted.
  10. Stack light fitted on this build? (Default: yes — relay module at slave ID 2, Green/Amber/Red on channels 1/2/3, §7.1 mapping.) If not fitted, the boot check reports it absent and the build runs on the pilot light alone — no code changes.
  11. Anything unusual (asymmetric strokes, multi-segment profiles, first-move direction)?
- **AI-03** — The AI shall echo back a derived-parameter table (pulses, rpm, register values, I/O assignments) including the full sequence of one cycle written in words, the cycle time, and the total test duration, and obtain confirmation before generating the sketch.
- **AI-04** — The AI shall not weaken any SAF requirement and shall not use register `0x000F` software-forced enable.

## 11. Toolchain Setup and Deployment

- **TC-01** — The Opta is programmed via the standard **Arduino IDE** (not the separate Arduino PLC IDE add-on — see note below). Both Arduino IDE 1.8.10+ and IDE 2 are supported.
- **TC-02** — Board core: install **"Arduino Mbed OS Opta Boards"** via Tools > Board > Boards Manager (search "opta" or "Mbed OS Opta"). This core provides the libraries and examples for the Opta's onboard peripherals — inputs, outputs, relays, and the Wi-Fi/Bluetooth module on the WiFi variant. Install the latest offered version; take the default rather than pinning an old one.
- **TC-03** — Libraries: install **ArduinoRS485** and **ArduinoModbus** via Library Manager (Tools > Manage Libraries). These provide `RS485` and `ModbusRTUClient`, used throughout per Appendix A §B.1.
- **TC-04** — Board/port selection: select **Opta** under Tools > Board once the core is installed, connect via USB-C, and select the corresponding port under Tools > Port (shows as a USB serial device).
- **TC-05** — Compile and upload as a normal sketch (Sketch > Upload, or the toolbar arrow). Open Serial Monitor at 115200 baud to observe SW-07 logging.

> **Note — Arduino PLC IDE is a separate, optional product** for programming the Opta in IEC 61131-3 languages (Ladder, Structured Text, FBD, SFC, IL) instead of C++. It is not required and this ConOps, the grill-the-user skill, and the reference implementation all target the Arduino IDE / C++ path only. If a future requirement mandates PLC IDE, note that the Opta is configured differently depending on which IDE last flashed it, and Modbus RTU is typically wired up there via a no-code fieldbus configurator rather than the Appendix A API — porting would need a new implementation-contract appendix, not a reuse of this one.

## 12. Features for the Future (not in current scope)

- Cycle-count logging: periodic `Serial`/network reporting and persistence of the count across power loss (e.g. write to Opta flash every N cycles) so long-duration tests survive a brown-out.
- WiFi status dashboard / remote monitoring.
- Multiple-axis support (additional CLRS drives on the same RS-485 bus, slave IDs 3+ — ID 2 is reserved for the relay module).

## 13. Assumptions and Open Items

| ID | Item | Status |
|---|---|---|
| OI-1 | E-stop channel-1 / DI1 wiring resolved (see §3 note): reconfigure DI1 to N.O. (register `0x0145`=`0x0008`, saved to EEPROM), wire N.C. E-stop contact pole into it | **Resolved** — pending physical implementation and the wire-break verification test |
| OI-2 | On the pilot light, PAUSED, GUARD_PAUSED, E-STOPPED, and FAULT all share the 2 s blink (since Rev K); the stack light disambiguates. Acceptable on builds without a stack light, or distinguish? | Assumed acceptable — stack light is the default fit |
| OI-3 | "One cycle" defined as one out-and-back pair | Assumed — confirm per test at interrogation |
| OI-4 | Start from COMPLETE begins a fresh run | Assumed |
| OI-5 | Guard auto-resume (CTL-08) vs requiring a start press | Per user direction: auto-resume |

## Appendix A — Implementation Contract (binding on any code-generating AI)

This appendix exists because register addresses alone do not constrain code quality: the library API, timing idioms, and race guards must be pinned too. Generated code that violates this appendix is non-conforming regardless of whether it "works".

### B.1 Permitted ArduinoModbus API — nothing else

The **only** Modbus calls permitted are (exact signatures):

```cpp
ModbusRTUClient.begin(baud, SERIAL_8N1)                                   // returns bool
ModbusRTUClient.setTimeout(ms)
ModbusRTUClient.holdingRegisterWrite(slaveId, address, value)             // FC06, returns 1 on success
ModbusRTUClient.beginTransmission(slaveId, HOLDING_REGISTERS, address, n) // FC10 start
ModbusRTUClient.write(value)                                              // FC10 payload, n times
ModbusRTUClient.endTransmission()                                         // FC10 send, returns 1 on success
ModbusRTUClient.requestFrom(slaveId, HOLDING_REGISTERS, address, n)       // FC03, returns count read
ModbusRTUClient.read()                                                    // FC03 payload, returns long (<0 = none)
ModbusRTUClient.coilWrite(slaveId, address, value)                        // FC05 (relay module only), value plain 0/1 ONLY
```

Functions such as `writeSingleRegister()`, `readHoldingRegister()`, and `writeMultipleRegisters()` **do not exist in ArduinoModbus**. If the AI believes it knows a different API, it is wrong — use the list above.

Relay-module specifics (all commissioning-verified): `coilWrite()` takes a plain `0`/`1` — the library applies the Modbus wire encoding itself, and passing `0xFF00` truncates to a `uint8_t` of 0, silently turning the coil OFF. `beginTransmission(..., COILS, ...)` (FC0F multi-coil write) is **prohibited**: it draws an *Illegal function* exception from the relay module through this library regardless of how the payload is packed. Stack-light updates are sequential FC05 writes per SW-13.

### B.2 Mandatory initialisation pattern

```cpp
const float bitduration = 1.0f / (float)BAUD;
RS485.setDelays((uint32_t)(bitduration * 9.6f * 3.5f * 1e6f),
                (uint32_t)(bitduration * 9.6f * 3.5f * 1e6f));
if (!ModbusRTUClient.begin(BAUD, SERIAL_8N1)) { /* latch FAULT */ }
ModbusRTUClient.setTimeout(MODBUS_TIMEOUT_MS);
```

Any `while (!Serial)` wait must be time-bounded (≤ 2 s) so the fixture boots without a USB monitor attached.

### B.3 Pre-emit checklist — the AI shall verify every item before presenting code

1. Compiles conceptually: every identifier used is declared; enum names match everywhere.
2. No `delay()` anywhere — including button debounce, which shall be millis()-based and **edge-triggered** (a press generates exactly one event; held buttons do nothing further).
3. **BTN_USER and the remote start input are both implemented** (CTL-01/CTL-03), both debounced and edge-triggered.
4. Every Modbus failure path terminates in FAULT after one retry — never a silent wait, never a return value that leaves the state machine spinning (COM-05/SW-05).
5. Drive alarm register `0x2203` and status fault bit (`0x1003` bit 0) are polled during motion; a set alarm latches FAULT.
6. Move-completion detection is guarded against the stale-status race: an arming delay after each trigger, plus a dual-source check (`0x6002` == 0x0000 **and** `0x1003` running bit clear).
7. State-entry actions are guarded against re-entry (e.g. a held E-stop shall not re-log or re-send quick-stop every loop pass).
8. FAULT recovery attempts alarm reset (`0x1801` = `0x1111`) and **verifies** `0x2203` reads zero before permitting restart; some alarms clear only on power cycle and the code shall say so on the Serial log.
9. Every constant in the configuration block is referenced by the code at least once — no placebo knobs; every quantity interrogated with options is independently settable (SW-10).
10. The SW-09 peak-current write-and-verify is the first Modbus transaction in `setup()`, before any motion is possible.
11. `LED_USER` is initialised HIGH in `setup()` and toggled unconditionally every `HEARTBEAT_MS` (default 500 ms) as the very first thing in `loop()`, per SW-11 — this must never be gated behind a system state or a serial connection check.
12. Stack-light writes are FC05-only via `coilWrite()` with plain 0/1, fired on state change (ordered OFF-then-ON), plus the paused-state amber toggle every 2 s, plus the bounded CTL-11 reset-flash burst — nothing per-loop (SW-13); no `COILS` beginTransmission anywhere in the sketch.
13. The SW-12 boot check is present and correctly asymmetric: CL57RS liveness failure latches FAULT; relay-module liveness failure warns, suppresses stack-light writes, and continues.

### B.4 Reference implementation

The grill-the-user skill bundles a known-good sketch (`references/reference-implementation.ino`) conforming to this appendix. Generating AIs shall **adapt that file** — normally only the configuration block and the phase sequence change per test — rather than writing from scratch. Deviations from the reference structure must be justified in the generated file's header comment.

## 14. Revision Control



| Rev | Date | Author | Description |
|---|---|---|---|
| A | 2026-07-14 | Ross Wilson | Initial release. Rebuilt from draft README: added dual-channel E-stop architecture, I/O map, CL57RS register reference (CLRS manual), state table incl. FAULT/GUARD_PAUSED, unit-translation rules, coding constraints, AI workflow, future features, open items. |
| B | 2026-07-14 | Ross Wilson | Post-trial improvements to AI workflow (§10): interrogation now opens with a topic roadmap and assumes user has not read this document; dwell question made structural (per-stroke vs per-cycle vs both) with cycle sequence and total duration restated; homing promoted to an explicit interrogation stage (trigger, method, post-homing behaviour); AI-03 table must state cycle sequence and total test duration prominently. |
| C | 2026-07-14 | Ross Wilson | Driver-setup interrogation reworked for non-specialist users: pulses/rev now a stated default (change only if drive reconfigured); peak current a mandatory 3.0 A confirm-or-change safety question, written to Pr5.00 at every power-up with read-back verify (new SW-09). New SW-10: interrogated quantities independently settable in config (separate out/back dwells). Positional-reference-loss note added to §7.2 (relative moves after abort; recovery homing where travel limited). |
| D | 2026-07-14 | Ross Wilson | Guardrails for lower-capability code-generating AIs after second trial produced non-compiling output: new Appendix A Implementation Contract (permitted ArduinoModbus API only, mandatory init pattern, 10-point pre-emit checklist covering delay()/edge-triggering/BTN_USER/fault escalation/completion race/re-entry guards/placebo knobs). Known-good reference implementation bundled in grill-the-user skill; generated code shall adapt it, not be written from scratch. |
| E | 2026-07-28 | Ross Wilson | New §11 Toolchain Setup and Deployment: Arduino IDE install steps (Arduino Mbed OS Opta Boards core, ArduinoRS485/ArduinoModbus libraries, board/port selection), and a note distinguishing the Arduino IDE / C++ path (this document's scope) from the separate, optional Arduino PLC IDE (IEC 61131-3) path. Subsequent sections renumbered §12-14. Restored a Revision Control section header dropped in a prior edit. |
| F | 2026-07-28 | Ross Wilson | New SW-11: onboard LED_USER blinks every 500 ms unconditionally, first action in loop(), as the sole proof of a live control loop (a running motor is not proof, since velocity-mode moves continue under the drive's own control after triggering even if the loop has hung). Added to Appendix A pre-emit checklist (item 11) and reference implementation. Also noted in §6.4 register table that alarm code 0x08 = encoder cable error (missing from the original extraction) and 0x09 = repeated input function assignment. |
| G | 2026-07-28 | Ross Wilson | Pilot light convention changed: RUNNING is now solid on (was 1 s blink), COMPLETE is now off (was solid on) - so E_STOPPED/FAULT (2 s blink) are the only blinking states, reducing relay wear on long tests. IDLE and COMPLETE are both off by design; note added to §7.1 explaining the overlap is intentional (off = safe to approach, solid-on = cycling now). Reference implementation's updatePilot() and BLINK_RUN_MS constant updated to match. |
| H | 2026-07-28 | Ross Wilson | OI-1 resolved: documented the correct fail-safe wiring for the DI1/channel-1 E-stop connection. CL57RS ships with DI1 as N.C.-type/Enable (drive enabled with nothing wired) - the fail-safe fix is reconfiguring DI1 to N.O. logical polarity (register 0x0145 = 0x0008, saved to drive EEPROM, one-time commissioning step independent of Opta software) and wiring a genuinely separate N.C. E-stop contact pole into it. Verification procedure added (0x1003 bit1 before/after E-stop press, and a wire-break test). |
| I | 2026-08-27 | Ross Wilson | Stack light integration and bus re-architecture from commissioning. New hardware: Waveshare Modbus RTU Relay 4CH (slave ID 2, address software-set once, persistence across power cycles verified) driving a 3-light 24 V tower — Green/Amber/Red on channels 1/2/3; §7.1 gains a stack-light column (steady traffic-light colours only). Bus baud moved 115200 → 9600 for the whole trunk after commissioning showed the relay module's baud change does not survive a power cycle while the CL57RS's EEPROM-saved settings do (COM-02/COM-06); CL57RS Pr5.22 = 2 saved to EEPROM, SW6–SW7 both OFF. Panel Start/Pause/Stop buttons on I3/I4/I5 (green/amber/red), replacing the I2 remote-start assignment (I2 now spare); guard → I6, counter → I7. New CTL-11 (Stop aborts to IDLE), SW-12 (boot check: motor fatal, relay non-fatal), SW-13 (stack-light writes FC05-only, on state change, OFF-then-ON). Appendix A: coilWrite() added to permitted API (plain 0/1 only — 0xFF00 truncates to OFF), FC0F banned (draws Illegal function via ArduinoModbus on this hardware regardless of payload packing), pre-emit checklist items 12–13. New §6.5 relay-module register reference. Repaired a corrupted title line introduced by a prior edit. |
| J | 2026-08-27 | Ross Wilson | Stack light scheme revised per user direction: IDLE now Amber solid (was all off), PAUSED/GUARD_PAUSED/RESTART_DWELL now Green + Amber blinking every 2 s (was Amber solid), COMPLETE now Green + Amber solid (was all off), E-STOPPED/FAULT remain Red solid. Consequence documented in §7.1: no state is all-off, so a dark stack light unambiguously means unpowered or module absent/failed. SW-13 and checklist item 12 amended to permit the single periodic amber toggle (one FC05 write per 2 s, paused states only) alongside the on-state-change writes; relay-wear note updated (negligible - blink exists only while paused). Reference implementation updated to match. |
| K | 2026-08-27 | Ross Wilson | Pilot paused-blink matched to the stack light: PAUSED and GUARD_PAUSED pilot blink moved 0.5 s → 2 s, tied to the same constant as the stack amber so the two lamps toggle in step (both timers reset at state entry). Consequence noted in §7.1 and OI-2 updated: pilot alone no longer distinguishes paused states from E-STOPPED/FAULT (all 2 s); the stack light carries that distinction. RESTART_DWELL keeps its 0.25 s pilot blink deliberately. |
| L | 2026-08-27 | Ross Wilson | CTL-11 reset annunciation added per user direction: pressing Stop flashes all three stack lights together 3 times at a 1 s interval before the Amber idle indication restores; implemented as a non-blocking overlay cancelled by any state change. SW-13 and checklist item 12 amended to include the bounded flash burst in the permitted writes. Reference implementation updated to match. (Not in this revision: a drive-enable verification gate before programme start was added to the reference pending confirmation it resolves the reported E-stop recovery failure — to be specified as a SAF requirement once verified.) |

*Document owner: Ross Wilson. Changes require a new revision line above; regenerated code must reference the revision used (SW-08).*
