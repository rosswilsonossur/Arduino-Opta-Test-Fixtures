# YK-LED4006-RS485 Countdown Display — Integration Plan

**Document ID:** GTF-DIP-001 Rev A (working plan — folds into GTF-CONOPS-001 as a
new revision once bench-validated)
**Applies to:** GTF-CONOPS-001 Rev L
**Hardware:** YK-LED4006-RS485 — 6-digit 0.4" LED display module, Modbus RTU
slave over RS-485 (AliExpress item 1005007513196126, colour option
"YK-LED4006-RS485")

---

## 1. Purpose and scope

Add a panel-mounted numeric display to the GTF that shows, at a glance, how much
of the running test remains. Two modes, selected by a **global flag in the
sketch's configuration block** (compile-time, per SW-04 — not a runtime switch):

- **COUNTDOWN CYCLES** — display shows remaining cycles (e.g. `9000` counting
  down to `0`).
- **COUNTDOWN MINUTES** — display shows remaining time to complete as
  minutes.seconds (e.g. `90.00` counting down through `89.59` … to `0.00`).

Validation is **offline-first**: a standalone reference sketch
(`commissioning/Display_Countdown_Reference/`) runs on a bench Opta with *only
the display* connected — no motor, no relay module, no panel. Only after the
§7 checklist passes does the display driver get lifted into the GTF reference
implementation and the ConOps revised.

## 2. Device digest (from the vendor register map)

Transcribed and translated from the vendor's register table. **Rule as per
ConOps §6: use only these addresses; anything not listed here is taken from the
vendor sheet, never guessed.** Items marked ⚠ are unverified translations to be
confirmed on the bench (§7).

All registers are holding registers. Function codes: **FC03** (read), **FC06**
(write single), **FC10** (write multiple) — all within the Appendix A permitted
ArduinoModbus API.

| Addr | Function | Range | Default | Notes |
|---|---|---|---|---|
| `0x0000` | Decimal point (number of decimal places) | 0–N (N = digit count) | 0 | 0 = no decimal point. Vendor note: write in one FC10 frame together with `0x0001`–`0x0002` when the decimal position is fixed. ⚠ verify semantics (V4) |
| `0x0001`–`0x0002` | Integer display value | ±2,147,483,647 | 0 | 32-bit signed, **high word first** (same convention as CL57RS, COM-04) |
| `0x0003`–`0x0004` | Float display value | 32-bit float | — | High word first; full-width display, zero-padded. Not used by this integration |
| `0x0005`–`0x000A` | ASCII characters (2 per register) | — | "YK…" | Right-aligned. Not used |
| `0x000B`–`0x0010` | Raw segment codes, digit pairs (2,1)…(12,11) | — | — | Not used |
| `0x0011`–`0x0016` | Power-on display value (segment codes) | — | `0xFFFF` | Shown from power-up **until the first bus write arrives**, then the device switches to the `0x0017` mode. Default = all segments lit — a free lamp test at power-on; leave as-is |
| `0x0017` | Display mode | 0–3 | 1 | 0 = integer (regs 0–2), 1 = float (3–4), 2 = ASCII (5–10), 3 = segment (11–16). **This integration uses 0** |
| `0x0018` | Brightness | 0–6 | 4 | 0 = off … 6 = max |
| `0x0019` | Blink digit select (bitmask, one bit per digit) | 0–(2ᴺ−1) | 0 | 0 = no blink. 6-digit all-blink = `0x003F` ⚠ verify bit→digit mapping (V6) |
| `0x001A` | Blink interval | 1–10 | — | 1 = 100 ms … 10 = 1000 ms |
| `0x001B` | Power-on display content | 0–3 | 3 | 0 blank / 1 device address / 2 last value at power-off / 3 the `0x0011`–`0x0016` segment values. Leave at default |
| `0x001C` | Baud rate code | 0–5 | **1 = 9600** | 0–5 → 4800/9600/19200/38400/57600/115200. Factory default already matches COM-02. **Never changed** — the bus is locked to 9600 |
| `0x001D` | Device address | 1–127 | **1** | 0 = broadcast. ⚠ **Factory default collides with the CL57RS (ID 1)** — must be re-addressed before ever touching the fixture trunk (§5) |
| `0x001E` | Save / restore | 1, 30 | — | Write 1 = save parameters to non-volatile memory; write 30 = factory reset + restart. Used only by the commissioning tool (address save) |
| `0x001F` | Version / model | — | — | **Read-only — the liveness-check register** (the display's equivalent of the relay module's `0x8000`) |

Other device facts:

- 6 digits, so integer mode displays up to `999999` (cycles) and
  minutes.seconds up to `9999.59` (≈166 h) — ample for any GTF programme.
- Supply: wide-range DC per the listing — intended to run from the fixture's
  24 V rail. ⚠ Confirm the delivered unit's marked supply range before wiring
  (V1).
- Serial format: the vendor sheet gives no parity option, assumed **8N1** ⚠
  (confirmed implicitly the moment V2 gets a reply at 9600 8N1).

## 3. Bus architecture decision

| Item | Decision | Rationale |
|---|---|---|
| Slave ID | **3** (software-set, saved to device NVM) | 1 = CL57RS (core), 2 = relay module. 3 is the next free ID, mirroring the relay-board precedent. Note: ConOps §12 pencilled IDs 3+ for future extra drives — if that future arrives, extra drives start at 4, or the display (software-settable) moves; record whichever in the ConOps at that time |
| Baud | 9600 8N1, unchanged | COM-02: bus is locked to the relay module's only power-cycle-safe baud. Display factory default is already 9600 — zero configuration, and its `0x001C` is never written |
| Failure policy | **Non-fatal peripheral**, exactly like the relay module (COM-07) | A cosmetic display must never stop a 9,000-cycle test. Boot-check failure logs a warning and suppresses display writes for the session; runtime write failures log, then the driver retries/reinitialises in the background |
| Write policy | Value writes **only on change**; steady-state worst case one FC10 (3 registers) per second (minutes mode) | SW-13 spirit: never rewrites every loop pass. One FC10 write+reply at 9600 is ~30 ms on the wire |
| E-stop latency budget | Accepted | Modbus transactions block (ConOps latency note). Worst case adds one ~50 ms transaction (~100 ms with the single retry) per second to the loop — same class as the existing paused-state stack-light toggle, and channel-1 hardware E-stop remains the primary (SAF-01) |
| Config persistence | Address saved once (`0x001E` = 1) at commissioning; **everything else rewritten at every boot**, never saved | Same philosophy as SW-09: rewriting at power-up costs nothing and spares the device EEPROM; it also self-heals a display swap. Only the address must persist (it's what lets us reach the device) |

## 4. Wiring

**Bench (offline validation):**

- Opta RS-485 A/B → display A/B (A–A, B–B; if no reply at V2, swap — pair
  polarity labelling varies by vendor).
- Display V+/GND → bench 24 V DC supply (confirm marked range first, V1).
- Common 0 V reference between Opta RS-485 GND and display GND.
- Short bench leads: termination not critical; no changes to the fixture's
  120 Ω arrangement are implied by the bench.

**Fixture (after validation):** display joins the existing trunk as a third
drop (CL57RS, relay module, display). Keep the stub short; the drive keeps the
SW8 terminator if it remains the physical end of line. 24 V from the same fused
feed as the stack light.

## 5. Commissioning — address 1 → 3 (one-time)

**Never connect a factory-fresh display to the fixture trunk.** At factory
address 1 it sits on the same address as the CL57RS: both devices would process
writes addressed to ID 1 and both would reply, colliding on the bus (same
hazard the relay board had, handled the same way).

Tool: `commissioning/Display_OneTime_SetAddr3/` — serial-menu sketch modelled
on `RelayBoard_SetAddr2`, run with **only the display** on the bench bus:

1. **STEP1** — verify the display answers at 1 (reads `0x001F`), write
   `0x001D` = 3 (FC06), save (`0x001E` = 1), verify it answers at 3.
2. Power-cycle the display.
3. **STEP2** — verify it *still* answers at 3: the persistence proof (the
   relay board taught us persistence is asserted only after it is verified).
4. **STEP3** — (later, on the fixture trunk) verify all three slaves answer:
   CL57RS at 1 (`0x1003`), relay at 2 (`0x8000`), display at 3 (`0x001F`).
5. **RESCUE** — if a display is at an unknown address: broadcast (address 0)
   the address write. Broadcast gets no reply (the tool says so), then probes
   address 3. Only ever with the display alone on the bus.

## 6. Reference implementation design (`Display_Countdown_Reference`)

Standalone bench sketch, structured so the display driver section lifts
verbatim into the GTF reference implementation later. Follows the Appendix A
contract even though it is a commissioning-class tool: permitted-API-only
Modbus helpers with one retry, `RS485.setDelays()` before
`ModbusRTUClient.begin()`, non-blocking `millis()` timing throughout, no
`delay()`, LED_USER heartbeat first in `loop()` (SW-11), bounded
`while (!Serial)` wait.

### 6.1 The mode flag (the point of the exercise)

```cpp
enum class CountdownMode : uint8_t { CYCLES, MINUTES };

// ---- THE GLOBAL MODE FLAG — pick one ----
constexpr CountdownMode MODE = CountdownMode::CYCLES;  // or ::MINUTES
constexpr uint32_t CYCLE_TARGET   = 25;   // cycles mode: cycles to count down
constexpr uint32_t MINUTES_TARGET = 2;    // minutes mode: duration to count down
```

(Bench-friendly defaults so DONE is reachable in under a minute of watching;
set the real programme values — e.g. 9000 cycles — when lifted into the GTF.)

### 6.2 Rendering rules

| Mode | Display shows | Value written (`0x0001`–`0x0002`) | Decimal places (`0x0000`) |
|---|---|---|---|
| CYCLES | remaining cycles, e.g. `9000` | remaining | 0 |
| MINUTES | remaining min.sec, e.g. `89.59` | `minutes × 100 + seconds` | 2 |

Every value update is **one FC10 frame** writing registers
`0x0000`–`0x0002` = `[decimal places, value high, value low]` — the exact
combined write the vendor sheet recommends, and atomic on the display.

### 6.3 Behaviour (bench state machine)

- **Boot:** detect display (read `0x001F`, log version) → init (mode = 0,
  brightness, blink interval, blink off) → show the full target. Display
  absent at boot is **non-fatal**: warn and keep probing at 1 Hz — plugging it
  in later must just work.
- **WAIT_START:** display shows the full target. BTN_USER starts.
- **COUNTING:**
  - CYCLES: a simulated cycle completes every `SIM_CYCLE_PERIOD_MS`
    (bench stand-in for the real fixture's "return stroke complete" event);
    remaining decrements and the display updates.
  - MINUTES: wall-clock countdown; display updates when the visible second
    changes (max one write/second).
  - BTN_USER during counting restarts from the top.
- **DONE:** display shows `0` / `0.00` and **all digits blink**
  (`0x0019` = `0x003F`) as the attention state. BTN_USER resets.
- **Comms failure:** each transaction retried once; a failed update marks the
  display offline (logged once), the countdown itself keeps running, and a 1 Hz
  background probe re-detects, **re-initialises** (a power-cycled display
  reverts to its power-on state — mode register included, since nothing is
  saved), and rewrites the current value and blink state. Unplug/replug
  mid-count is a validation step (V7), not an afterthought.

### 6.4 Serial log

`115200` baud, fixture-style lines: state changes, every simulated cycle,
minute boundaries plus the final 10 s in minutes mode, display
offline/online transitions, and every Modbus failure with the register it hit.

## 7. Offline validation checklist

Run on the bench (Opta + display only). Record pass/fail against each ID —
the ConOps revision cites these as "commissioning-verified".

| ID | Step | Expected |
|---|---|---|
| V1 | Inspect delivered unit: marked supply range, terminal labels | 24 V within marked range; A/B/V+/G identified |
| V2 | Power display only | Power-on pattern (all segments, `0x0011`–`0x0016` defaults) until first write |
| V3 | Run `Display_OneTime_SetAddr3` STEP1+STEP2 | Answers at 3 after save **and** after power cycle; version register value logged |
| V4 | Flash `Display_Countdown_Reference`, MODE = CYCLES, defaults | Boot log shows version; display shows `25`; decimal-point semantics confirmed en route (V5 shows `x.yz` form) |
| V5 | MODE = MINUTES | Display shows `2.00`, counts `1.59 … 0.00` at 1 Hz — confirms `0x0000` = "number of decimal places" reading of the vendor sheet |
| V6 | Let either mode reach 0 | All 6 digits blink at the configured interval — confirms `0x003F` mask and bit→digit mapping |
| V7 | Unplug display A/B mid-count, ~10 s, replug | "display offline" logged once; countdown continues; on replug: re-init, correct current value restored, no stale/garbled digits |
| V8 | Power-cycle display mid-count (V+ off/on) | Same recovery as V7 — proves the re-init path handles the mode register reverting |
| V9 | BTN_USER: start, mid-count restart, reset from DONE | Each press acts exactly once (debounced, edge-triggered); blink cleared on restart |
| V10 | Soak: MODE = CYCLES, `CYCLE_TARGET` = 9000, `SIM_CYCLE_PERIOD_MS` = 2000 (~5 h) | Reaches 0 with no missed updates, no comms degradation; heartbeat never stops |

## 8. GTF integration (after §7 passes — not before)

The display driver (register namespace + helpers + service function) lifts into
the GTF reference implementation. Hook points, by function, in
`skill/grill-the-user/references/reference-implementation.ino`:

1. **Config block additions (SW-04):** `DISPLAY_FITTED`, `DISPLAY_ID` (3),
   `DISPLAY_MODE` flag (CYCLES / MINUTES), brightness — all beside the
   existing `STACKLIGHT_FITTED`.
2. **Boot check (SW-12):** display liveness read (`0x001F`) beside the relay
   module's, same asymmetry — absence warns, sets `displayOk = false`,
   continues.
3. **Cycles mode source:** the one place `cycleCount++` happens
   (`pollDriveWhileMoving()`, return-stroke-complete branch) also calls
   `displayShowRemaining()`. Plus at `startProgramme()` (full target) and at
   COMPLETE entry (0 + blink). PAUSED freezes the value by doing nothing.
4. **Minutes mode source — decision needed at integration (OI-D3):**
   - *Recommended:* derive time-to-complete from cycles — remaining cycles ×
     nominal cycle time (computable from the config block: stroke time at
     speed ×2 + both dwells). Single source of truth, self-corrects every
     cycle, pauses correctly for free.
   - *Alternative:* a plain run-time countdown (total configured minutes minus
     accumulated RUNNING time) for tests defined by duration rather than
     cycles.
   - The reference sketch's rendering path is identical either way — this
     decision affects only where the number comes from.
5. **State overlays:** COMPLETE = 0 + all-blink; FAULT / E-STOPPED — leave the
   remaining value frozen (the stack light already annunciates red; the frozen
   number is diagnostic gold: it says exactly where the test died).

**Proposed ConOps deltas (draft for the next revision, applied only with the
§7 results in hand):** §2 hardware table row (optional display + this plan
doc); COM-01 add slave ID 3; COM-03 add display FCs (03/06/10); COM-07/SW-12
extend the non-fatal boot check; new §6.6 display register table (the §2 digest
above, only-these-addresses rule); SW-13-style write policy (on change, ≤1 Hz);
§12 multi-axis note updated (future drives start at ID 4).

## 9. Open items

| ID | Item | Closed by |
|---|---|---|
| OI-D1 | Supply range and terminal labels of the delivered unit | V1 |
| OI-D2 | Decimal-point register semantics (places vs position), blink bit→digit mapping | V5, V6 |
| OI-D3 | Minutes-mode source in the GTF: derived from remaining cycles (recommended) vs accumulated run-time | Decision at §8 integration |
| OI-D4 | Address-save persistence across power cycles on this specific unit | V3 |
| OI-D5 | Whether the display answers at the new address immediately after the `0x001D` write or only after save/power-cycle (tool handles both) | V3 |

---

*Owner: Ross Wilson. This plan is working documentation; the ConOps remains the
authoritative spec and takes the validated results as a new revision.*
