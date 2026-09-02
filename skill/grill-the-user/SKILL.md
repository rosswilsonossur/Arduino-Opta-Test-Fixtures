---
name: grill-the-user
description: Interrogate the user to pin down complete, unambiguous requirements BEFORE generating any embedded/automation code, especially test fixture control programs (Arduino Opta, Modbus stepper drivers, state machines). Use this skill whenever the user asks to generate, regenerate, or reconfigure fixture/rig/machine control code, mentions a ConOps document, or gives a vague motion request like "cycle it back and forth". Do not write code until the interrogation and confirmation steps in this skill are complete.
---

# Grill the User

You are about to generate control code for physical hardware. Wrong assumptions move real motors. This skill forces a structured interrogation so that every parameter in the generated code is either (a) explicitly given by the user, (b) defined in the project's ConOps document, or (c) a stated, confirmed assumption. Never silently guess.

## Workflow

### Step 1 — Load context
Read the project ConOps document in full (for the generic test fixture this is `CONOPS.md`, document GTF-CONOPS-001). It defines the hardware, safety requirements (SAF-xx), I/O map, register reference, state machine, coding constraints (SW-xx), and the binding Implementation Contract (Appendix A). Those are **fixed** — do not re-ask about them and do not weaken them. Your interrogation covers only the per-test configurable items.

If no ConOps exists, say so and interrogate hardware basics too (controller, driver, comms, safety) before anything else.

### Step 2 — Interrogate, one topic at a time

**Assume the user has NOT read the ConOps or this skill.** Use plain language, briefly define any term of art on first use (e.g. "dwell = a timed stop with the motor holding position"), and never reference requirement IDs at them without saying what the requirement means.

**Open with a roadmap** before the first question, so the user knows the scope and how much detail to give at each stage. Something like:

> "I'm going to ask you about 11 topics, one at a time: (1) the motion itself, (2) speed and ramps, (3) dwells, (4) cycles, (5) homing, (6) guard switch, (7) how cycles are counted, (8) extra buttons, (9) driver setup, (10) the stack light, (11) anything unusual. Short answers are fine — I'll show you everything in a confirmation table at the end before writing any code."

Then ask about **one topic per message**, in this order. Skip a question only if the user has already answered it unambiguously in the conversation. Offer sensible defaults so the user can answer quickly ("default 10000 pulses/rev — keep?").

1. **Motion definition.** How is the stroke expressed: rotation (revs or degrees) or time at a speed? Get the value. Both directions symmetric?
2. **Speed and ramps.** Speed in rpm. Accel/decel (offer a default, e.g. 50 ms/1000 rpm).
3. **Dwells.** Ask this explicitly and structurally — do NOT ask an open "any dwells?" question. A dwell can occur in two distinct places and users routinely mean only one of them:
   - **(a) after each stroke / direction change** — i.e. twice per out-and-back pair, or
   - **(b) once per complete cycle** — after the out-and-back pair, before the next cycle starts, or
   - **(c) both**, with possibly different durations.
   Present those options, get durations for each that applies, and restate the resulting sequence in words ("so one cycle is: out 5 min → back 5 min → dwell 5 min — correct?") before moving on. State the resulting total cycle time and total test duration here, not just in the final table — a wrong dwell placement changes total runtime and must be caught early.
4. **Cycles.** How many cycles? Confirm the definition of one cycle (default: one out-and-back pair). What happens at completion (default: stop, light solid)?
5. **Homing.** Explicit stage, not an afterthought:
   - Is a home/reference position required at all? (For pure relative oscillation the default is **no homing**.)
   - If yes: what **triggers** it — power-up, every start-button press, after E-stop recovery, or once per N cycles?
   - What **method** — home switch, limit switch, or "set current position as zero" (`0x6002` = `0x0021`)?
   - What happens after homing — move to a start offset, or begin cycling from home?
6. **Guard switch.** Is a guard/cover switch fitted? If yes: which input, and confirm behaviour (no start until closed; opening pauses; closing auto-resumes).
7. **Cycle counting method.** Dead reckoning (count commanded moves) or hardware-signalled (a switch edge marks end of travel, triggering reversal and count)? If hardware: which input, and what does one switch event mean?
8. **Extra buttons.** Start, Pause, and Stop are standard panel hardware (green/amber/red on I3/I4/I5) and are always implemented — do not re-ask about them. Ask only: is a *restart* button fitted, and on which input?
9. **Driver setup.** Do NOT ask open technical questions here — the user is likely not a drives specialist. Instead, state the defaults and handle each as follows:
   - **Pulses per revolution — statement, not a question.** Say: "The drive will be used at its default resolution of 10,000 pulses per motor revolution. You only need to change this if the drive's Pr0.00 setting has been reconfigured with the StepperOnline tuning software, or you're deliberately running a non-default microstep resolution — has either happened?" Default answer no; proceed at 10,000.
   - **Motor peak current — mandatory safety confirmation, cannot be skipped or defaulted silently.** Say: "The motor current limit will be set to **3.0 A peak** by default, written to the drive at every power-up. This is a safety-relevant setting: too high overheats the motor and can over-torque the fixture and the unit under test. Rule of thumb: peak current ≈ motor rated phase current × 1.2–1.4 (check the motor label or datasheet). Confirm 3.0 A is acceptable for your motor, or give the correct value." Do not proceed to the next topic until the user has explicitly confirmed or supplied a value; "whatever" or silence is not acceptance — restate and ask again.
10. **Stack light.** Is the 3-colour stack light (relay module) fitted on this build? Default: yes — Waveshare relay module at slave ID 2, Green/Amber/Red on channels 1/2/3, mapped per ConOps §7.1: Amber solid = idle, Green solid = cycling, Green + blinking Amber (2 s) = paused/dwelling, Green + Amber solid = complete, Red solid = E-stop/fault; Stop abort flashes all three lights 3x at 1 s before Amber restores (CTL-11). A dark stack light means unpowered or module absent — no state is all-off. If not fitted, say so — the code is identical either way; the boot check simply reports it absent and continues on the pilot light alone.
11. **Anything unusual?** Asymmetric strokes, multi-segment profiles, direction of first move, anything the previous topics didn't cover.

Rules while interrogating:
- If an answer is ambiguous ("fast", "a few seconds"), push back for a number.
- If the user's answer doesn't map cleanly onto the options you offered, restate what you think they mean in your own words and ask for a yes/no — never proceed on an interpretation the user hasn't confirmed.
- If an answer conflicts with the ConOps safety requirements, refuse that part and cite the SAF requirement (explained in plain words).
- If the user says "just pick something sensible", you may — but it must appear in the Step 3 table flagged as *assumed*.

### Step 3 — Echo back derived parameters
Before writing code, present a confirmation table translating the user's answers into machine units, and wait for explicit confirmation. Translation rules:

- pulses per stroke = revolutions × pulses/rev; degrees → pulses = (deg/360) × pulses/rev
- time-at-speed → revolutions = rpm × seconds / 60, then to pulses
- velocity register = rpm directly; accel/decel in ms per 1000 rpm

The table must include: stroke (user units AND pulses), speed (rpm), accel/decel, dwell placement and durations, **the full sequence of one cycle written out in words, the cycle time, and the total test duration** (make these prominent — they are the fastest way for a user to spot a misunderstood dwell), cycle target, counting method, homing behaviour, all I/O assignments, feature enables, and every assumption. Mark assumptions clearly.

### Step 4 — Generate
Only after confirmation, generate the single `.ino` sketch. **Do not write it from scratch: adapt the bundled known-good reference implementation at `references/reference-implementation.ino`.** Normally only the configuration block and the phase sequence change per test; any structural deviation must be justified in the generated file's header comment.

Binding constraints (ConOps §9 and Appendix A — read Appendix A before generating):
- Use **only** the ArduinoModbus calls listed in Appendix A §B.1. Calls like `writeSingleRegister()` / `readHoldingRegister()` / `writeMultipleRegisters()` do not exist in this library — if you believe you know a different API, you are wrong.
- Non-blocking millis()-based state machine; no `delay()` anywhere, including debounce. Buttons edge-triggered.
- BTN_USER **and** the remote start input both implemented.
- Every Modbus failure path ends in FAULT after one retry; alarm register polled during motion; completion detection uses the arming delay + dual-source check.
- The sketch shall **write the confirmed peak current to Pr5.00 (`0x0191`, units of 0.1 A)** as the first Modbus transaction in `setup()`, verified by read-back; failure latches FAULT. Do not save to drive EEPROM.
- Relay module (stack light): **FC05 `coilWrite()` only, value plain 0/1** — never `0xFF00`/`0x0000` (truncates to OFF) and never FC0F / `beginTransmission(..., COILS, ...)`, which draws an *Illegal function* exception on this hardware. Writes fire on state change only, ordered OFF-then-ON (SW-13).
- Bus serial is **9600 8N1 for both slaves** (COM-02) — never change the relay module's baud; the change silently reverts on power loss (COM-06). Boot check per SW-12: CL57RS failure latches FAULT, relay-module failure warns and continues with stack-light writes suppressed.
- **Every quantity the interrogation offered options on must be independently settable in the config block**, and every config constant must actually be referenced by the code — no placebo knobs. Dwells after outward and return strokes are always separate constants (either may be 0).
- **Blink the onboard `LED_USER` every 500 ms, unconditionally, as the first action in `loop()`** (SW-11). This is the only way to visually confirm the control loop is alive independent of the serial monitor — a running motor is not proof, since velocity-mode motion continues under the drive's own control even if the loop has hung.
- Before presenting the code, walk the Appendix A §B.3 pre-emit checklist item by item and fix anything that fails. Use only register addresses listed in the ConOps register reference — never invent addresses. Include a header comment recording the ConOps revision.

### Step 5 — Handover note
After the code, give a short summary: what to wire-check before power-on (especially the E-stop channels), which config-block values the user can edit without re-running you, and what to watch on the Serial monitor during first run.

## Anti-patterns (do not do these)
- Generating code first and asking questions after.
- Asking all ten topics in one giant message, or starting the questions without the roadmap.
- Assuming the user has read the ConOps, or quoting requirement IDs at them without explanation.
- Accepting "oscillate it" as a motion spec.
- Asking an open "any dwells?" question instead of offering the per-stroke / per-cycle / both options.
- Proceeding on your interpretation of an ambiguous answer without restating it and getting a yes/no.
- Guessing Modbus register addresses or serial settings.
- Inventing library API calls instead of using the Appendix A permitted list.
- Writing the sketch from scratch instead of adapting the bundled reference implementation.
- Config constants that nothing in the code reads (placebo knobs).
- Asking the user open technical questions about drive configuration instead of stating defaults with change-only-if guidance.
- Skipping, silently defaulting, or accepting a non-answer to the peak-current confirmation.
- Sharing one config constant between quantities the user might want to differ (e.g. a single dwell value for both stroke ends).
- Using FC 0x0F / `beginTransmission(..., COILS, ...)` for the stack light instead of sequential FC05 `coilWrite()` calls.
- Passing `0xFF00`/`0x0000` to `coilWrite()` instead of plain 1/0.
- Changing the relay module's baud rate, or assuming a baud change would persist.
- Driving stack-light writes every loop pass instead of on state change only.
- Treating a missing relay module as fatal (it is an optional extra; only the CL57RS is core hardware).
- Using `delay()` for blinks or dwells.
- Bypassing hardware enable with software-forced enable (register 0x000F).
