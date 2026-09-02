# Stack Light Relay Board — Commissioning Procedure

**Hardware:** Waveshare Modbus RTU Relay 4CH
**Companion sketch:** `RelayBoard_Commissioning.ino`
**Author:** Ross
**Rev:** - (initial)

Run every step on the **bench, point-to-point** — Opta RS-485 A/B wired
directly to the relay board's A/B, nothing else on the bus. Do not join
this board to the shared CL57RS trunk until Step 7 passes.

---

## 1. Power-up check

- Power the relay board from a bench 24V supply (7–36V accepted) — not
  yet from the HDR-60-24 logic bus, so a wiring fault here can't touch
  the rest of the fixture.
- Confirm the **STA** LED is flashing (normal MCU operation). If it's
  solid on/off, stop — that's a board fault, not a wiring issue.

## 2. Comms check

- Upload `RelayBoard_Commissioning.ino` to the Opta, leaving
  `relaySlaveAddr = 1` (factory default is commonly 1, but treat this
  as unverified until Step 3 confirms it).
- Open Serial Monitor, 115200 baud, Newline line ending.
- Run `VER`. Expect a version string back (e.g. `V3.00`). A timeout
  here means RS-485 wiring (A/B swapped is the usual culprit) or baud
  mismatch — board default is 9600,N,8,1.

## 3. Confirm the board's address

- Run `IDENTIFY` (broadcast read — safe now since it's the only device
  on the bus).
- Note the address returned. This is the board's *actual* current
  address, which may not be what you assumed.
- If it isn't `1`, either:
  - update `relaySlaveAddr` in the sketch to match and re-upload, or
  - run `SETADDR <n>` to move the board to the address you want it to
    live at once it joins the shared trunk (see Step 8 note below).
- Run `ADDR` afterwards to confirm the board now answers on
  `relaySlaveAddr` as configured.

## 4. Individual channel test

- Run `GREEN ON`, then `STATUS` — confirm channel 1 reads back ON and
  you hear/see the relay physically click over.
- Repeat for `AMBER`, `RED`, `SPARE`, each ON then OFF, checking
  `STATUS` after every change.
- This confirms both directions of the coil interface (write and
  read-back) on all four channels before any light is wired.

## 5. Walking test

- Run `WALK`. Expect channels to energise 1→2→3→4 in sequence
  (400 ms apart), then de-energise in the same order.
- Watch the board's own channel LEDs for this — no light needs to be
  wired yet. This is the same behaviour as Waveshare's factory demo,
  so it's a good independent check that the board itself is healthy
  before you introduce your own wiring or logic.

## 6. Wire the tower light, one colour at a time

- Power down. Wire the 24VDC tower light's Green segment to Channel 1's
  NO contact and the light's common return, per the wiring note from
  our last session (24V+ through NO contact, common 24V− to the
  light's common terminal). Fuse the shared 24V+ feed.
- Power up, run `GREEN ON` / `GREEN OFF`, confirm the correct segment
  lights.
- Repeat for Amber → Channel 2 and Red → Channel 3, confirming each
  segment against its intended channel before moving to the next —
  this catches a swapped wire immediately rather than after all three
  are landed.

## 7. Pattern test (production logic)

- Run `RUN` — expect Green on, Amber/Red off.
- Run `PAUSE` — expect Amber on, Green/Red off.
- Run `FAULT` — expect Red on, Green/Amber off.
- Run `ALLOFF` — expect all off (COMPLETE state).
- Confirm each transition is clean — no flicker of a wrong colour
  between states. This exercises the same atomic multi-coil write
  (`FC 0x0F`) the production sketch will use at every state change.

## 8. Move onto the shared trunk

- Power down everything.
- Confirm the relay board's address (from Step 3) does **not** collide
  with the CL57RS's configured Modbus address — check the drive's
  MotionStudio settings or your existing ConOps record of it.
- Land the relay board's A/B onto the shared signal terminal blocks
  alongside the Opta and CL57RS, move its 24V feed onto the HDR-60-24
  logic bus, and re-run Steps 2–4 (`VER`, `ADDR`, `STATUS`) to confirm
  it still answers correctly with the CL57RS also present on the bus.

---

## Findings log

*(Fill in as you go — mirrors the format of the alarm-code and USB
dropout findings already in the ConOps.)*

| Step | Result | Notes |
|------|--------|-------|
|      |        |       |
