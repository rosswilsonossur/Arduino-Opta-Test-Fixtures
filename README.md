# Arduino Opta Test Fixture

Endurance-cycling rig. An Arduino Opta WiFi talks Modbus RTU over RS-485 to a
CL57RS stepper drive, and to a Waveshare relay module that drives a stack light.

## Run the programme

Sketch: `programmes/GTF_135deg_9000cyc/GTF_135deg_9000cyc.ino`
(135° out and back at 23 rpm, 2 s hold after each return, 9,000 cycles, about 10 h)

1. Get the repo: **Code → Download ZIP** on GitHub, or `git clone`.
2. Install the Arduino IDE, then:
   - Boards Manager: **Arduino Mbed OS Opta Boards**
   - Library Manager: **ArduinoRS485** and **ArduinoModbus**
3. Open the `.ino` from inside its own folder (the IDE needs the folder name to match).
4. Select the Opta board and its USB port, then Upload.
5. Optional: Serial Monitor at 115200 shows boot checks and fault reasons.

Tested with: Opta core `<fill in>`, ArduinoModbus `<fill in>`, ArduinoRS485 `<fill in>`

## Hardware must already be set up

- Bus: RS-485, 9600 baud, 8N1
- CL57RS drive: Modbus ID 1 (DIP switches), SW6/SW7 off for 9600
- Relay module (stack light): address 2. Optional, the programme runs without it.
- Panel buttons, normally open: Start on I3, Pause on I4, Stop on I5
- E-stop: wired on both channels, I1 and the drive's DI1

The motor moves with no homing. Read the safety sections of `docs/CONOPS.md`
before first power-on.

## Make a new programme with a chatbot

Start a new chat. In the first message, attach or paste these three files in full:

1. `docs/CONOPS.md`
2. `skill/grill-the-user/SKILL.md`
3. `skill/grill-the-user/references/reference-implementation.ino`

Then type: **I want to make a new programme**

The AI asks 11 questions one at a time, then shows a confirmation table.
Check the numbers against what you meant before confirming. After you confirm
it generates the sketch.

Before flashing:
- It should have changed only the config block and header of the reference sketch. If it wrote the code from scratch, reject it.
- Compile it in the Arduino IDE.
- Save it as `programmes/<name>/<name>.ino` (folder name must match the file name).

Use a new chat for each programme.

## Other folders (not needed to run a programme)

| Folder | Purpose |
|---|---|
| `commissioning/` | Hardware bring-up and one-time setup sketches |
| `tools/` | Browser-based RS-485 tester |
