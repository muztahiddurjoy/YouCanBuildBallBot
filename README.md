# YouCanBuildBallBot — BallBot mini

A self-balancing **ball-balancing robot** (a "ballbot"): the robot balances on top of a single ball and drives around by rolling that ball with three omni wheels. It is controlled wirelessly from a matching hand-held ESP32 remote.

This repository contains everything needed to build one: the **firmware** for the robot and the remote, and the **CAD** for the 3D-printed mechanical parts.

> **Credit:** This is an open-source project by **James Bruton** (see [`LICENSE`](LICENSE), MIT). This README documents the code and hardware in this repository so you can build it end-to-end.

---

## Table of contents

1. [What it is & how it works](#1-what-it-is--how-it-works)
2. [Repository layout](#2-repository-layout)
3. [Control architecture](#3-control-architecture)
4. [Bill of materials](#4-bill-of-materials)
5. [Electronics & wiring](#5-electronics--wiring)
6. [Mechanical / CAD](#6-mechanical--cad)
7. [Software setup (Arduino IDE)](#7-software-setup-arduino-ide)
8. [Build & flash procedure](#8-build--flash-procedure)
9. [Operating the robot](#9-operating-the-robot)
10. [Tuning guide](#10-tuning-guide)
11. [Configuration reference](#11-configuration-reference-all-the-magic-numbers)
12. [Troubleshooting](#12-troubleshooting)
13. [Safety](#13-safety)
14. [License & credits](#14-license--credits)

---

## 1. What it is & how it works

A ballbot is a dynamically-stable robot: like a broom balanced on your palm, it is always falling and constantly correcting. Instead of a flat wheelbase, it sits on a single ball and stays upright by driving that ball in whatever direction it is tipping.

Three **omni wheels** press onto the ball, spaced around it (two at the front-left / front-right, one at the back). By spinning the wheels in a coordinated way (omni-wheel *mixing*), the controller can roll the ball in any horizontal direction — and rotate the robot in place — without the wheels fighting each other.

An **IMU** (inertial measurement unit) measures the robot's tilt (pitch and roll) 100 times a second. A cascade of **PID** controllers turns that tilt into wheel-speed commands that keep the robot balanced, while a wireless **remote** lets you nudge the balance point to make it travel.

**Key facts (from the firmware):**

| Property | Value |
|---|---|
| Controller | ESP32 (robot) + ESP32 (remote) |
| Wireless link | ESP-NOW (peer-to-peer, no Wi-Fi router needed) |
| Balance sensor | SparkFun BNO08x IMU (I²C / Qwiic) |
| Drive | 3 × DC gear motors with quadrature encoders + 3 × omni wheels |
| Control loop rate | 100 Hz (10 ms) |
| Control law | Cascaded PID: 2 balance loops (pitch/roll) → 3 wheel-velocity loops |

---

## 2. Repository layout

```
YouCanBuildBallBot/
├── CAD/
│   └── BallBot_mini.zip
│       ├── BallBot_mini.stp     # Full robot mechanical assembly (STEP)
│       └── ESP32_remote2.stp    # Hand-held remote enclosure (STEP)
├── Code/
│   ├── getMAC/
│   │   ├── getMAC.ino           # Utility: prints the ESP32 Wi-Fi MAC address
│   │   └── MAC.txt              # Recorded robot MAC → B0:CB:D8:E2:E3:C8
│   ├── Remote/
│   │   └── Remote.ino           # Transmitter firmware (joysticks + buttons)
│   └── Robot004/
│       └── Robot004.ino         # Robot firmware (balancing + drive)
├── LICENSE                      # MIT (© 2026 James Bruton)
└── README.md                    # This file
```

---

## 3. Control architecture

The robot firmware ([`Code/Robot004/Robot004.ino`](Code/Robot004/Robot004.ino)) runs **five PID controllers** in a cascade, all recomputed every 10 ms.

### 3.1 Outer loop — balancing (2 controllers)

The IMU reports **pitch** and **roll** in degrees. Two PID loops try to keep both at the setpoint (upright, plus any lean commanded by the remote / trim pots):

| PID | Axis | Input | Setpoint | Output |
|---|---|---|---|---|
| **PID4** | Sideways (X) | `-pitch` | `trimPot1 + stick(sideways)` | `velX` |
| **PID5** | Front/back (Y) | `roll` | `trimPot2 + stick(fwd/back)` | `velY` |

`velX` and `velY` are the desired ball velocities that will restore balance.

### 3.2 Inner loop — wheel velocity (3 controllers) + omni mixing

`velX`, `velY`, and a rotation term `rotVel` are mixed into a target speed for each wheel, and each wheel has its own PID that drives the motor to hit that speed using the encoder feedback:

```
Wheel 0 (left)  target = velY − velX
Wheel 1 (right) target = velY + velX
Wheel 2 (back)  target = velX / 1.5 − rotVel
```

Each wheel PID (PID1/PID2/PID3) compares its target against the measured encoder velocity (encoder counts per 10 ms) and outputs a PWM value in the range −255…255. The sign selects motor direction; the magnitude is the PWM duty applied to the motor driver.

### 3.3 Wireless remote → robot

The remote ([`Code/Remote/Remote.ino`](Code/Remote/Remote.ino)) reads two joystick potentiometers and two buttons and transmits them ~50×/second over **ESP-NOW** to the robot's MAC address. Both ends share the same packet layout:

```c
typedef struct struct_message {
  int a;   // joystick pot 1
  int b;   // joystick pot 2
  int c;   // button 1 (top)
  int d;   // button 2 (bottom)
} struct_message;
```

On the robot the raw stick values are **re-centred** (offset), passed through a **dead-band** (±50, so the robot ignores tiny stick noise), **scaled** to a lean angle, and **smoothed** with a low-pass filter before being fed into the balance setpoints.

---

## 4. Bill of materials

> The exact part numbers are not pinned in the repo (this is a reference design by James Bruton — match his build video / parts list for the precise components). The table below lists what the **firmware and CAD require**, with the electrical constraints taken directly from the code.

### Robot

| Qty | Item | Requirement / notes |
|----|------|---------------------|
| 1 | **ESP32 dev board** | Any common ESP32 (e.g. DevKitC / WROOM-32). Needs the pins listed in [§5](#5-electronics--wiring). |
| 1 | **SparkFun BNO08x IMU** | I²C / Qwiic breakout, address **0x4B**. Provides fused pitch/roll. |
| 3 | **DC gear motors w/ quadrature encoders** | Each motor: 2 encoder channels (A/B) + 2 motor terminals. Speed/torque to suit the robot's mass. |
| 3 | **Motor drivers (H-bridge)** | One channel per motor, driven **sign-magnitude** (two PWM inputs, one PWM'd, the other held low). E.g. TB6612FNG / DRV8871 / MX1508 / L298N-class. |
| 3 | **Omni wheels** | Mounted to the motor shafts, pressing on the ball at ~120° spacing. |
| 1 | **Ball** | The ball the robot balances on (sized to the CAD — "mini" build). |
| 2 | **Trim potentiometers** | Board-mounted, to fine-tune the balance point (pins 34 & 35). |
| 1 | **LiPo battery + regulation** | Sized for the motors; regulate 3.3 V/5 V rails for the ESP32 and logic as needed. |
| — | **3D-printed chassis** | From [`CAD/BallBot_mini.zip`](CAD/BallBot_mini.zip). |
| — | Wiring, JST/Dupont connectors, standoffs, fasteners | As required. |

### Remote

| Qty | Item | Requirement / notes |
|----|------|---------------------|
| 1 | **ESP32 dev board** | Transmitter. |
| 2 | **Potentiometers / joystick** | Two analog axes (pins 34 & 35). |
| 2 | **Momentary push buttons** | Wired to GND (firmware uses internal pull-ups on pins 32 & 33). |
| 1 | **Battery / USB power** | Any 3.3 V-capable supply for the ESP32. |
| — | **3D-printed enclosure** | `ESP32_remote2.stp` inside the CAD zip. |

---

## 5. Electronics & wiring

All pin numbers below are **ESP32 GPIO numbers** taken verbatim from the firmware.

### 5.1 Robot pinout ([`Robot004.ino`](Code/Robot004/Robot004.ino))

**IMU (BNO08x, I²C):**

| Signal | GPIO |
|---|---|
| SDA | 21 (default ESP32 I²C) |
| SCL | 22 (default ESP32 I²C) |
| INT | 19 |
| RST | 18 |
| I²C address | `0x4B` |

**Motor encoders (quadrature A/B):**

| Wheel | Channel A | Channel B |
|---|---|---|
| 0 — Left | GPIO 13 | GPIO 14 |
| 1 — Right | GPIO 4 | GPIO 16 |
| 2 — Back | GPIO 17 | GPIO 23 |

Both edges of both channels are handled by interrupts (`CHANGE`), giving full quadrature decoding.

**Motor driver outputs (sign-magnitude PWM pairs):**

| Wheel | Reverse pin | Forward pin |
|---|---|---|
| 0 — Left | GPIO 26 | GPIO 27 |
| 1 — Right | GPIO 25 | GPIO 33 |
| 2 — Back | GPIO 32 | GPIO 5 |

> To drive a wheel forward the firmware writes PWM to the *forward* pin and 0 to the *reverse* pin (and vice-versa). If a wheel spins the wrong way, swap that motor's two pins (or its two motor terminals).

**Board-mounted trim pots (analog in):**

| Function | GPIO |
|---|---|
| Trim pot 1 (sideways / pitch trim) | 35 |
| Trim pot 2 (front-back / roll trim) | 34 |

### 5.2 Remote pinout ([`Remote.ino`](Code/Remote/Remote.ino))

| Function | GPIO | Mode |
|---|---|---|
| Joystick pot 1 → packet `a` | 34 | analog in |
| Joystick pot 2 → packet `b` | 35 | analog in |
| Button 1 (top) → packet `c` | 33 | `INPUT_PULLUP` (button to GND) |
| Button 2 (bottom) → packet `d` | 32 | `INPUT_PULLUP` (button to GND) |

### 5.3 Wiring notes

- **Common ground:** the ESP32, motor drivers, IMU, and battery grounds must all be tied together.
- **Motor power vs. logic power:** power the motor drivers from the battery and the ESP32 from a regulated rail; do not back-feed motor voltage into the ESP32.
- **Pull-ups:** the remote buttons need no external resistors — the firmware enables internal pull-ups, so a button simply connects its pin to GND (pressed = `0`).
- **PWM pins:** the six motor pins use the ESP32's LEDC/`analogWrite` PWM. Keep them on PWM-capable GPIOs (the ones listed above are).
- **Input-only pins:** GPIO 34 & 35 are input-only on the ESP32 — correct for the analog trim pots / joysticks, never use them as outputs.

---

## 6. Mechanical / CAD

The 3D model files live in [`CAD/BallBot_mini.zip`](CAD/BallBot_mini.zip):

| File | Description |
|---|---|
| `BallBot_mini.stp` | Full robot assembly — chassis, motor mounts, wheel/ball geometry. |
| `ESP32_remote2.stp` | Hand-held remote enclosure. |

Both are **STEP (`.stp`)** files, openable in Fusion 360, FreeCAD, SolidWorks, Onshape, or any STEP-compatible CAD. Export the individual parts to STL and slice them for your printer. The three motors mount so their omni wheels contact the ball at even spacing; keep that geometry when adjusting for your specific motors/wheels.

---

## 7. Software setup (Arduino IDE)

### 7.1 Install the ESP32 core

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. **File → Preferences → Additional Board Manager URLs**, add:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. **Tools → Board → Boards Manager**, search **esp32** and install the Espressif package (use a recent v3.x — the firmware uses `analogWrite()`, which requires a modern core).
4. Select your board under **Tools → Board → ESP32 Arduino** (e.g. *ESP32 Dev Module*).

### 7.2 Install libraries

Install via **Sketch → Include Library → Manage Libraries…**:

| Library | Used by | Notes |
|---|---|---|
| **PID** (by Brett Beauregard, `PID_v1`) | Robot | The five PID controllers. |
| **ESP32Servo** | Robot | Included by the sketch. |
| **SparkFun BNO08x Arduino Library** | Robot | IMU driver. |
| `esp_now`, `WiFi`, `Wire` | Both | Bundled with the ESP32 core — no separate install. |

---

## 8. Build & flash procedure

The remote and robot are paired by **MAC address**, so flash them in this order.

### Step 1 — Get the robot's MAC address
1. Open [`Code/getMAC/getMAC.ino`](Code/getMAC/getMAC.ino) and upload it to the **robot's** ESP32.
2. Open **Serial Monitor at 115200 baud**. It prints the MAC every 5 s, e.g. `B0:CB:D8:E2:E3:C8`.
3. Record it. (This repo's example robot MAC is `B0:CB:D8:E2:E3:C8`, saved in [`Code/getMAC/MAC.txt`](Code/getMAC/MAC.txt).)

### Step 2 — Flash the remote with that MAC
1. Open [`Code/Remote/Remote.ino`](Code/Remote/Remote.ino).
2. Set `broadcastAddress[]` to **your robot's** MAC:
   ```c
   uint8_t broadcastAddress[] = {0xB0, 0xCB, 0xD8, 0xE2, 0xE3, 0xC8}; // ← your robot's MAC
   ```
3. Upload to the **remote's** ESP32.

### Step 3 — Flash the robot
1. Open [`Code/Robot004/Robot004.ino`](Code/Robot004/Robot004.ino).
2. Upload to the **robot's** ESP32 (the same board whose MAC you recorded).

### Step 4 — Calibrate joystick centres
The robot subtracts a fixed offset so a stationary stick reads zero (`pot2 = a − 1919`, `pot1 = b − 1950`). Your pots will differ:
1. In `Robot004.ino`, un-comment the debug block inside `OnDataRecv()` that prints `myData.a` and `myData.b`.
2. Upload, open Serial Monitor, and read the values with the sticks centred.
3. Replace `1919` and `1950` with your centred readings, re-comment the block, and re-upload.

### Step 5 — First power-up
Hold the robot upright (or bench-test with wheels off the ball) and follow [§9](#9-operating-the-robot) and the [tuning guide](#10-tuning-guide). Expect to tune PID and trim before it balances cleanly.

---

## 9. Operating the robot

**Arming / disarming (latching):**

| Button | GPIO | Action |
|---|---|---|
| **Top (button 1)** | 33 | **Enable motors** (arm) |
| **Bottom (button 2)** | 32 | **Disable motors** (stop) — motors are cut immediately |

The motor-enable state latches: press top to arm, press bottom to kill. Always start disarmed and keep a thumb on the bottom button as a kill switch.

**Driving:**

| Stick | Effect |
|---|---|
| Front/back axis (pot 1) | Lean the balance point forward/back → robot drives forward/back. |
| Left/right axis (pot 2) | Lean sideways → robot strafes left/right. |
| Left/right axis **while holding the top button** | Switches to **rotate-in-place** (yaw) instead of strafing. |

Both stick axes have a **±50 dead-band**, so small stick offsets are ignored and the robot holds position when the sticks are centred.

---

## 10. Tuning guide

All tuning constants live at the top of / inside [`Robot004.ino`](Code/Robot004/Robot004.ino).

### 10.1 PID gains

```c
// Wheel-velocity loops (inner) — PID1/2/3
Pk = 3.5,  Ik = 0.5,  Dk = 0.05

// Balance loops (outer) — PID4 (pitch/sideways), PID5 (roll/front-back)
Pk = 19,   Ik = 150,  Dk = 0.3
```

- Tune the **inner wheel loops first** (with the robot held so it can't fall): a wheel should reach and hold commanded speed without buzzing or lag.
- Then tune the **balance loops**: raise `Pk` until it holds upright but starts to oscillate, back it off; add `Dk` to damp; use `Ik` to remove steady lean/drift. Both balance axes use identical gains — keep them matched unless your geometry is asymmetric.
- Output limits are `±255` and the loops run at 10 ms (100 Hz) — do not change these without reason.

### 10.2 Trim pots (balance-point trim)

The two board-mounted pots bias the upright setpoint so the robot doesn't drift when the sticks are centred:

```c
trimPot1Scaled = (analogRead(35) - 2000) / 250;   // sideways trim
trimPot2Scaled = (analogRead(34) - 2000) / 250;   // front/back trim
```

Adjust them live until the robot holds still. Change the `2000` centre or `250` divisor if the pots don't reach neutral or feel too coarse/fine.

### 10.3 Stick feel

| Constant | Meaning | Where |
|---|---|---|
| `± 50` | Stick dead-band | `OnDataRecv()` |
| `/ 500` | Stick → lean-angle scale (smaller = more aggressive) | `pot1Scaled`, `pot2Scaled` |
| `/ 20` | Stick → rotation-rate scale | `rotVel` |
| `filter(..., 20)` | Stick smoothing (higher = snappier, lower = smoother) | `filter()` |

---

## 11. Configuration reference (all the "magic numbers")

| Value | Location | Meaning |
|---|---|---|
| `115200` | `Serial.begin` | Serial baud (all three sketches). |
| `10` (ms) | main loop / `SetSampleTime` | Control-loop period → 100 Hz. |
| `20` (ms) | `Remote.ino` `delay()` | Remote transmit period → ~50 Hz. |
| `0x4B` | `BNO08X_ADDR` | IMU I²C address. |
| `1919`, `1950` | `OnDataRecv()` | Joystick centre offsets — **calibrate per build**. |
| `± 50` | `OnDataRecv()` | Stick dead-band. |
| `2000`, `250` | trim-pot scaling | Trim-pot centre & sensitivity. |
| `velX / 1.5` | back-wheel mixing | Omni-wheel geometry factor for the rear wheel. |
| `± 255` | `SetOutputLimits` | PWM output range. |

---

## 12. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `BNO08x not detected... Freezing...` on serial | IMU wiring/address. Check SDA/SCL/INT(19)/RST(18) and that the board is at `0x4B`. |
| `Error initializing ESP-NOW` | ESP-NOW/Wi-Fi init failed — re-flash; ensure `WiFi.mode(WIFI_STA)` runs. |
| Remote prints `Delivery Fail` | Wrong `broadcastAddress` (must be the robot's MAC), robot not powered, or out of range. |
| Robot drifts with sticks centred | Re-run the joystick-centre calibration ([Step 4](#step-4--calibrate-joystick-centres)) and/or adjust the trim pots. |
| A wheel spins the wrong way | Swap that motor's two driver pins (or its two motor terminals). |
| Encoder counts don't change / count backwards | Check encoder A/B wiring for that wheel; swapping A/B inverts the counting direction. |
| Robot leans and runs away instantly | Balance PID sign/gains or IMU orientation. Verify `pitch`/`roll` change in the expected direction (use the commented `Serial.print` debug in `loop()`). |
| Motors never move | Not armed — press the **top** button; also confirm `motorEnable` logic and driver power. |

> The robot sketch contains several commented-out `Serial.print` blocks (encoder pins, encoder velocities, stick/IMU/trim values). Un-comment them one at a time to inspect each subsystem while bringing the robot up.

---

## 13. Safety

- The wheels have real torque — **always bring the robot up on a stand / with wheels off the ball first**, and keep clear of pinch points.
- Keep the **bottom button (kill switch)** under your thumb whenever the robot is armed.
- Observe LiPo safety: correct charger, don't over-discharge, don't short the motor rail.
- Test PID changes at low power / on a stand before free balancing.

---

## 14. License & credits

- **License:** MIT — see [`LICENSE`](LICENSE). Copyright © 2026 **James Bruton**.
- **ESP-NOW transmit/receive** structure adapted from **Rui Santos / Random Nerd Tutorials** — <https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/>.
- **PID** library by Brett Beauregard; **BNO08x** library by SparkFun.

Contributions and remixes welcome under the MIT terms.
