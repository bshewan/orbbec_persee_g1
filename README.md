# orbbec_persee_g1

A C++11 application for the **Orbbec Persee** (ARM Linux) that visualises Depth/IR/RGB streams from an Orbbec Astra camera and performs real-time heuristic gesture analysis for treadmill control. Detected gestures — **Jump**, **Duck**, and **Walk** — are shown as an on-screen overlay.

---

## Building on the Persee

The Persee is an ARM Linux device with the Orbbec Astra camera built in. All required libraries (OpenNI 2, LibUSB, OpenGL/GLUT) are bundled in `ThirdParty/`.

### Prerequisites

Install the system packages needed for linking:

```sh
sudo apt-get update
sudo apt-get install -y libusb-1.0-0-dev freeglut3-dev
```

### Compile

```sh
cd Platform/Linux\(ARM\)
make
```

The binary `orbbecinteragesimple_rev1.0` is produced in the same directory, and the OpenNI2 redist libraries are copied next to it automatically.

### Clean

```sh
make clean
```

### Run

```sh
cd Platform/Linux\(ARM\)
./orbbecinteragesimple_rev1.0
```

> **Note:** The application must be run from the `Platform/Linux(ARM)/` directory so it can find the OpenNI2 shared libraries at the relative `rpath` (`.`).

---

## Camera Mounting

The sensor must be mounted so that **only the rider's upper body is visible** — roughly from the elbows upward:

- **Height:** Position the camera at approximately **chest height** of the seated/standing rider (roughly 1.0–1.4 m from the floor depending on treadmill height).
- **Distance:** The rider should stand **0.8–1.2 m** from the sensor face. The active detection zone is 400 mm – 1200 mm; pixels outside this range are ignored.
- **Angle:** Mount the sensor **level and facing directly toward the rider** (no tilt). The rider should be centred in the frame.
- **Field of view:** The bottom edge of the frame should be at roughly **elbow height** when the rider walks normally with arms bent at 90°. This is intentional — it allows forearm detection for Walk recognition (see below).

A desktop/autostart launcher (`orbbec-camera.desktop`) is included for convenience.

---

## Gesture Reference

Gesture analysis runs on every depth frame. The algorithm uses **Vertical Zone Analysis** — it segments the user from the background (depth 400–1200 mm) and examines where pixels are distributed within the bounding box.

A gesture requires **5 consecutive frames** before it is reported (debounce), so brief noise will not trigger a false detection.

| Gesture | How to perform it | How it is detected |
|---|---|---|
| **WALKING** | Walk normally on the treadmill, arms bent at ~90°, forearms swinging toward the camera. | Forearms enter the "close zone" (< 650 mm). When the fraction of close-zone pixels exceeds 10% of total visible pixels (`close_ratio > 0.10`), WALKING is declared. |
| **JUMP** | Raise both arms above your head. | Arms move away from the camera (close_ratio drops) and shift to the **top half** of the bounding box. Triggered when `topHalfCount > botHalfCount × 0.30`. |
| **DUCK** | Lower your body/arms below the camera's field of view (duck down or drop arms to sides). | Total visible pixel count drops sharply. When no forearm blob is detected and the top half is not dominant, DUCK is declared. |

The live HUD (top-left of the window) shows:
- `ACTION:` — current gesture state (`WALKING`, `JUMP`, `DUCK`, `NO USER`, or `WAITING`)
- `close_ratio` — fraction of pixels in the forearm zone (key Walk signal)
- `top/bot` ratio — used to distinguish Jump from Duck

---

## Keyboard Controls

| Key | Action |
|---|---|
| `1` | Switch to Depth stream |
| `2` | Switch to RGB stream |
| `3` | Switch to IR stream |
| `l` / `L` | Emitter ON / OFF |
| `d` / `D` | IR Flood LED ON / OFF |
| `b` / `B` | LDP ON / OFF |
| `r` / `R` | Increase / Decrease IR Exposure |
| `o` / `O` | Increase / Decrease IR Gain |
| `f` / `v` | Print firmware version / serial number |
| `Esc` | Exit |

---

## Notes

- **Astra Pro (PID 0x0403):** The RGB sensor on Astra Pro models uses UVC and is not accessible through OpenNI. The application detects this automatically and skips the OpenNI colour stream; the `2` key will show a blank frame.
- The depth visualisation window only renders pixels within the 400–1200 mm gesture zone (everything else is black), so the display matches exactly what the gesture algorithm evaluates.
