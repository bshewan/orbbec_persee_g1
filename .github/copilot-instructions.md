# Copilot Instructions

## Project Overview

This is a C++ application for the **Orbbec Persee / Astra** depth camera family. It demonstrates hardware integration via OpenNI 2 for stream access and raw USB control transfers for vendor-specific hardware commands not covered by the OpenNI API (emitter, flood LED, LDP, I2C registers). An OpenGL/GLUT viewer displays depth/IR/RGB streams and runs real-time gesture analysis on the depth feed.

## Build

### Windows

Open `Platform\Windows\Orbbecintegratesimple.vcxproj` in **Visual Studio 2013** (toolset v120, Win32 target). Build via the IDE or:

```bat
msbuild Platform\Windows\Orbbecintegratesimple.vcxproj /p:Configuration=Debug /p:Platform=Win32
```

The post-build step copies OpenNI2 and GL redistributables into the output directory automatically.

### Linux (ARM / x64 / x86)

```bash
cd Platform/Linux(ARM)   # or Linux(x64) / Linux(x86)
make
./orbbecinteragesimple_rev1.0
```

`make clean` removes the binary and object files. The makefile copies OpenNI2 Redist files next to the binary after a successful build.

**Required system packages:** `libusb-1.0`, `libGL`, `libGLU`, `libglut` (freeglut).

**Compiler flags:** `-std=c++11 -DLINUX -fpermissive`

## Architecture

The application has three layers, each owning a distinct concern:

```
main.cpp
  └── cmd (hardware layer)       – cmd.h / cmd.cpp
  └── SimpleViewer (UI layer)    – SimpleViewer.h / SimpleViewer.cpp
```

### `cmd` — Hardware / USB layer

Owns the `openni::Device` handle and all vendor-specific communication. Every control transfer starts with an 8-byte `protocol_header` (magic `0x4D47`, size in 16-bit words, opcode from `EPsProtocolOpCodes`, auto-incrementing sequence ID). USB backend is **XnUSB** on Windows and **libusb** on Linux — guarded by `#ifdef WIN32` / `#ifdef LINUX`.

`cmd::init()` detects the device PID (`m_pid`) to distinguish Astra Pro (PID `0x0403`) from other models.

### `SimpleViewer` — Visualization / Gesture layer

GLUT requires free-function callbacks, so `SimpleViewer` uses a **static singleton** (`m_self`) to dispatch `glutDisplay`, `glutIdle`, and `glutKeyboard` back to the instance.

Frame pipeline: `display()` → `loadFrameToTexture()` → `loadDepthFrame()` / `loadIRFrame()` / `loadCOLORFrame()`. On every depth frame, `analyzeGestures()` also runs.

OpenGL textures must be **power-of-2** dimensions; the `MIN_CHUNKS_SIZE` macro enforces this, so the actual texture buffer (`m_pTexMap`) is always larger than the 640×480 frame.

### `main.cpp` — Entry point / wiring

Calls `cmd.init()`, creates the three `openni::VideoStream` objects, constructs `SimpleViewer`, sets `mSimpleViewer.m_cmd = &cmd`, then calls `init()` + `run()`. The `cmd` global object is declared at file scope.

## Key Conventions

### Astra Pro special case

Astra Pro (`m_pid == Astra_Pro` / `0x0403`) does **not** expose its RGB sensor through OpenNI — it uses UVC. The RGB stream creation is skipped for this model and the keyboard `'2'` key prints a notice instead of switching to the RGB stream. Always guard RGB-specific code with this check.

### Gesture debouncing

`analyzeGestures()` uses frame counters (`m_framesJump`, `m_framesDuck`, `m_framesWalk`). A gesture string is only committed to `m_gestureText` after **5 consecutive frames** in the same state. Incrementing one counter must reset the others.

### USB packet sequence IDs

`cmd::seq_num` is a `uint16_t` that increments on every `send()` call. The same sequence ID must appear in both the request header and be matched in the response for correct pairing.

### IR frame bit-depth

IR frames arrive as 16-bit grayscale (`PIXEL_FORMAT_GRAY16`) with only the lower ~10 bits active. `loadIRFrame()` shifts right by 2 (`>> 2`) to map into 8-bit range. Do not change this without understanding the sensor's active bit depth.

### Depth histogram visualization

`calculateHistogram()` builds a **cumulative** histogram so that the grayscale mapping spreads depth variation evenly across the visible range (closer = brighter). The histogram array is `m_pDepthHist[MAX_DEPTH]` where `MAX_DEPTH = 10000`.

### Third-party libraries (vendored)

All dependencies are vendored under `ThirdParty/`:
- `OpenNI2/` — SDK headers and libs
- `GL/` — GLUT/OpenGL headers and libs  
- `LibUSB/` — libusb headers (Linux)
- `XnLib/` — XnUSB and XnPlatform (Windows)

Do not update these independently; they are matched to the Orbbec firmware.
