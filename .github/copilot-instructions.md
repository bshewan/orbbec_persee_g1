# Copilot Instructions for orbbec_persee_g1

## Project Overview

A C++11 application that integrates with Orbbec depth cameras (Astra series) via OpenNI 2 and raw USB control transfers. The primary purpose is visualizing Depth/IR/RGB streams and performing heuristic gesture analysis (Jump, Duck, Walk, Dodge) for treadmill control using vertical-zone analysis on depth data.

## Architecture

The application has three layers:

1. **Hardware layer (`cmd.h` / `cmd.cpp`)** — Wraps OpenNI device management and adds Orbbec-specific vendor commands via raw USB control transfers. On Windows it uses `XnUSB` (from `XnLib`); on Linux it uses `libusb-1.0`. All USB packets start with the 8-byte `protocol_header` (magic `0x4d47`, size, opcode, sequence ID). Opcodes are defined in `EPsProtocolOpCodes`.

2. **Visualization + Gesture layer (`SimpleViewer.h` / `SimpleViewer.cpp`)** — Drives the GLUT/OpenGL window. Uses a static singleton (`m_self`) to bridge GLUT C callbacks to the class. Converts OpenNI `VideoFrameRef` frames into an `RGB888Pixel` texture buffer (`m_pTexMap`) — texture dimensions must be powers of 512. Gesture analysis runs on every depth frame only (not IR or RGB).

3. **Entry point (`main.cpp`)** — Initializes `cmd`, creates OpenNI streams, conditionally skips RGB stream for Astra Pro (PID `0x0403`, which uses UVC instead of OpenNI), then hands off to `SimpleViewer`.

## Build

### Linux (ARM or x64)

```sh
# From the platform directory (e.g., Platform/Linux(ARM)/ or Platform/Linux(x64)/)
make

# Clean
make clean
```

The binary is output as `orbbecinteragesimple_rev1.0` in the same directory. OpenNI2 redist libs are automatically copied next to it.

### Windows

Open `Platform\Windows\Orbbecintegratesimple.vcxproj` in Visual Studio 2013 (toolset `v120`). Build configurations: **Debug|Win32** and **Release|Win32**. Post-build steps copy OpenNI2 and GL redist DLLs to the output directory.

## Key Conventions

### Platform-conditional USB code
The `cmd` class uses `#ifdef WIN32` / `#ifdef LINUX` to switch between the XnUSB API and libusb. When adding new hardware commands, implement both paths and keep them symmetric.

### Astra Pro RGB special case
`m_pid == Astra_Pro` (`0x0403`) means the RGB sensor is UVC-only — `createRGBStream()` is skipped entirely. This check appears in both `main.cpp` and `SimpleViewer`. Do not try to open an OpenNI color stream for this model.

### USB packet protocol
Every command to the hardware goes through `cmd::send()`. Build the request by calling `cmd::init_header()` first (fills `req_buf` with magic, size, opcode, and auto-incrementing `seq_num`), then append any payload after the 8-byte header before calling `send()`.

### Gesture analysis depth range
The user zone is hard-coded as 800 mm – 2000 mm. Pixels outside this range are ignored for gesture detection. The debounce counters (`m_framesJump`, `m_framesDuck`, etc.) must reach a threshold before a gesture is reported to avoid single-frame noise.

### Depth histogram visualization
`calculateHistogram()` maps non-linear 16-bit depth values (0–10000 mm) into a cumulative 8-bit grayscale range. The histogram array (`m_pDepthHist[MAX_DEPTH]`) is recomputed every frame before `loadDepthFrame()` renders it.

### Texture sizing
OpenGL texture dimensions are rounded up to the nearest multiple of `TEXTURE_SIZE` (512) using the `MIN_CHUNKS_SIZE` macro. `m_pTexMap` is reallocated whenever the stream resolution changes.

## Third-Party Dependencies (bundled in `ThirdParty/`)

| Library | Path | Notes |
|---|---|---|
| OpenNI 2 | `ThirdParty/OpenNI2/` | Headers, libs, and platform redist |
| XnLib | `ThirdParty/XnLib/` | Windows USB layer (`XnUSB`) |
| LibUSB | `ThirdParty/LibUSB/` | Linux USB layer (libusb-1.0) |
| OpenGL/GLUT | `ThirdParty/GL/` | For visualization |
