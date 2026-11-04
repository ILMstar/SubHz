# SubHz 🎸🔊

Real-time bass guitar audio visualizer and chromatic tuner written in **C++20** using **Raylib** and **miniaudio**.

---

## Features

- **Low-Latency Audio Capture**: Direct microphone/audio interface PCM stream (48 kHz, 32-bit float, mono) captured via `miniaudio` into a thread-safe circular ring buffer (4096 samples).
- **Time-Domain Oscilloscope**: Continuous waveform polyline rendered at 180 FPS with 4x MSAA anti-aliasing.
- **Logarithmic Frequency Spectrum**:
  - Focuses heavily on the fundamental and harmonic range of bass guitars (**20 Hz – 2000 Hz** covers ~74% of visual width; range extends to 10 kHz).
  - Open-string reference markers (**B0**, **E1**, **A1**, **D2**, **G2**).
  - Pre-FFT **Hann window** to eliminate spectral leakage.
  - In-place, custom **Cooley-Tukey Radix-2 FFT** (zero heap allocations per frame).
  - **Temporal smoothing** using `std::lerp` to eliminate jitter while maintaining responsive dynamics.
- **High-Precision Chromatic Bass Tuner**:
  - Targeted peak detection strictly within the bass fundamental band (**30 Hz – 400 Hz**).
  - **Noise gate** prevents erratic jumping during idle room silence.
  - **Hann-calibrated parabolic interpolation** overcomes the ~11.7 Hz FFT bin spacing to deliver sub-cent pitch accuracy even down to low B0 (30.87 Hz).
  - Dynamic cents tuning dial (-50 to +50 cents) with color feedback (bright green for $\pm 5$ cents in-tune, orange/red when out of tune).
- **Input Gain & Metering**:
  - Real-time digital gain adjustment with dB readout.
  - Live RMS level meter (linear + dBFS) and visual clipping indicator (`CLIP`).

---

## Controls

| Key | Action |
|---|---|
| **Up Arrow** | Increase input gain (+1 dB) |
| **Down Arrow** | Decrease input gain (-1 dB) |
| **R** | Reset input gain to 1.0x (0 dB) |
| **Escape** / **Close** | Exit application |

---

## Prerequisites (Windows / MSYS2 UCRT64)

1. **MSYS2**: Install [MSYS2](https://www.msys2.org/).
2. Open the **MSYS2 UCRT64** terminal and install the toolchain and dependencies:
   ```bash
   pacman -S mingw-w64-ucrt-x86_64-gcc \
             mingw-w64-ucrt-x86_64-raylib \
             mingw-w64-ucrt-x86_64-glfw
   ```

---

## Building & Running

### Using MSYS2 UCRT64 Terminal:
```bash
cd /c/unibuc/proiecte/SubHz
g++ -std=c++20 -O2 -Wall -Wextra main.cpp -o SubHz.exe \
  -static-libgcc -static-libstdc++ -Wl,-Bstatic -lwinpthread -Wl,-Bdynamic \
  -lraylib -lopengl32 -lgdi32 -lwinmm
./SubHz.exe
```

### Running outside MSYS2:
Ensure `libraylib.dll` and `glfw3.dll` (from `C:\msys64\ucrt64\bin`) reside in the same folder as `SubHz.exe`, or have `C:\msys64\ucrt64\bin` added to your system `PATH`.

