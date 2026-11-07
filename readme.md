# SubHz 🎸🔊

Real-time full-duplex audio rig, spectrum visualizer and chromatic tuner (bass **and** voice) written in **C++17** using **Raylib** and **miniaudio** (Windows / WASAPI).

---

## Features

- **Full-Duplex WASAPI Audio** (48 kHz, 32-bit float, 2 ch in / 2 ch out, dual-mono output):
  - Opens **Shared mode first** so the laptop mic-array drivers (beamforming, mic boost, noise/echo suppression) stay active; falls back to **Exclusive** mode only if Shared fails.
  - Input selector: **IN 1+2** (sum, default), **IN 1**, **IN 2**.
  - Soft-knee `tanh` safety limiter (ceiling ±0.98) and a click-free mute ramp.
- **Lock-Free Dual-Tap Routing** (`LockFreeHistoryBuffer<4096>`, no mutexes):
  - **Dry tap** (DC-blocked, pre-FX) → tuner.
  - **Wet tap** (post-FX, post-limiter) → oscilloscope + FFT spectrum.
- **Modular DSP Chain** (`IDspNode`): `HPF 28 Hz → Noise Gate → Cab Sim → Limiter`
  - `DcBlockerNode`: 28 Hz DC-blocking high-pass.
  - `EnvelopeNoiseGateNode`: 1 ms attack / 80 ms release with hysteresis (open −54 dBFS / close −62 dBFS). Sits **before** the cab filter so high-frequency consonants still open it. **Off by default.**
  - `CabSimNode`: 2nd-order Butterworth low-pass at 4.5 kHz with 10 ms click-free crossfade. **Off by default** (keeps voice full-range).
- **Time-Domain Oscilloscope** at 180 FPS with 4x MSAA.
- **Logarithmic Frequency Spectrum** (20 Hz – 10 kHz, ~74% of the width devoted to 20 Hz – 2 kHz):
  - Pre-FFT **Hann window**, custom in-place **Cooley-Tukey radix-2 FFT**, temporal smoothing (lerp 0.15).
  - Open-string reference markers (B0, E1, A1, D2, G2).
- **Chromatic Tuner (30 Hz – 1200 Hz, bass through soprano voice)**:
  - Local-peak detection with a −54 dBFS noise gate.
  - **Hann-calibrated parabolic interpolation** for sub-cent accuracy (≈1.5 ct worst case, down to B0 at 30.87 Hz).
  - **Subharmonic lock**: if a peak exists near f0 = peak/2..4 with ≥ 25% of the magnitude, the tuner locks onto the true fundamental instead of jumping octaves on louder harmonics (vowel formants, bass pickups).
  - Cents dial (−50…+50) — green within ±5 cents, orange/red otherwise.
- **Studio HUD**: driver mode (`WASAPI SHARED` / `EXCLUSIVE`), buffer latency, input channel, `GATE` / `CAB` / `MUTED` chips, input/output peak meters, `LIMIT` LED and gate LED.

---

## Controls

| Key | Action | Default |
|---|---|---|
| **Space** | Toggle live speaker monitoring (`MUTED` / `LIVE`) | `MUTED` |
| **G** | Toggle noise gate | `OFF` |
| **C** | Toggle 4.5 kHz cab-sim low-pass | `OFF` |
| **I** | Cycle input: `IN 1+2` → `IN 1` → `IN 2` | `IN 1+2` |
| **Up / Down** | Input trim +1 / −1 dB | 0 dB |
| **R** | Reset trim to 0 dB | |
| **Escape** / **Close** | Exit | |

> When unmuted on laptop speakers, keep the volume moderate (or use headphones) to avoid mic feedback.

---

## Changelog (voice-quality fix)

The full-duplex refactor made voice capture sound thin and drop quiet sounds. Causes and fixes:

| Problem | Fix |
|---|---|
| Exclusive mode bypassed the mic-array drivers (no beamforming/boost) and read a single raw capsule | Shared mode first, Exclusive as fallback; default input is `IN 1+2` |
| Noise gate (−50/−58 dBFS) could not be disabled and ran *after* the 4.5 kHz low-pass, chopping soft syllables and consonants | Gate is toggleable (`G`), off by default, thresholds lowered to −54/−62 dBFS, moved before the cab filter |
| 4.5 kHz cab low-pass was always on, muffling voice | Cab sim off by default (`C` to enable) |
| Tuner limited to 30–400 Hz at −44 dBFS and could jump octaves on harmonics | 30–1200 Hz range, −54 dBFS gate, subharmonic lock |
| `noAutoConvertSRC` forced miniaudio's linear resampler in Shared mode | Only enabled for Exclusive mode |

The old `SignalConditionerNode` was split into `DcBlockerNode` + `CabSimNode` so `IDspNode::process(sample, sampleRate)` stays a pure per-sample interface.

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

### Using `build.bat` (Windows):
```bat
build.bat
SubHz.exe
```

### Using MSYS2 UCRT64 Terminal:
```bash
cd /c/unibuc/proiecte/SubHz
g++ -std=c++17 -O3 -Wall -Wextra main.cpp -o SubHz.exe \
  -static-libgcc -static-libstdc++ \
  -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive,-Bdynamic \
  -lraylib -lopengl32 -lgdi32 -lwinmm
./SubHz.exe
```

### Running outside MSYS2:
Ensure `libraylib.dll` and `glfw3.dll` (from `C:\msys64\ucrt64\bin`) reside in the same folder as `SubHz.exe`, or have `C:\msys64\ucrt64\bin` added to your system `PATH`.
