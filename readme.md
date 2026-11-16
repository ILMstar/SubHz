# SubHz 🎸🔊

Real-time full-duplex bass & vocal DSP rig, dual-band spectrum visualizer, triggered oscilloscope, and chromatic tuner written in **C++17** using **Raylib** and **miniaudio** (Windows / WASAPI). Licensed under the [MIT License](LICENSE).

![SubHz Application Screenshot](assets/subbhz.png)

---

## Signal Flow

```text
IN (1+2 / 1 / 2) ─▶ Pre-Trim ADC Meter ─▶ Trim ─▶ DcBlocker (10 Hz HPF)
                                                     │
                                                     ├─▶ Dry Tap (48 kHz) ──▶ Hybrid YIN + FFT Tuner
                                                     ▼
                                     [ Dynamic Reorderable DSP Rack ]
                                      GATE ⇄ COMP ⇄ OD (4x OS) ⇄ EQ ⇄ CAB
                                                     │
                                                     ▼
                                           Soft-Knee Limiter (±0.98)
                                                     │
                                                     ├─▶ Wet Tap (48 kHz) ──────▶ Triggered Scope & 900 Hz–10 kHz FFT
                                                     ├─▶ 4× Decimator (12 kHz) ─▶ 2.93 Hz/bin Low-Band FFT (20–900 Hz)
                                                     ▼
                                            5 ms Mute Ramp ─▶ OUT (Dual Mono L/R)
```

---

## Features

- **Full-Duplex WASAPI Audio & Device Management** (48 kHz, 32-bit float, 2 ch in / 2 ch out, dual-mono output):
  - Opens **Shared mode first** so laptop mic-array drivers (beamforming, mic boost, noise/echo suppression) stay active; toggles live to **Exclusive** mode (`E`) with `kPeriods = 1` and MMCSS `"Pro Audio"` thread priority for low-latency interface monitoring.
  - Explicit HUD toast banner when an Exclusive mode request is rejected by a busy/incompatible endpoint and falls back to Shared mode.
  - Live hardware device hot-plug / loss detection (`ma_device_notification_type_stopped` / `rerouted`) with an immediate red status alert (`DEVICE LOST - PRESS [E] OR [D] TO RECONNECT`).
  - Live WASAPI capture (`D`) and playback (`F2`) endpoint cycling without restarting the app *(Note: when selecting separate physical input and output hardware with independent crystal clocks, use the same interface for both capture and playback whenever possible to avoid long-term duplex clock drift)*.
  - Input channel selector (`I`): **IN 1+2** (sum, default), **IN 1**, **IN 2**.
  - Pre-trim **ADC** peak meter & `ADC CLIP` indicator (reads before digital trim so hardware input clipping is always visible), plus post-FX **OUT** peak meter, soft-knee `tanh` safety limiter (ceiling ±0.98), and click-free mute ramp (`Space`).
- **Dynamic Reorderable DSP Pedal Rack** (`subhz_audio.h`, `main.cpp`):
  - Drag any pedal block (`GATE`, `COMP`, `OD`, `EQ`, `CAB`) left or right with the mouse in the Signal Flow panel to reorder the live signal chain on the fly (e.g., place **EQ before Overdrive** to sculpt clip drive, or **Compressor after Overdrive** to tame dynamics).
  - Powered by a wait-free bit-packed permutation (`std::atomic<uint32_t>`, 4 bits per slot) with zero heap allocations or mutex locks on the audio thread.
- **6-Stage Modular OOP DSP Chain** (`IDspNode` in `subhz_dsp.h`):
  1. `DcBlockerNode` *(fixed input stage)*: 1st-order **10 Hz** high-pass filter (`−0.43 dB` at `B0 = 30.87 Hz`, `−0.25 dB` at `E1 = 41.20 Hz`, `−0.14 dB` at `A1 = 55.00 Hz`).
  2. `EnvelopeNoiseGateNode` (`G`): 1 ms attack / 80 ms release with hysteresis (open `−54 dBFS` / close `−62 dBFS`), off by default.
  3. `CompressorNode`: Feed-forward optical bass compressor (`10 ms` attack, single-stage `120 ms` release to `37%`, `4:1` ratio) with `6.67 ms` peak bridging, `15 ms` click-free parameter smoothing, and live Gain Reduction (`GR`) telemetry.
  4. `BassOverdriveNode` (`O`, `X`, `1/2`, `3/4`): Phase-coherent 1st-order crossover at **150 Hz** keeps sub-bass `< 150 Hz` clean and tight while driving `> 150 Hz` through **1× / 2× (`96 kHz`) / 4× (`192 kHz`, default) oversampled** asymmetric tube-style soft clipping (`tanh(u * drive + 0.1) - tanh(0.1)`), filtered by a **4th-order Butterworth anti-aliasing LPF** (`0.40 × fs`, `19.2 kHz` @ `48 kHz`) before decimation, with automatic gain compensation, a **post-saturation 15 Hz DC blocker** (`< −66 dBFS` residual DC), and smoothed dry/wet blend.
  5. `ThreeBandEqNode` (`5/6`, `7/8`, `9/0`): 3 cascaded RBJ Audio EQ Cookbook biquads in **Transposed Direct Form II** with `15 ms` parameter smoothing and steady-state coefficient caching (Bass low-shelf `80 Hz Q=0.707`, Mid peaking `650 Hz Q=1.0`, Treble high-shelf `3.2 kHz Q=0.707`, each `±15 dB`).
  6. `CabSimNode` (`C`): 2nd-order Butterworth low-pass at 4.5 kHz with 10 ms click-free crossfade.
- **4 Click-Free Built-In Tone Presets (`P`) & JSON Rig Serialization (`S` / `L`)**:
  - Save (`S`) and load (`L`) complete custom rigs—including preset name, trim, gate, compressor threshold/makeup, overdrive drive/blend/oversampling, 3-band EQ, cab sim, and custom `chainOrder`—to/from `subhz_preset.json` with a zero-dependency parser/serializer.
  1. **Clean Studio DI** *(Default)* — Flat EQ, Drive off, light optical compression (`−18 dB` / `+1.5 dB`), Cab off.
  2. **Warm Motown Vintage** — Bass `+4 dB`, Mid `+2 dB`, Treble `−6 dB`, Cab Sim ON.
  3. **Modern Slap Scoop** — Bass `+5 dB`, Mid `−6 dB`, Treble `+6 dB`, high compression (`−26 dB` / `+5.0 dB`).
  4. **Darkglass Clank / Grit** — Drive `12.0x`, Blend `65%`, Bass `+4 dB`, Mid `+3 dB`, Treble `+4 dB`, Cab Sim ON.
- **Lock-Free Multi-Tap Analysis** (`LockFreeHistoryBuffer<4096>`, wait-free SPSC):
  - **Dry tap (48 kHz)** → hybrid YIN + FFT chromatic tuner.
  - **Wet tap (48 kHz)** → Schmitt-triggered rising zero-crossing oscilloscope (`2048` samples with sub-sample interpolation) + high-band FFT spectrum (`900 Hz – 10 kHz`).
  - **Low-Band Wet tap (12 kHz, 4× decimated)** → high-resolution `2.93 Hz`/bin low-frequency FFT (`20 Hz – 900 Hz`) so 5-string `B0 (30.87 Hz)` and `E1 (41.20 Hz)` sit `3.52` bins apart and resolve as sharp, distinct peaks.
- **Hybrid YIN + Spectral Chromatic Tuner (`30 Hz – 1200 Hz`, `subhz_tuner.h`)**:
  - Combines Hann-calibrated spectral peak interpolation (`< 1.5 ct` at `30–45 Hz`, `< 0.25 ct` above `45 Hz`) with time-domain **YIN / Normalized Square Difference** period verification (`NormalizedLagDiff`), locking onto true fundamentals on `B0, E1, A1, D2, G2` even when the fundamental is **−18 dB below the 2nd harmonic**.
  - User-adjustable tuner gate threshold (`[` / `]`, default `−44 dBFS`), spectral crest-factor SNR check, 50/60 Hz low-level mains-hum rejection, and 3-frame pitch stability lock (`TunerTracker`).
- **Frame-Rate Independent Smoothing & Persistence**:
  - Frame-rate independent exponential ballistics (`TimeConstantLerp`), live **VSync** toggle (`V`), resizable High-DPI window (`FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI`), and automatic state persistence (`subhz.cfg`).

---

## Controls

| Input | Action | Default |
|---|---|---|
| **Space** | Toggle live speaker monitoring (`MUTED` / `LIVE`) | `MUTED` |
| **Mouse Drag** | Drag `GATE`, `COMP`, `OD`, `EQ`, `CAB` in rack to reorder chain | `GATE > COMP > OD > EQ > CAB` |
| **P** | Cycle 4 built-in Bass Tone Presets (click-free smoothed) | `Clean Studio DI` |
| **S / L** | Export / Import custom rig & chain order (`subhz_preset.json`) | `subhz_preset.json` |
| **O** | Toggle Split-Band Overdrive bypass | `OFF` |
| **X** | Cycle Overdrive oversampling (`1x` → `2x` → `4x` @ `192 kHz`) | `4x` |
| **1 / 2** | Decrease / Increase Overdrive Drive (`1.0x – 25.0x`) | `2.0x` |
| **3 / 4** | Decrease / Increase Overdrive Dry/Wet Blend (`0% – 100%`) | `0%` |
| **5 / 6** | Decrease / Increase Bass EQ (`80 Hz`, `±15 dB`) | `0.0 dB` |
| **7 / 8** | Decrease / Increase Mid EQ (`650 Hz`, `±15 dB`) | `0.0 dB` |
| **9 / 0** | Decrease / Increase Treble EQ (`3.2 kHz`, `±15 dB`) | `0.0 dB` |
| **G** | Toggle Noise Gate (`−54 / −62 dBFS`) | `OFF` |
| **C** | Toggle 4.5 kHz Cab Sim LPF | `OFF` |
| **[ / ]** | Decrease / Increase Tuner Gate Threshold (`−60` to `−20 dB`) | `−44 dB` |
| **I** | Cycle input channel: `IN 1+2` → `IN 1` → `IN 2` | `IN 1+2` |
| **D / F2** | Cycle WASAPI Capture Input (`D`) / Playback Output (`F2`) device | `System Default` |
| **E** | Toggle WASAPI `SHARED` ↔ `EXCLUSIVE` driver mode | `SHARED` |
| **V** | Toggle `VSYNC` ↔ `180 FPS` frame pacing | `VSYNC ON` |
| **Up / Down / R** | Input trim `+1 / −1 dB` / Reset `0 dB` | `0.0 dB` |
| **Escape** | Save settings to `subhz.cfg` and exit | |

---

## JSON Rig Preset Format (`subhz_preset.json`)

Press `S` to export the current rig or `L` to load `subhz_preset.json` at runtime:

```json
{
  "name": "Smashing Pumpkins Fuzz Bass",
  "trim": 1.7500,
  "gateEnabled": true,
  "compThreshDb": -24.0000,
  "compMakeupDb": 4.0000,
  "odBypass": false,
  "odDrive": 18.5000,
  "odBlend": 0.8500,
  "odOversample": 4,
  "bassDb": 5.0000,
  "midDb": -3.5000,
  "trebleDb": 4.0000,
  "cabEnabled": true,
  "chainOrder": ["GATE", "OD", "COMP", "EQ", "CAB"]
}
```

---

## Prerequisites & Building (Windows / MSYS2 UCRT64)

### 1. Install Dependencies via MSYS2
Install [MSYS2](https://www.msys2.org/) and open the **MSYS2 UCRT64** terminal to install the GCC C++17 toolchain and **Raylib (≥ 5.0 recommended; 4.5+ supported via fallback guard)**:
```bash
pacman -Syu
pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-raylib
```

### 2. Build & Run
`build.bat` automatically detects `g++` from `%MSYS2_ROOT%\ucrt64\bin`, your system `PATH`, or standard `C:\msys64\ucrt64\bin` / `D:\msys64\ucrt64\bin` locations:
```bat
build.bat
SubHz.exe
```

### 3. Run Offline Verification Test Suite
To compile and run the 17-test offline unit & DSP verification suite (`tests.cpp`):
```bat
build.bat test
```

Or manually from the **MSYS2 UCRT64** shell:
```bash
g++ -std=c++17 -O3 -Wall -Wextra main.cpp -o SubHz.exe \
  -static-libgcc -static-libstdc++ \
  -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive,-Bdynamic \
  -lraylib -lopengl32 -lgdi32 -lwinmm
```

---

## Project Structure

- [`main.cpp`](main.cpp) — Raylib UI loop, resizable High-DPI layout, interactive drag-and-drop DSP rack, status bar, meters, triggered scope, and dual-band spectrum renderer.
- [`subhz_config.h`](subhz_config.h) — Configuration constants, `DspSlot` validation, `TimeConstantLerp`, `LockFreeHistoryBuffer<N>`, `subhz.cfg` persistence, and `RigPreset` JSON serialization.
- [`subhz_dsp.h`](subhz_dsp.h) — `IDspNode`, `DcBlockerNode` (`10 Hz`), `EnvelopeNoiseGateNode`, `CompressorNode` (`120 ms` single-stage), `BassOverdriveNode` (`150 Hz` split, `1×/2×/4×` oversampling + 4th-order Butterworth AA LPF + `15 Hz` DC block), `ThreeBandEqNode`, `CabSimNode`, `SoftKneeLimit`, and built-in presets.
- [`subhz_tuner.h`](subhz_tuner.h) — In-place Radix-2 FFT, hybrid YIN + spectral `DetectBassPitch`, `TunerTracker` stability/hum filter, and `FindScopeTriggerIndex`.
- [`subhz_audio.h`](subhz_audio.h) — WASAPI full-duplex `AudioEngine`, lock-free dynamic chain routing (`packedOrder_`), JSON rig preset import/export, endpoint enumeration/cycling, hot-plug notification handling, and 4×-decimated low-band tap.
- [`tests.cpp`](tests.cpp) — Automated 17-test offline suite covering tuner octave lock, hum rejection, compressor ballistics, click-free preset transitions, DC blocking, dynamic rack routing, overdrive oversampling anti-aliasing, JSON preset serialization, and thread safety.
- [`.github/workflows/ci.yml`](.github/workflows/ci.yml) — GitHub Actions CI compiling `SubHz.exe` and running `tests.cpp` with `-Wall -Wextra -Werror`.
