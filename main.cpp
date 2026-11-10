// =====================================================================================
//  SubHz - Full-Duplex Audio & Bass DSP Rig, Visualizer & Tuner (C++17, raylib, miniaudio)
//
//  Audio thread (WASAPI duplex, shared mode default for mic APOs, exclusive [E] toggle):
//    IN (1+2 / 1 / 2) -> Pre-Trim IN Meter -> Trim -> [DcBlocker: HPF 10 Hz]
//                                                        |  (DC-blocked, pre-FX) -> dryBuffer -> Tuner
//                                                        v
//                                                  [EnvelopeNoiseGate (G)] -> [Compressor (Optical 4:1)]
//                                                        v
//                                                  [BassOverdrive: 150 Hz Split + DC Block (O)] -> [ThreeBandEq (5-0)]
//                                                        v
//                                                  [CabSim: LPF 4.5 kHz (C)] -> Soft-knee tanh limiter
//                                                        | -> wetBuffer (48 kHz) + lowWetBuffer (12 kHz, 2.93 Hz bins)
//                                                        v
//                                                  Mute ramp -> OUT (dual mono L/R)
//
//  UI thread (raylib, VSync / 180 FPS, resizable HighDPI): lock-free snapshots, never blocks audio.
// =====================================================================================

#if defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #define Rectangle   Win32_Rectangle
    #define CloseWindow Win32_CloseWindow
    #define ShowCursor  Win32_ShowCursor
#endif

#define MA_NO_DECODING
#define MA_NO_ENCODING
#define MA_NO_GENERATION
#define MA_NO_RESOURCE_MANAGER
#define MA_NO_NODE_GRAPH
#define MA_NO_ENGINE
#define MA_API static
#define MINIAUDIO_IMPLEMENTATION
#if defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "miniaudio.h"
#if defined(__GNUC__)
    #pragma GCC diagnostic pop
#endif

#if defined(_WIN32)
    #undef Rectangle
    #undef CloseWindow
    #undef ShowCursor
    #undef DrawText
    #undef DrawTextEx
    #undef LoadImage
    #undef PlaySound
#endif

#include "raylib.h"

// Compatibility fallback for Raylib < 5.0 (where IsKeyPressedRepeat was introduced)
#if !defined(RAYLIB_VERSION_MAJOR) || (RAYLIB_VERSION_MAJOR < 5)
static inline bool IsKeyPressedRepeat(int) { return false; }
#endif

#include "subhz_config.h"
#include "subhz_dsp.h"
#include "subhz_tuner.h"
#include "subhz_audio.h"

namespace ui_col {
const Color kBackground = { 0x12, 0x12, 0x12, 0xFF };   // #121212
const Color kStatusBg   = { 0x0B, 0x0B, 0x0B, 0xFF };
const Color kPanelBg    = { 0x1A, 0x1A, 0x1A, 0xFF };
const Color kNeon       = { 0x00, 0xFF, 0xC8, 0xFF };
const Color kNeonLow    = { 0x00, 0xD2, 0xFF, 0xFF };
const Color kNeonHot    = { 0xAD, 0xFF, 0x2F, 0xFF };
const Color kTuneGreen  = { 0x00, 0xFF, 0x66, 0xFF };
const Color kTuneOrange = { 0xFF, 0x9F, 0x1C, 0xFF };
const Color kTuneRed    = { 0xFF, 0x3B, 0x30, 0xFF };
const Color kGrid       = { 0x2A, 0x2A, 0x2A, 0xFF };
}  // namespace ui_col

// -------------------------------------------------------------------------------------
//  UI helpers
// -------------------------------------------------------------------------------------
static float FreqToNormLogX(float freqHz) {
    const float logMin = std::log10(cfg::kMinFreq);
    const float logMax = std::log10(cfg::kMaxFreq);
    return (std::log10(freqHz) - logMin) / (logMax - logMin);
}

static Color BlendColor(Color a, Color b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    auto mix = [t](unsigned char ca, unsigned char cb) {
        return static_cast<unsigned char>(std::round(
            Lerp(static_cast<float>(ca), static_cast<float>(cb), t)));
    };
    return Color{ mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b), mix(a.a, b.a) };
}

static void UpdatePeakMeter(float& displayDb, float peakLin, float dt) {
    const float db     = 20.0f * std::log10(std::max(peakLin, 1e-6f));
    const float fallen = displayDb - cfg::kMeterFallDbPerSec * dt;
    displayDb = std::max(db, std::max(fallen, cfg::kMeterFloorDb));
}

static int DrawChip(int x, int y, const char* text, int fontSize, Color fg, Color bg) {
    constexpr int kPadX = 8;
    const int w = MeasureText(text, fontSize) + 2 * kPadX;
    DrawRectangle(x, y, w, fontSize + 8, bg);
    DrawText(text, x + kPadX, y + 4, fontSize, fg);
    return w;
}

static void DrawStatusBar(const AudioEngine& engine, int screenW, bool vsyncOn) {
    constexpr int kBarH  = 34;
    constexpr int kFont  = 16;
    constexpr int kChipY = (kBarH - (kFont + 8)) / 2;
    constexpr int kGap   = 8;
    const Color chipBg   = { 0x1C, 0x1C, 0x1C, 0xFF };

    DrawRectangle(0, 0, screenW, kBarH, ui_col::kStatusBg);
    DrawLine(0, kBarH, screenW, kBarH, ui_col::kGrid);

    int x = 14;
    DrawText("SubHz", x, kChipY + 3, 20, ui_col::kNeon);
    x += MeasureText("SubHz", 20) + 14;

    if (!engine.Running()) {
        DrawChip(x, kChipY, "NO AUDIO DEVICE - PRESS [E] OR [D]", kFont, RAYWHITE, ui_col::kTuneRed);
    } else if (engine.DeviceLost()) {
        DrawChip(x, kChipY, "DEVICE LOST - PRESS [E] OR [D] TO RECONNECT", kFont,
                 RAYWHITE, ui_col::kTuneRed);
    } else {
        x += DrawChip(x, kChipY,
                      engine.Exclusive() ? "WASAPI EXCLUSIVE [E]" : "WASAPI SHARED [E]",
                      kFont, ui_col::kBackground,
                      engine.Exclusive() ? ui_col::kNeon : ui_col::kTuneGreen) + kGap;

        const LatencyInfo lat = engine.Latency();
        x += DrawChip(x, kChipY,
                      TextFormat("%ux%u @ %.1fk  %.2f ms  RT~%.1f ms", lat.periodFrames,
                                 lat.periods, lat.sampleRate / 1000.0f, lat.periodMs,
                                 lat.roundTripMs),
                      kFont, LIGHTGRAY, chipBg) + kGap;

        const char* inLabel = engine.Input() == InputSelect::In1 ? "IN 1 [I]"
                            : engine.Input() == InputSelect::In2 ? "IN 2 [I]"
                                                                 : "IN 1+2 [I]";
        x += DrawChip(x, kChipY, inLabel, kFont, ui_col::kNeon, chipBg) + kGap;

        x += DrawChip(x, kChipY, engine.GateEnabled() ? "GATE: ON [G]" : "GATE: OFF [G]",
                      kFont, engine.GateEnabled() ? ui_col::kTuneGreen : GRAY, chipBg) + kGap;

        x += DrawChip(x, kChipY, engine.OverdriveEnabled() ? "OD: ON [O]" : "OD: OFF [O]",
                      kFont, engine.OverdriveEnabled() ? ui_col::kTuneOrange : GRAY, chipBg) + kGap;

        x += DrawChip(x, kChipY, engine.CabEnabled() ? "CAB: 4.5k [C]" : "CAB: OFF [C]",
                      kFont, engine.CabEnabled() ? ui_col::kTuneGreen : GRAY, chipBg) + kGap;

        DrawChip(x, kChipY, engine.Muted() ? "MUTED [SPACE]" : "LIVE [SPACE]", kFont,
                 engine.Muted() ? RAYWHITE : ui_col::kBackground,
                 engine.Muted() ? ui_col::kTuneRed : ui_col::kTuneGreen);
    }

    const char* fps = TextFormat("%s  FPS %d", vsyncOn ? "VSYNC [V]" : "180Hz [V]", GetFPS());
    DrawText(fps, screenW - MeasureText(fps, kFont) - 14, kChipY + 4, kFont, GRAY);
}

static void DrawPeakMeter(int x, int y, int w, int h, const char* label, float db) {
    constexpr int kLabelW = 40;
    const int barX = x + kLabelW;
    const int barW = w - kLabelW - 76;
    auto dbToX = [&](float d) {
        const float t = (d - cfg::kMeterFloorDb) / (0.0f - cfg::kMeterFloorDb);
        return barX + static_cast<int>(std::round(std::clamp(t, 0.0f, 1.0f) * barW));
    };

    DrawText(label, x, y, 14, LIGHTGRAY);
    DrawRectangle(barX, y, barW, h, Color{ 0x10, 0x10, 0x10, 0xFF });

    struct Zone { float lo, hi; Color c; };
    const Zone zones[] = { { cfg::kMeterFloorDb, -12.0f, ui_col::kTuneGreen },
                           { -12.0f, -3.0f, ui_col::kTuneOrange },
                           { -3.0f, 0.0f, ui_col::kTuneRed } };
    for (const auto& z : zones) {
        if (db <= z.lo) break;
        const int x0 = dbToX(z.lo);
        const int x1 = dbToX(std::min(db, z.hi));
        if (x1 > x0) DrawRectangle(x0, y + 1, x1 - x0, h - 2, z.c);
    }
    for (float tick : { -48.0f, -36.0f, -24.0f, -12.0f, -6.0f, -3.0f }) {
        const int tx = dbToX(tick);
        DrawLine(tx, y, tx, y + h, Fade(ui_col::kBackground, 0.8f));
    }
    DrawRectangleLines(barX, y, barW, h, ui_col::kGrid);

    const char* value = db <= cfg::kMeterFloorDb + 0.05f ? "-inf dB" : TextFormat("%.1f dB", db);
    DrawText(value, barX + barW + 8, y, 14, GRAY);
}

static void DrawTelemetryPanel(const AudioEngine& engine, float inDb, float outDb,
                               bool limitLed, bool adcClipLed) {
    constexpr int kX = 24;
    constexpr int kY = 46;
    constexpr int kW = 392;

    DrawText(TextFormat("TRIM x%.2f (%+.1f dB)  [Up/Down, R]", engine.Trim(),
                        20.0f * std::log10(engine.Trim())),
             kX, kY, 16, LIGHTGRAY);
    DrawPeakMeter(kX, kY + 22, kW, 12, "ADC", inDb);
    DrawPeakMeter(kX, kY + 40, kW, 12, "OUT", outDb);

    // Compressor Gain Reduction (GR) meter (0..18 dB scale)
    const int   grY    = kY + 58;
    const int   barX   = kX + 40;
    const int   barW   = kW - 40 - 76;
    const float grDb   = std::clamp(engine.CompGainReductionDb(), 0.0f, 18.0f);
    const int   fillW  = static_cast<int>(std::round((grDb / 18.0f) * barW));
    DrawText("GR", kX, grY - 1, 14, LIGHTGRAY);
    DrawRectangle(barX, grY, barW, 12, Color{ 0x10, 0x10, 0x10, 0xFF });
    if (fillW > 0) DrawRectangle(barX, grY + 1, fillW, 10, ui_col::kTuneOrange);
    DrawRectangleLines(barX, grY, barW, 12, ui_col::kGrid);
    DrawText(TextFormat("-%.1f dB", engine.CompGainReductionDb()),
             barX + barW + 8, grY - 1, 14, grDb > 0.2f ? ui_col::kTuneOrange : GRAY);

    // LIMIT / ADC CLIP LED
    const int  ledY   = kY + 78;
    const bool anyHot = limitLed || adcClipLed;
    DrawRectangle(kX, ledY, 14, 14, anyHot ? ui_col::kTuneRed : Color{ 0x3A, 0x10, 0x10, 0xFF });
    DrawRectangleLines(kX, ledY, 14, 14, ui_col::kGrid);
    DrawText(adcClipLed ? "ADC CLIP" : "LIMIT", kX + 20, ledY, 14,
             anyHot ? ui_col::kTuneRed : DARKGRAY);

    // Gate LED
    const bool gateEnabled = engine.GateEnabled();
    const bool gateOpen    = engine.GateOpen();
    const Color gateLedCol = !gateEnabled ? Color{ 0x26, 0x26, 0x26, 0xFF }
                           : gateOpen     ? ui_col::kTuneGreen
                                          : Color{ 0x10, 0x30, 0x18, 0xFF };
    DrawRectangle(kX + 94, ledY, 14, 14, gateLedCol);
    DrawRectangleLines(kX + 94, ledY, 14, 14, ui_col::kGrid);
    const char* gateTxt = !gateEnabled ? "GATE BYPASS" : (gateOpen ? "GATE OPEN" : "GATE CLOSED");
    DrawText(gateTxt, kX + 114, ledY, 14,
             !gateEnabled ? GRAY : (gateOpen ? ui_col::kTuneGreen : DARKGRAY));

    DrawText(TextFormat("COMP %.0fdB / %+.1fdB (4:1)", engine.CompThresholdDb(),
                        engine.CompMakeupDb()),
             kX + 226, ledY + 1, 12, DARKGRAY);
}

static void DrawSignalFlowPanel(const AudioEngine& engine, int screenW) {
    const int kX = std::max(864, screenW - 416);
    constexpr int kY = 46;
    DrawText(TextFormat("PRESET [P]: %s%s", engine.PresetName(),
                        engine.PresetModified() ? " *" : ""),
             kX, kY, 14, ui_col::kNeon);
    DrawText("IN > HPF 10Hz > GATE > COMP > OD > EQ > CAB > LIM", kX, kY + 18, 12, LIGHTGRAY);
    DrawText(TextFormat("OD [O]: %-3s  Drive [1/2]: %4.1fx  Blend [3/4]: %3d%%",
                        engine.OverdriveEnabled() ? "ON" : "OFF",
                        engine.Drive(),
                        static_cast<int>(std::round(engine.Blend() * 100.0f))),
             kX, kY + 36, 12, engine.OverdriveEnabled() ? ui_col::kTuneOrange : GRAY);
    DrawText(TextFormat("EQ (dB)  Bass [5/6]: %+5.1f  Mid [7/8]: %+5.1f  Treb [9/0]: %+5.1f",
                        engine.BassDb(), engine.MidDb(), engine.TrebleDb()),
             kX, kY + 54, 12, ui_col::kNeonLow);
    DrawText(TextFormat("In [D]:  %.42s", engine.CaptureName()),  kX, kY + 74, 12, DARKGRAY);
    DrawText(TextFormat("Out [F2]: %.41s", engine.PlaybackName()), kX, kY + 90, 12, DARKGRAY);
}

static void DrawTunerOverlay(const TunerReading& tuner, float smoothedCents,
                             float smoothedHz, float tunerGateDb, int screenW) {
    constexpr int kBoxW    = 420;
    constexpr int kBoxH    = 118;
    const     int kBoxX    = (screenW - kBoxW) / 2;
    constexpr int kBoxY    = 44;
    const     int kCenterX = screenW / 2;

    Color statusCol = DARKGRAY;
    if (tuner.active) {
        const float absCents = std::fabs(smoothedCents);
        if (absCents <= cfg::kInTuneCents) {
            statusCol = ui_col::kTuneGreen;
        } else {
            const float severity = std::clamp((absCents - cfg::kInTuneCents) / 20.0f,
                                              0.0f, 1.0f);
            statusCol = BlendColor(ui_col::kTuneOrange, ui_col::kTuneRed, severity);
        }
    }

    DrawRectangle(kBoxX, kBoxY, kBoxW, kBoxH, ui_col::kPanelBg);
    DrawRectangleLines(kBoxX, kBoxY, kBoxW, kBoxH,
                       tuner.active ? Fade(statusCol, 0.85f) : ui_col::kGrid);

    const char* noteText = tuner.active
                               ? TextFormat("%s%d", tuner.noteName, tuner.octave)
                               : "--";
    constexpr int kNoteFontSize = 44;
    const int noteW = MeasureText(noteText, kNoteFontSize);
    DrawText(noteText, kCenterX - noteW / 2, kBoxY + 10, kNoteFontSize,
             tuner.active ? statusCol : GRAY);

    if (tuner.active) {
        DrawText(TextFormat("%.1f Hz", smoothedHz), kBoxX + 16, kBoxY + 20, 18, LIGHTGRAY);
        DrawText(TextFormat("Gate [ / ]: %.0f dB", tunerGateDb), kBoxX + 16, kBoxY + 42, 12, DARKGRAY);
        const bool inTune = std::fabs(smoothedCents) <= cfg::kInTuneCents;
        const char* centsTxt = inTune ? TextFormat("%+.1f ct  OK", smoothedCents)
                                      : TextFormat("%+.1f ct", smoothedCents);
        const int centsW = MeasureText(centsTxt, 18);
        DrawText(centsTxt, kBoxX + kBoxW - centsW - 16, kBoxY + 24, 18, statusCol);
    } else {
        DrawText("YIN+FFT 30-1200 Hz", kBoxX + 16, kBoxY + 20, 14, DARKGRAY);
        DrawText(TextFormat("Gate [ / ]: %.0f dB", tunerGateDb), kBoxX + 16, kBoxY + 40, 12, DARKGRAY);
        const char* idleTxt = "PLAY A NOTE";
        DrawText(idleTxt, kBoxX + kBoxW - MeasureText(idleTxt, 16) - 16,
                 kBoxY + 26, 16, DARKGRAY);
    }

    constexpr int   kGaugeW    = 350;
    const     int   kGaugeX    = kCenterX - kGaugeW / 2;
    constexpr int   kGaugeY    = kBoxY + 76;
    constexpr int   kGaugeH    = 14;
    constexpr float kPxPerCent = kGaugeW / 100.0f;

    DrawRectangle(kGaugeX, kGaugeY, kGaugeW, kGaugeH, Color{ 0x10, 0x10, 0x10, 0xFF });
    const int inTuneHalfW = static_cast<int>(std::round(cfg::kInTuneCents * kPxPerCent));
    DrawRectangle(kCenterX - inTuneHalfW, kGaugeY, inTuneHalfW * 2, kGaugeH,
                  Fade(ui_col::kTuneGreen, tuner.active ? 0.25f : 0.10f));
    DrawRectangleLines(kGaugeX, kGaugeY, kGaugeW, kGaugeH, ui_col::kGrid);

    for (int ct : { -50, -25, 0, 25, 50 }) {
        const int tx = kCenterX + static_cast<int>(std::round(ct * kPxPerCent));
        const int th = (ct == 0) ? 6 : 4;
        DrawLine(tx, kGaugeY - th, tx, kGaugeY + kGaugeH + th,
                 (ct == 0) ? RAYWHITE : GRAY);
    }
    DrawText("-50", kGaugeX - 10, kGaugeY + kGaugeH + 6, 12, DARKGRAY);
    DrawText("0",   kCenterX - 3, kGaugeY + kGaugeH + 6, 12, GRAY);
    DrawText("+50", kGaugeX + kGaugeW - 14, kGaugeY + kGaugeH + 6, 12, DARKGRAY);
    DrawText("b",   kBoxX + 14, kGaugeY, 16, GRAY);
    DrawText("#",   kBoxX + kBoxW - 22, kGaugeY, 16, GRAY);

    if (tuner.active) {
        const float clamped = std::clamp(smoothedCents, -50.0f, 50.0f);
        const int   needleX = kCenterX + static_cast<int>(std::round(clamped * kPxPerCent));
        const int   barLeft = std::min(kCenterX, needleX);
        const int   barW    = std::abs(needleX - kCenterX);
        if (barW > 0) {
            DrawRectangle(barLeft, kGaugeY + 3, barW, kGaugeH - 6, Fade(statusCol, 0.55f));
        }
        DrawRectangle(needleX - 2, kGaugeY - 5, 5, kGaugeH + 10, statusCol);
    }
}

// -------------------------------------------------------------------------------------
//  main (UI thread)
// -------------------------------------------------------------------------------------
int main() {
    AppSettings settings{};
    LoadSettings(cfg::kConfigFile, settings);
    bool vsyncOn = settings.vsync;

    unsigned int winFlags = FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI;
    if (vsyncOn) winFlags |= FLAG_VSYNC_HINT;
    SetConfigFlags(winFlags);
    InitWindow(cfg::kWidth, cfg::kHeight, "SubHz - Full-Duplex Bass DSP Rig");
    SetWindowMinSize(cfg::kMinWidth, cfg::kMinHeight);
    SetTargetFPS(vsyncOn ? 0 : cfg::kTargetFps);

    // ---- Audio engine ---------------------------------------------------------------
    auto engine = std::make_unique<AudioEngine>();
    engine->ImportSettings(settings);
    if (engine->Start(settings.exclusive)) {
        const LatencyInfo lat = engine->Latency();
        TraceLog(LOG_INFO, "SubHz: WASAPI %s | in: %s | out: %s | %u x %u frames @ %u Hz",
                 engine->Exclusive() ? "EXCLUSIVE" : "SHARED", engine->CaptureName(),
                 engine->PlaybackName(), lat.periodFrames, lat.periods, lat.sampleRate);
    } else {
        TraceLog(LOG_WARNING, "SubHz: failed to open WASAPI full-duplex device");
    }

    // ---- Analysis buffers -----------------------------------------------------------
    std::vector<float> hannWindow(cfg::kFftSize);
    for (size_t i = 0; i < cfg::kFftSize; ++i) {
        hannWindow[i] = static_cast<float>(
            0.5 * (1.0 - std::cos(2.0 * cfg::kPi * static_cast<double>(i) /
                                  static_cast<double>(cfg::kFftSize - 1))));
    }

    std::vector<std::complex<float>> fftBuffer(cfg::kFftSize);
    std::vector<float>               dryMagnitudes(cfg::kHalfBins + 1, 0.0f);
    std::vector<float>               wetMagnitudes(cfg::kHalfBins + 1, 0.0f);
    std::vector<float>               lowWetMagnitudes(cfg::kHalfBins + 1, 0.0f);
    std::array<float, cfg::kNumBars> previous_frame_heights{};

    static std::array<float, cfg::kFftSize>        drySamples{};
    static std::array<float, cfg::kFftSize>        wetSamples{};
    static std::array<float, cfg::kFftSize>        lowWetSamples{};
    static std::array<Vector2, cfg::kScopeSamples> wavePoints{};

    TunerTracker tunerTracker{};
    tunerTracker.SetGateDb(settings.tunerGateDb);
    TunerReading lastTuner{};
    int   lastMidiNote  = -1;
    float smoothedCents = 0.0f;
    float smoothedHz    = 0.0f;

    float inMeterDb     = cfg::kMeterFloorDb;
    float outMeterDb    = cfg::kMeterFloorDb;
    float limitLedTime  = 0.0f;
    float adcClipTime   = 0.0f;

    char  bannerText[160]{};
    bool  bannerWarning = false;
    float bannerTimer   = 0.0f;

    // Precompute dual-band logarithmic spectrum bar bin bounds:
    // - Bars below kLowBandCrossHz (900 Hz) use the 4x-decimated 12 kHz FFT (2.93 Hz bins,
    //   separating B0 30.87 Hz and E1 41.20 Hz by 3.52 full bins).
    // - Bars >= 900 Hz use the full-rate 48 kHz FFT (11.72 Hz bins).
    struct BarBinSpec {
        bool   useLowBand;
        bool   subBin;
        size_t k0;
        size_t k1;
        float  frac;
    };
    std::array<BarBinSpec, cfg::kNumBars> barSpecs{};
    float cachedSpecRate = 0.0f;

    auto rebuildBarSpecs = [&](float fullRate) {
        cachedSpecRate        = fullRate;
        const float lowRate   = fullRate / static_cast<float>(cfg::kLowBandDecim);
        const float fullBinHz = fullRate / static_cast<float>(cfg::kFftSize);
        const float lowBinHz  = lowRate  / static_cast<float>(cfg::kFftSize);
        const float logMin    = std::log10(cfg::kMinFreq);
        const float logMax    = std::log10(cfg::kMaxFreq);

        for (int b = 0; b < cfg::kNumBars; ++b) {
            const float t0 = static_cast<float>(b) / cfg::kNumBars;
            const float t1 = static_cast<float>(b + 1) / cfg::kNumBars;
            const float f0 = std::pow(10.0f, logMin + t0 * (logMax - logMin));
            const float f1 = std::pow(10.0f, logMin + t1 * (logMax - logMin));
            const float fc = 0.5f * (f0 + f1);

            const bool  useLow = (f1 <= cfg::kLowBandCrossHz);
            const float bw     = useLow ? lowBinHz : fullBinHz;
            const float maxBin = std::min(static_cast<float>(cfg::kHalfBins), cfg::kMaxFreq / bw);
            const float bin0   = std::clamp(f0 / bw, 1.0f, maxBin);
            const float bin1   = std::clamp(f1 / bw, 1.0f, maxBin);

            if (bin1 - bin0 <= 1.0f) {
                const float  binCenter = std::clamp(fc / bw, 1.0f, maxBin);
                const size_t k0        = static_cast<size_t>(std::floor(binCenter));
                const size_t k1        = std::min(k0 + 1, cfg::kHalfBins);
                barSpecs[b] = { useLow, true, k0, k1, binCenter - static_cast<float>(k0) };
            } else {
                const size_t kStart = static_cast<size_t>(std::floor(bin0));
                const size_t kEnd   = std::min(static_cast<size_t>(std::ceil(bin1)), cfg::kHalfBins);
                barSpecs[b] = { useLow, false, kStart, kEnd, 0.0f };
            }
        }
    };
    rebuildBarSpecs(engine->SampleRate());

    size_t lastDryWrite    = 0;
    size_t lastWetWrite    = 0;
    size_t lastLowWetWrite = 0;

    // ---- Main loop ------------------------------------------------------------------
    while (!WindowShouldClose()) {
        const float dt      = GetFrameTime();
        const int   screenW = std::max(GetScreenWidth(),  cfg::kMinWidth);
        const int   screenH = std::max(GetScreenHeight(), cfg::kMinHeight);
        if (engine->SampleRate() != cachedSpecRate) rebuildBarSpecs(engine->SampleRate());

        // 1. Controls
        auto KeyStep = [](int k) { return IsKeyPressed(k) || IsKeyPressedRepeat(k); };
        const float step = std::pow(10.0f, cfg::kTrimStepDb / 20.0f);
        if (KeyStep(KEY_UP))           engine->SetTrim(engine->Trim() * step);
        if (KeyStep(KEY_DOWN))         engine->SetTrim(engine->Trim() / step);
        if (IsKeyPressed(KEY_R))       engine->SetTrim(1.0f);
        if (IsKeyPressed(KEY_G))       engine->ToggleGate();
        if (IsKeyPressed(KEY_C))       engine->ToggleCab();
        if (IsKeyPressed(KEY_I))       engine->CycleInput();
        if (IsKeyPressed(KEY_SPACE))   engine->ToggleMute();
        if (IsKeyPressed(KEY_E))       engine->ToggleDriverMode();
        if (IsKeyPressed(KEY_D))       engine->CycleCaptureDevice();
        if (IsKeyPressed(KEY_F2))      engine->CyclePlaybackDevice();
        if (IsKeyPressed(KEY_P))       engine->CyclePreset();
        if (IsKeyPressed(KEY_O))       engine->ToggleOverdrive();
        if (KeyStep(KEY_ONE))          engine->SetDrive(engine->Drive() - cfg::kDriveStep);
        if (KeyStep(KEY_TWO))          engine->SetDrive(engine->Drive() + cfg::kDriveStep);
        if (KeyStep(KEY_THREE))        engine->SetBlend(engine->Blend() - cfg::kBlendStep);
        if (KeyStep(KEY_FOUR))         engine->SetBlend(engine->Blend() + cfg::kBlendStep);
        if (KeyStep(KEY_FIVE))         engine->SetBassDb(engine->BassDb() - cfg::kEqStepDb);
        if (KeyStep(KEY_SIX))          engine->SetBassDb(engine->BassDb() + cfg::kEqStepDb);
        if (KeyStep(KEY_SEVEN))        engine->SetMidDb(engine->MidDb() - cfg::kEqStepDb);
        if (KeyStep(KEY_EIGHT))        engine->SetMidDb(engine->MidDb() + cfg::kEqStepDb);
        if (KeyStep(KEY_NINE))         engine->SetTrebleDb(engine->TrebleDb() - cfg::kEqStepDb);
        if (KeyStep(KEY_ZERO))         engine->SetTrebleDb(engine->TrebleDb() + cfg::kEqStepDb);
        if (KeyStep(KEY_LEFT_BRACKET)) {
            tunerTracker.SetGateDb(tunerTracker.GateDb() - cfg::kTunerGateStepDb);
        }
        if (KeyStep(KEY_RIGHT_BRACKET)) {
            tunerTracker.SetGateDb(tunerTracker.GateDb() + cfg::kTunerGateStepDb);
        }
        if (IsKeyPressed(KEY_V)) {
            vsyncOn = !vsyncOn;
            if (vsyncOn) {
                SetWindowState(FLAG_VSYNC_HINT);
                SetTargetFPS(0);
            } else {
                ClearWindowState(FLAG_VSYNC_HINT);
                SetTargetFPS(cfg::kTargetFps);
            }
        }

        // Poll engine status notices (mode fallback, device loss, endpoint change)
        bool noticeWarn = false;
        if (engine->ConsumeNotice(bannerText, sizeof(bannerText), noticeWarn)) {
            bannerWarning = noticeWarn;
            bannerTimer   = noticeWarn ? 4.0f : 2.5f;
        } else if (bannerTimer > 0.0f) {
            bannerTimer = std::max(0.0f, bannerTimer - dt);
        }

        // 2. Telemetry
        UpdatePeakMeter(inMeterDb,  engine->ConsumeInputPeak(),  dt);
        UpdatePeakMeter(outMeterDb, engine->ConsumeOutputPeak(), dt);
        limitLedTime = engine->ConsumeLimitEvent()   ? cfg::kLimitLedHoldSec
                                                     : std::max(0.0f, limitLedTime - dt);
        adcClipTime  = engine->ConsumeAdcClipEvent() ? cfg::kLimitLedHoldSec
                                                     : std::max(0.0f, adcClipTime - dt);

        // 3. DRY tap -> tuner (only run FFT + YIN when new audio samples have been written)
        const float binWidthHz = engine->SampleRate() / static_cast<float>(cfg::kFftSize);
        TunerReading tuner     = lastTuner;
        const size_t curDryWrite = engine->DryTap().TotalWritten();
        if (curDryWrite != lastDryWrite && engine->DryTap().Snapshot(drySamples)) {
            lastDryWrite = curDryWrite;
            ComputeMagnitudes(drySamples, hannWindow, fftBuffer, dryMagnitudes);
            tuner = tunerTracker.Update(drySamples, dryMagnitudes, binWidthHz);
        }
        lastTuner = tuner;
        if (tuner.active) {
            if (tuner.midiNote != lastMidiNote) {
                smoothedCents = tuner.cents;
                smoothedHz    = tuner.frequencyHz;
                lastMidiNote  = tuner.midiNote;
            } else {
                smoothedCents = TimeConstantLerp(smoothedCents, tuner.cents, cfg::kTunerSmoothRate, dt);
                smoothedHz    = TimeConstantLerp(smoothedHz, tuner.frequencyHz, cfg::kTunerSmoothRate, dt);
            }
        } else {
            lastMidiNote = -1;
        }

        // 4. WET tap -> triggered scope + dual-resolution spectrum
        const size_t curWetWrite = engine->WetTap().TotalWritten();
        if (curWetWrite != lastWetWrite) {
            lastWetWrite = curWetWrite;
            engine->WetTap().Snapshot(wetSamples);
            ComputeMagnitudes(wetSamples, hannWindow, fftBuffer, wetMagnitudes);
        }
        const size_t curLowWetWrite = engine->LowWetTap().TotalWritten();
        if (curLowWetWrite != lastLowWetWrite) {
            lastLowWetWrite = curLowWetWrite;
            engine->LowWetTap().Snapshot(lowWetSamples);
            ComputeMagnitudes(lowWetSamples, hannWindow, fftBuffer, lowWetMagnitudes);
        }

        // Dynamic layout coordinates (scales smoothly when window is resized / HighDPI)
        const float kMarginX  = 24.0f;
        const float kPlotW    = static_cast<float>(screenW) - 2.0f * kMarginX;
        const float kWaveTop  = 172.0f;
        const float kWaveH    = std::max(64.0f, static_cast<float>(screenH) * 0.114f);
        const float kSpecTop  = kWaveTop + kWaveH + 14.0f;
        const float kSpecBot  = static_cast<float>(screenH) - 44.0f;
        const float kSpecMaxH = std::max(120.0f, kSpecBot - kSpecTop);

        const bool hasLowBandHistory = (curLowWetWrite >= cfg::kFftSize / 4);
        for (int b = 0; b < cfg::kNumBars; ++b) {
            const BarBinSpec& spec = barSpecs[b];
            const auto& mags = (spec.useLowBand && hasLowBandHistory) ? lowWetMagnitudes : wetMagnitudes;
            float bandMag = 0.0f;
            if (spec.subBin) {
                bandMag = Lerp(mags[spec.k0], mags[spec.k1], spec.frac);
            } else {
                for (size_t k = spec.k0; k <= spec.k1; ++k) {
                    bandMag = std::max(bandMag, mags[k]);
                }
            }

            const float db   = 20.0f * std::log10(std::max(bandMag, 1e-9f));
            const float norm = std::clamp((db - cfg::kMinDb) / (cfg::kMaxDb - cfg::kMinDb),
                                          0.0f, 1.0f);
            previous_frame_heights[b] = TimeConstantLerp(
                previous_frame_heights[b], norm * kSpecMaxH, cfg::kSpectrumSmoothRate, dt);
        }

        // Triggered 2048-sample oscilloscope (locks on rising zero-crossing)
        float        subSampleFrac = 0.0f;
        const size_t trigIdx = FindScopeTriggerIndex(wetSamples, cfg::kScopeSamples, &subSampleFrac);
        const float  waveCenterY = kWaveTop + kWaveH * 0.5f;
        const float  waveAmpPx   = kWaveH * 0.45f;
        const float  waveDx      = kPlotW / static_cast<float>(cfg::kScopeSamples - 1);
        for (size_t i = 0; i < cfg::kScopeSamples; ++i) {
            const float v = std::clamp(wetSamples[trigIdx + i], -1.1f, 1.1f);
            const float xPos = kMarginX + (static_cast<float>(i) - subSampleFrac) * waveDx;
            wavePoints[i] = { std::clamp(xPos, kMarginX, kMarginX + kPlotW),
                              waveCenterY - v * waveAmpPx };
        }

        // ---- Draw -------------------------------------------------------------------
        BeginDrawing();
        ClearBackground(ui_col::kBackground);

        DrawStatusBar(*engine, screenW, vsyncOn);
        DrawTelemetryPanel(*engine, inMeterDb, outMeterDb, limitLedTime > 0.0f, adcClipTime > 0.0f);
        DrawSignalFlowPanel(*engine, screenW);
        DrawTunerOverlay(tuner, smoothedCents, smoothedHz, tunerTracker.GateDb(), screenW);

        // Oscilloscope (WET, triggered)
        DrawRectangleLines(static_cast<int>(kMarginX), static_cast<int>(kWaveTop),
                           static_cast<int>(kPlotW), static_cast<int>(kWaveH), ui_col::kGrid);
        DrawLine(static_cast<int>(kMarginX), static_cast<int>(waveCenterY),
                 static_cast<int>(kMarginX + kPlotW), static_cast<int>(waveCenterY),
                 ui_col::kGrid);
        DrawLineStrip(wavePoints.data(), static_cast<int>(cfg::kScopeSamples),
                      Fade(ui_col::kNeon, 0.85f));
        DrawText("WET SCOPE (triggered rising 0-cross, 2048 samples)", static_cast<int>(kMarginX) + 8,
                 static_cast<int>(kWaveTop) + 6, 12, GRAY);

        // Status / fallback toast banner (if active)
        if (bannerTimer > 0.0f && bannerText[0] != '\0') {
            const int bw = MeasureText(bannerText, 16) + 28;
            const int bx = (screenW - bw) / 2;
            const int by = static_cast<int>(kWaveTop) + 6;
            DrawRectangle(bx, by, bw, 26,
                          bannerWarning ? ui_col::kTuneRed : Color{ 0x18, 0x38, 0x30, 0xF0 });
            DrawRectangleLines(bx, by, bw, 26, bannerWarning ? ui_col::kTuneOrange : ui_col::kNeon);
            DrawText(bannerText, bx + 14, by + 5, 16, RAYWHITE);
        }

        // Spectrum (WET, dual-band 2.93 Hz low-end resolution)
        const float bassZoneW = FreqToNormLogX(2000.0f) * kPlotW;
        DrawRectangle(static_cast<int>(kMarginX), static_cast<int>(kSpecTop),
                      static_cast<int>(bassZoneW), static_cast<int>(kSpecMaxH),
                      Color{ 0x00, 0xFF, 0xC8, 0x07 });
        DrawRectangleLines(static_cast<int>(kMarginX), static_cast<int>(kSpecTop),
                           static_cast<int>(kPlotW), static_cast<int>(kSpecMaxH), ui_col::kGrid);

        for (int dbTick : { -15, -30, -45 }) {
            const float norm = (static_cast<float>(dbTick) - cfg::kMinDb) /
                               (cfg::kMaxDb - cfg::kMinDb);
            const int y = static_cast<int>(kSpecBot - norm * kSpecMaxH);
            DrawLine(static_cast<int>(kMarginX), y,
                     static_cast<int>(kMarginX + kPlotW), y, Fade(ui_col::kGrid, 0.65f));
            DrawText(TextFormat("%d dB", dbTick), static_cast<int>(kMarginX) + 6, y + 3,
                     12, DARKGRAY);
        }

        struct FreqTick { float hz; const char* label; };
        constexpr FreqTick kTicks[] = {
            { 20.0f,    "20Hz"  }, { 30.87f,   "B0"    }, { 41.20f,   "E1"    },
            { 55.0f,    "A1"    }, { 73.42f,   "D2"    }, { 98.0f,    "G2"    },
            { 200.0f,   "200"   }, { 500.0f,   "500"   }, { 1000.0f,  "1k"    },
            { 2000.0f,  "2k"    }, { 5000.0f,  "5k"    }, { 10000.0f, "10kHz" },
        };
        for (const auto& tick : kTicks) {
            const int x = static_cast<int>(std::round(
                kMarginX + FreqToNormLogX(tick.hz) * kPlotW));
            const bool isNote = (tick.hz > 25.0f && tick.hz < 110.0f);
            DrawLine(x, static_cast<int>(kSpecTop), x, static_cast<int>(kSpecBot),
                     isNote ? Color{ 0x00, 0xFF, 0xC8, 0x22 } : Fade(ui_col::kGrid, 0.7f));
            DrawText(tick.label, x - 10, static_cast<int>(kSpecBot) + 8, 14,
                     isNote ? ui_col::kNeon : GRAY);
        }
        DrawText("WET SPECTRUM - BASS FOCUS (20 Hz - 2 kHz, 2.93 Hz low-band FFT)",
                 static_cast<int>(kMarginX) + 8,
                 static_cast<int>(kSpecTop) + 6, 12, Fade(ui_col::kNeon, 0.65f));
        DrawText("HARMONICS / ATTACK (2k - 10k)",
                 static_cast<int>(kMarginX + bassZoneW) + 8,
                 static_cast<int>(kSpecTop) + 6, 12, GRAY);

        const float slotW = kPlotW / static_cast<float>(cfg::kNumBars);
        const int   barW  = std::max(1, static_cast<int>(std::floor(slotW)) - 2);
        for (int b = 0; b < cfg::kNumBars; ++b) {
            const int barH = static_cast<int>(std::round(previous_frame_heights[b]));
            if (barH <= 0) continue;

            const int x = static_cast<int>(std::round(kMarginX + b * slotW)) + 1;
            const int y = static_cast<int>(kSpecBot) - barH;

            const float freqFrac = static_cast<float>(b) / (cfg::kNumBars - 1);
            const float ampFrac  = std::clamp(barH / kSpecMaxH, 0.0f, 1.0f);
            const Color baseCol  = BlendColor(ui_col::kNeonLow, ui_col::kNeon,
                                              std::min(freqFrac * 1.4f, 1.0f));
            DrawRectangle(x, y, barW, barH, BlendColor(baseCol, ui_col::kNeonHot, ampFrac * 0.45f));
        }

        if (tuner.active) {
            const int peakX = static_cast<int>(std::round(
                kMarginX + FreqToNormLogX(smoothedHz) * kPlotW));
            const bool inTune = std::fabs(smoothedCents) <= cfg::kInTuneCents;
            DrawLine(peakX, static_cast<int>(kSpecTop), peakX, static_cast<int>(kSpecBot),
                     Fade(inTune ? ui_col::kTuneGreen : ui_col::kTuneOrange, 0.6f));
        }

        DrawText("[P] Preset  [O] OD  [1/2] Drive  [3/4] Blend  [5-0] EQ  "
                 "[G] Gate  [C] Cab  [ [ / ] ] Tuner Gate  [D/F2] Device  [E] Driver  [V] VSync  [Space] Mute",
                 static_cast<int>(kMarginX), screenH - 18, 12, DARKGRAY);

        EndDrawing();
    }

    // ---- Persist settings & Shutdown ------------------------------------------------
    engine->ExportSettings(settings);
    settings.tunerGateDb = tunerTracker.GateDb();
    settings.vsync       = vsyncOn;
    SaveSettings(cfg::kConfigFile, settings);

    engine->Stop();
    engine.reset();
    CloseWindow();
    return 0;
}
