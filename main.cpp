// =====================================================================================
//  SubHz - Full-Duplex Audio & Bass DSP Rig, Visualizer & Tuner (C++17, raylib, miniaudio)
//
//  Audio thread (WASAPI duplex, shared mode default for mic APOs, exclusive [E] toggle):
//    IN (1+2 / 1 / 2) -> Pre-Trim IN Meter -> Trim -> [DcBlocker: HPF 10 Hz]
//                                                        |  (DC-blocked, pre-FX) -> dryBuffer -> Tuner
//                                                        v
//                                      [Reorderable Rack: GATE <-> COMP <-> OD (4x OS) <-> EQ <-> CAB]
//                                                        v
//                                                  Soft-knee tanh limiter
//                                                        | -> wetBuffer (48 kHz) + lowWetBuffer (12 kHz, 2.93 Hz bins)
//                                                        v
//                                                  Mute ramp -> OUT (dual mono L/R)
//
//  UI thread (raylib, VSync / 180 FPS, resizable HighDPI): AmpliTube-style studio rig layout.
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
const Color kBackground  = { 0x10, 0x10, 0x12, 0xFF };
const Color kTopBarBg    = { 0x18, 0x18, 0x1C, 0xFF };
const Color kSidebarBg   = { 0x14, 0x14, 0x18, 0xFF };
const Color kStageBg     = { 0x0B, 0x0B, 0x0E, 0xFF };
const Color kTolex       = { 0x16, 0x16, 0x19, 0xFF };
const Color kOledBg      = { 0x07, 0x0A, 0x0D, 0xFF };
const Color kPanelBg     = { 0x1C, 0x1C, 0x21, 0xFF };
const Color kBrassHi     = { 0xD8, 0xB4, 0x67, 0xFF };
const Color kBrassMid    = { 0xB5, 0x8E, 0x3E, 0xFF };
const Color kBrassDark   = { 0x78, 0x5A, 0x22, 0xFF };
const Color kPiping      = { 0xDF, 0xCA, 0x92, 0xFF };
const Color kCableActive = { 0xE6, 0x8A, 0x2E, 0xFF };
const Color kCableDim    = { 0x3A, 0x3A, 0x42, 0xFF };
const Color kNeon        = { 0x00, 0xFF, 0xC8, 0xFF };
const Color kNeonLow     = { 0x00, 0xD2, 0xFF, 0xFF };
const Color kNeonHot     = { 0xAD, 0xFF, 0x2F, 0xFF };
const Color kTuneGreen   = { 0x00, 0xE6, 0x64, 0xFF };
const Color kTuneOrange  = { 0xFF, 0x9F, 0x1C, 0xFF };
const Color kTuneRed     = { 0xFF, 0x3B, 0x30, 0xFF };
const Color kGrid        = { 0x28, 0x28, 0x30, 0xFF };
}  // namespace ui_col

// -------------------------------------------------------------------------------------
//  UI helpers & interactive widgets
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

static bool PointInRect(Vector2 p, int x, int y, int w, int h) {
    return p.x >= static_cast<float>(x) && p.x <= static_cast<float>(x + w) &&
           p.y >= static_cast<float>(y) && p.y <= static_cast<float>(y + h);
}

static bool DrawButton(int x, int y, int w, int h, const char* text, int fontSize,
                       Color fg, Color bg, Color border, bool active = false) {
    const Vector2 m       = GetMousePosition();
    const bool    hovered = PointInRect(m, x, y, w, h);
    const Color   fill    = active  ? BlendColor(bg, border, 0.28f)
                          : hovered ? BlendColor(bg, RAYWHITE, 0.10f)
                                    : bg;
    DrawRectangle(x, y, w, h, fill);
    DrawRectangleLines(x, y, w, h, active || hovered ? border : ui_col::kGrid);
    const int tw = MeasureText(text, fontSize);
    DrawText(text, x + (w - tw) / 2, y + (h - fontSize) / 2, fontSize, fg);
    return hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

// Interactive hardware rotary knob (mouse vertical drag, mouse wheel, right-click reset)
static void DrawRotaryKnob(int id, int cx, int cy, int radius, const char* label,
                           const char* valText, float value, float minVal, float maxVal,
                           float defaultVal, float step, int& activeKnobId,
                           float& outNewValue, bool& outChanged) {
    outNewValue = value;
    outChanged  = false;

    const Vector2 m       = GetMousePosition();
    const float   dx      = m.x - static_cast<float>(cx);
    const float   dy      = m.y - static_cast<float>(cy);
    const bool    hovered = (dx * dx + dy * dy) <= static_cast<float>((radius + 8) * (radius + 8));

    if (hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        activeKnobId = id;
    }
    if (activeKnobId == id) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            const float deltaY = GetMouseDelta().y;
            if (std::fabs(deltaY) > 0.01f) {
                const float range = maxVal - minVal;
                outNewValue = std::clamp(value - (deltaY / 110.0f) * range, minVal, maxVal);
                outChanged  = (outNewValue != value);
            }
        } else {
            activeKnobId = -1;
        }
    }
    if (hovered) {
        const float wheel = GetMouseWheelMove();
        if (std::fabs(wheel) > 0.01f) {
            outNewValue = std::clamp(value + wheel * step, minVal, maxVal);
            outChanged  = (outNewValue != value);
        }
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            outNewValue = defaultVal;
            outChanged  = (outNewValue != value);
        }
    }

    const float norm     = std::clamp((outNewValue - minVal) / (maxVal - minVal), 0.0f, 1.0f);
    // Sweep from -135 deg (7 o'clock) to +135 deg (5 o'clock), where 0 deg is 12 o'clock (up)
    constexpr float kMinDeg = -135.0f;
    constexpr float kMaxDeg =  135.0f;

    // Surrounding scale ticks (0..10)
    for (int t = 0; t <= 10; ++t) {
        const float tf  = static_cast<float>(t) / 10.0f;
        const float deg = kMinDeg + tf * (kMaxDeg - kMinDeg);
        const float rad = (deg - 90.0f) * static_cast<float>(cfg::kPi) / 180.0f;
        const float r0  = static_cast<float>(radius + 3);
        const float r1  = static_cast<float>(radius + ((t % 5 == 0) ? 7 : 5));
        const Color tc  = (tf <= norm + 0.01f) ? Color{ 0x1A, 0x14, 0x08, 0xFF }
                                               : Color{ 0x58, 0x45, 0x1E, 0xFF };
        DrawLineEx({ cx + std::cos(rad) * r0, cy + std::sin(rad) * r0 },
                   { cx + std::cos(rad) * r1, cy + std::sin(rad) * r1 },
                   (t % 5 == 0) ? 2.0f : 1.2f, tc);
    }

    // Outer shadow + knurled skirt + brushed gold cap
    DrawCircle(cx + 1, cy + 2, static_cast<float>(radius + 1), Color{ 0x18, 0x12, 0x06, 0xAA });
    DrawCircle(cx, cy, static_cast<float>(radius), Color{ 0x22, 0x20, 0x1C, 0xFF });
    DrawCircleLines(cx, cy, static_cast<float>(radius),
                    (activeKnobId == id || hovered) ? ui_col::kNeon : Color{ 0x10, 0x0E, 0x0A, 0xFF });
    DrawCircle(cx, cy, static_cast<float>(radius - 4), ui_col::kBrassMid);
    DrawCircle(cx - 1, cy - 1, static_cast<float>(radius - 6), Color{ 0xEB, 0xCE, 0x86, 0xFF });
    DrawCircleLines(cx, cy, static_cast<float>(radius - 4), Color{ 0x55, 0x40, 0x18, 0xFF });

    // Indicator pointer line
    const float curDeg = kMinDeg + norm * (kMaxDeg - kMinDeg);
    const float curRad = (curDeg - 90.0f) * static_cast<float>(cfg::kPi) / 180.0f;
    DrawLineEx({ static_cast<float>(cx), static_cast<float>(cy) },
               { cx + std::cos(curRad) * static_cast<float>(radius - 2),
                 cy + std::sin(curRad) * static_cast<float>(radius - 2) },
               2.6f, Color{ 0x14, 0x10, 0x08, 0xFF });

    // Label above knob (engraved dark text on brass plate)
    const int lw = MeasureText(label, 11);
    DrawText(label, cx - lw / 2, cy - radius - 17, 11, Color{ 0x18, 0x12, 0x06, 0xFF });

    // Value pill below knob
    const int vw = std::max(44, MeasureText(valText, 10) + 10);
    DrawRectangle(cx - vw / 2, cy + radius + 5, vw, 13, Color{ 0x14, 0x12, 0x0E, 0xE8 });
    const int tw = MeasureText(valText, 10);
    DrawText(valText, cx - tw / 2, cy + radius + 7, 10, ui_col::kPiping);
}

// -------------------------------------------------------------------------------------
//  1. Top Studio Toolbar (Preset switcher, Save/Load JSON, Amp Branding, Quick Status)
// -------------------------------------------------------------------------------------
static void DrawTopToolbar(AudioEngine& engine, int screenW, bool& vsyncOn) {
    constexpr int kBarH = 38;
    DrawRectangle(0, 0, screenW, kBarH, ui_col::kTopBarBg);
    DrawLine(0, kBarH - 1, screenW, kBarH - 1, ui_col::kGrid);

    int x = 12;
    constexpr int kBtnY = 6;
    constexpr int kBtnH = 26;

    // Preset selector box (AmpliTube style)
    if (DrawButton(x, kBtnY, 24, kBtnH, "<", 14, LIGHTGRAY, ui_col::kPanelBg, ui_col::kBrassHi)) {
        engine.ApplyPreset(engine.PresetIndex() - 1);
    }
    x += 26;

    constexpr int kPresetBoxW = 196;
    DrawRectangle(x, kBtnY, kPresetBoxW, kBtnH, Color{ 0x10, 0x10, 0x14, 0xFF });
    DrawRectangleLines(x, kBtnY, kPresetBoxW, kBtnH, ui_col::kBrassMid);
    const char* presetLabel = TextFormat("%s%s", engine.PresetModified() ? "*" : "", engine.PresetName());
    DrawText(presetLabel, x + 8, kBtnY + 6, 14, ui_col::kBrassHi);
    x += kPresetBoxW + 2;

    if (DrawButton(x, kBtnY, 24, kBtnH, ">", 14, LIGHTGRAY, ui_col::kPanelBg, ui_col::kBrassHi)) {
        engine.CyclePreset();
    }
    x += 30;

    if (DrawButton(x, kBtnY, 64, kBtnH, "SAVE [S]", 11, RAYWHITE, ui_col::kPanelBg, ui_col::kNeon)) {
        engine.SaveRigPresetJson(cfg::kPresetJsonFile);
    }
    x += 68;

    if (DrawButton(x, kBtnY, 64, kBtnH, "LOAD [L]", 11, RAYWHITE, ui_col::kPanelBg, ui_col::kNeonLow)) {
        engine.LoadRigPresetJson(cfg::kPresetJsonFile);
    }

    // Center Brand Title
    const char* brand = "SubHz";
    const int   bw    = MeasureText(brand, 22);
    const int   bx    = screenW / 2 - bw / 2;
    DrawText(brand, bx + 1, 9, 22, Color{ 0x00, 0x00, 0x00, 0xAA });
    DrawText(brand, bx,     8, 22, ui_col::kBrassHi);

    // Right status controls
    int rx = screenW - 12;

    const bool muted = engine.Muted();
    rx -= 98;
    if (DrawButton(rx, kBtnY, 98, kBtnH, muted ? "MUTED [SPC]" : "LIVE [SPC]", 11,
                   muted ? RAYWHITE : ui_col::kBackground,
                   muted ? ui_col::kTuneRed : ui_col::kTuneGreen,
                   muted ? ui_col::kTuneRed : ui_col::kTuneGreen, true)) {
        engine.ToggleMute();
    }

    rx -= 88;
    if (DrawButton(rx, kBtnY, 82, kBtnH,
                   TextFormat("%s %d", vsyncOn ? "VSYNC" : "180Hz", GetFPS()), 11,
                   LIGHTGRAY, ui_col::kPanelBg, ui_col::kGrid)) {
        vsyncOn = !vsyncOn;
        if (vsyncOn) {
            SetWindowState(FLAG_VSYNC_HINT);
            SetTargetFPS(0);
        } else {
            ClearWindowState(FLAG_VSYNC_HINT);
            SetTargetFPS(cfg::kTargetFps);
        }
    }

    rx -= 136;
    if (!engine.Running() || engine.DeviceLost()) {
        if (DrawButton(rx, kBtnY, 130, kBtnH, "DEVICE LOST [E]", 11,
                       RAYWHITE, ui_col::kTuneRed, ui_col::kTuneRed, true)) {
            engine.ToggleDriverMode();
        }
    } else {
        const bool excl = engine.Exclusive();
        if (DrawButton(rx, kBtnY, 130, kBtnH,
                       excl ? "WASAPI EXCL [E]" : "WASAPI SHARED [E]", 11,
                       excl ? ui_col::kNeon : ui_col::kTuneGreen,
                       ui_col::kPanelBg, excl ? ui_col::kNeon : ui_col::kTuneGreen)) {
            engine.ToggleDriverMode();
        }
    }
}

// -------------------------------------------------------------------------------------
//  2. Upper Amp-Head Visualizer Cabinet (OLED Scope + Spectrum + Brass Faceplate Knobs)
// -------------------------------------------------------------------------------------
static void DrawAmpVisualizerCabinet(
    AudioEngine& engine, int x, int y, int w, int h,
    const std::array<float, cfg::kFftSize>& wetSamples,
    const std::array<float, cfg::kNumBars>& barHeights,
    const TunerReading& tuner, float smoothedHz, float smoothedCents,
    std::array<Vector2, cfg::kScopeSamples>& wavePoints,
    int& activeKnobId, const char* bannerText, bool bannerWarn, float bannerTimer) {

    // Outer Tolex Amp Shell
    DrawRectangle(x, y, w, h, ui_col::kTolex);
    DrawRectangleLinesEx({ static_cast<float>(x), static_cast<float>(y),
                           static_cast<float>(w), static_cast<float>(h) },
                         2.0f, Color{ 0x2C, 0x2C, 0x32, 0xFF });

    // Corner protectors (Marshall style)
    constexpr int kCorner = 14;
    for (int cx : { x, x + w - kCorner }) {
        for (int cy : { y, y + h - kCorner }) {
            DrawRectangle(cx, cy, kCorner, kCorner, Color{ 0x0A, 0x0A, 0x0C, 0xFF });
            DrawRectangleLines(cx, cy, kCorner, kCorner, Color{ 0x38, 0x38, 0x40, 0xFF });
        }
    }

    // Inner piping frame
    const int inX = x + 12;
    const int inY = y + 10;
    const int inW = w - 24;
    const int inH = h - 20;
    const int faceplateH = std::clamp(static_cast<int>(inH * 0.27f), 82, 102);
    const int oledH      = inH - faceplateH - 4;

    // ---- A. Studio OLED Visualizer Screen (replaces speaker cloth grille) -----------
    DrawRectangle(inX, inY, inW, oledH, ui_col::kOledBg);
    DrawRectangleLinesEx({ static_cast<float>(inX - 2), static_cast<float>(inY - 2),
                           static_cast<float>(inW + 4), static_cast<float>(oledH + 4) },
                         2.0f, ui_col::kPiping);

    const float padX     = static_cast<float>(inX + 10);
    const float plotW    = static_cast<float>(inW - 20);
    const float scopeTop = static_cast<float>(inY + 6);
    const float scopeH   = std::max(44.0f, static_cast<float>(oledH) * 0.23f);
    const float specTop  = scopeTop + scopeH + 6.0f;
    const float specBot  = static_cast<float>(inY + oledH - 20);
    const float specMaxH = std::max(60.0f, specBot - specTop);

    // 1) Triggered Oscilloscope strip (top of OLED)
    DrawRectangle(static_cast<int>(padX), static_cast<int>(scopeTop),
                  static_cast<int>(plotW), static_cast<int>(scopeH), Color{ 0x05, 0x08, 0x0B, 0xFF });
    DrawRectangleLines(static_cast<int>(padX), static_cast<int>(scopeTop),
                       static_cast<int>(plotW), static_cast<int>(scopeH), Color{ 0x1A, 0x24, 0x2C, 0xFF });

    float        subSampleFrac = 0.0f;
    const size_t trigIdx = FindScopeTriggerIndex(wetSamples, cfg::kScopeSamples, &subSampleFrac);
    const float  waveCenterY = scopeTop + scopeH * 0.5f;
    const float  waveAmpPx   = scopeH * 0.44f;
    const float  waveDx      = plotW / static_cast<float>(cfg::kScopeSamples - 1);
    for (size_t i = 0; i < cfg::kScopeSamples; ++i) {
        const float v    = std::clamp(wetSamples[trigIdx + i], -1.1f, 1.1f);
        const float xPos = padX + (static_cast<float>(i) - subSampleFrac) * waveDx;
        wavePoints[i] = { std::clamp(xPos, padX, padX + plotW), waveCenterY - v * waveAmpPx };
    }
    DrawLine(static_cast<int>(padX), static_cast<int>(waveCenterY),
             static_cast<int>(padX + plotW), static_cast<int>(waveCenterY), Color{ 0x18, 0x28, 0x30, 0xFF });
    DrawLineStrip(wavePoints.data(), static_cast<int>(cfg::kScopeSamples), Fade(ui_col::kNeon, 0.90f));
    DrawText("WET SCOPE (TRIGGERED 0-CROSS)", static_cast<int>(padX) + 6,
             static_cast<int>(scopeTop) + 4, 10, Fade(ui_col::kNeon, 0.55f));

    // 2) Dual-Band FFT Spectrum Analyzer (main OLED area)
    const float bassZoneW = FreqToNormLogX(2000.0f) * plotW;
    DrawRectangle(static_cast<int>(padX), static_cast<int>(specTop),
                  static_cast<int>(bassZoneW), static_cast<int>(specMaxH),
                  Color{ 0x00, 0xFF, 0xC8, 0x07 });
    DrawRectangleLines(static_cast<int>(padX), static_cast<int>(specTop),
                       static_cast<int>(plotW), static_cast<int>(specMaxH), Color{ 0x1A, 0x24, 0x2C, 0xFF });

    for (int dbTick : { -15, -30, -45 }) {
        const float norm = (static_cast<float>(dbTick) - cfg::kMinDb) / (cfg::kMaxDb - cfg::kMinDb);
        const int   ty   = static_cast<int>(specBot - norm * specMaxH);
        DrawLine(static_cast<int>(padX), ty, static_cast<int>(padX + plotW), ty,
                 Color{ 0x16, 0x20, 0x26, 0xFF });
        DrawText(TextFormat("%ddB", dbTick), static_cast<int>(padX) + 4, ty + 2, 10, DARKGRAY);
    }

    struct FreqTick { float hz; const char* label; };
    constexpr FreqTick kTicks[] = {
        { 20.0f, "20" },   { 30.87f, "B0" }, { 41.20f, "E1" }, { 55.0f, "A1" },
        { 73.42f, "D2" },  { 98.0f, "G2" },  { 200.0f, "200" }, { 500.0f, "500" },
        { 1000.0f, "1k" }, { 2000.0f, "2k" }, { 5000.0f, "5k" }, { 10000.0f, "10k" },
    };
    for (const auto& tick : kTicks) {
        const int  tx     = static_cast<int>(std::round(padX + FreqToNormLogX(tick.hz) * plotW));
        const bool isNote = (tick.hz > 25.0f && tick.hz < 110.0f);
        DrawLine(tx, static_cast<int>(specTop), tx, static_cast<int>(specBot),
                 isNote ? Color{ 0x00, 0xFF, 0xC8, 0x22 } : Color{ 0x18, 0x20, 0x26, 0xFF });
        DrawText(tick.label, tx - 7, static_cast<int>(specBot) + 4, 11,
                 isNote ? ui_col::kNeon : GRAY);
    }
    DrawText("BASS SPECTRUM (20 Hz - 2 kHz, 2.93 Hz low-band FFT)",
             static_cast<int>(padX) + 6, static_cast<int>(specTop) + 4, 10, Fade(ui_col::kNeon, 0.65f));
    DrawText("HARMONICS (2k - 10k)",
             static_cast<int>(padX + bassZoneW) + 6, static_cast<int>(specTop) + 4, 10, GRAY);

    const float slotW = plotW / static_cast<float>(cfg::kNumBars);
    const int   barW  = std::max(1, static_cast<int>(std::floor(slotW)) - 2);
    for (int b = 0; b < cfg::kNumBars; ++b) {
        const float normH = std::clamp(barHeights[b], 0.0f, 1.0f);
        const int   barH  = static_cast<int>(std::round(normH * specMaxH));
        if (barH <= 0) continue;

        const int bx = static_cast<int>(std::round(padX + b * slotW)) + 1;
        const int by = static_cast<int>(specBot) - barH;

        const float freqFrac = static_cast<float>(b) / (cfg::kNumBars - 1);
        const Color baseCol  = BlendColor(ui_col::kNeonLow, ui_col::kNeon, std::min(freqFrac * 1.4f, 1.0f));
        DrawRectangle(bx, by, barW, barH, BlendColor(baseCol, ui_col::kNeonHot, normH * 0.45f));
    }

    if (tuner.active) {
        const int  peakX  = static_cast<int>(std::round(padX + FreqToNormLogX(smoothedHz) * plotW));
        const bool inTune = std::fabs(smoothedCents) <= cfg::kInTuneCents;
        DrawLine(peakX, static_cast<int>(specTop), peakX, static_cast<int>(specBot),
                 Fade(inTune ? ui_col::kTuneGreen : ui_col::kTuneOrange, 0.70f));
    }

    // Toast Banner inside OLED if active
    if (bannerTimer > 0.0f && bannerText && bannerText[0] != '\0') {
        const int bw = MeasureText(bannerText, 14) + 24;
        const int bx = inX + (inW - bw) / 2;
        const int by = inY + 12;
        DrawRectangle(bx, by, bw, 24, bannerWarn ? ui_col::kTuneRed : Color{ 0x14, 0x34, 0x2E, 0xF2 });
        DrawRectangleLines(bx, by, bw, 24, bannerWarn ? ui_col::kTuneOrange : ui_col::kNeon);
        DrawText(bannerText, bx + 12, by + 5, 14, RAYWHITE);
    }

    // ---- B. Brushed Gold/Brass Amp Control Faceplate --------------------------------
    const int fpY = inY + oledH + 4;
    DrawRectangleGradientV(inX, fpY, inW, faceplateH, ui_col::kBrassHi, ui_col::kBrassMid);
    DrawRectangleLinesEx({ static_cast<float>(inX - 1), static_cast<float>(fpY),
                           static_cast<float>(inW + 2), static_cast<float>(faceplateH) },
                         2.0f, ui_col::kBrassDark);

    // Left Rocker Switches (POWER/MONITOR + OVERDRIVE)
    const int swY = fpY + (faceplateH - 42) / 2;
    const bool live = !engine.Muted();
    DrawText("MONITOR", inX + 18, swY - 13, 10, Color{ 0x1A, 0x12, 0x06, 0xFF });
    if (DrawButton(inX + 18, swY, 42, 42, live ? "ON" : "MUTE", 10,
                   RAYWHITE, live ? Color{ 0xD8, 0x22, 0x18, 0xFF } : Color{ 0x48, 0x14, 0x12, 0xFF },
                   Color{ 0x18, 0x12, 0x08, 0xFF }, live)) {
        engine.ToggleMute();
    }

    const bool odOn = engine.OverdriveEnabled();
    DrawText("DRIVE", inX + 72, swY - 13, 10, Color{ 0x1A, 0x12, 0x06, 0xFF });
    if (DrawButton(inX + 68, swY, 42, 42, odOn ? "OD ON" : "BYP", 10,
                   odOn ? ui_col::kBrassHi : GRAY,
                   Color{ 0x1C, 0x1A, 0x16, 0xFF }, Color{ 0x10, 0x0E, 0x08, 0xFF }, odOn)) {
        engine.ToggleOverdrive();
    }

    // 6 Rotary Hardware Knobs (DRIVE, BLEND, BASS, MIDDLE, TREBLE, TRIM)
    const int knobStartX = inX + 148;
    const int knobEndX   = inX + inW - 96;
    const int knobSpan   = std::max(240, knobEndX - knobStartX);
    const int knobStep   = knobSpan / 5;
    const int knobCy     = fpY + faceplateH / 2 - 1;
    const int knobRadius = std::clamp(faceplateH / 4, 18, 23);

    float newVal = 0.0f;
    bool  changed = false;

    // 1. DRIVE
    DrawRotaryKnob(0, knobStartX + 0 * knobStep, knobCy, knobRadius, "DRIVE",
                   TextFormat("%.1fx", engine.Drive()), engine.Drive(),
                   cfg::kDriveMin, cfg::kDriveMax, 2.0f, cfg::kDriveStep,
                   activeKnobId, newVal, changed);
    if (changed) engine.SetDrive(newVal);

    // 2. BLEND
    DrawRotaryKnob(1, knobStartX + 1 * knobStep, knobCy, knobRadius, "BLEND",
                   TextFormat("%d%%", static_cast<int>(std::round(engine.Blend() * 100.0f))),
                   engine.Blend(), cfg::kBlendMin, cfg::kBlendMax, 0.0f, cfg::kBlendStep,
                   activeKnobId, newVal, changed);
    if (changed) engine.SetBlend(newVal);

    // 3. BASS (80 Hz)
    DrawRotaryKnob(2, knobStartX + 2 * knobStep, knobCy, knobRadius, "BASS",
                   TextFormat("%+.1fdB", engine.BassDb()), engine.BassDb(),
                   cfg::kEqMinDb, cfg::kEqMaxDb, 0.0f, cfg::kEqStepDb,
                   activeKnobId, newVal, changed);
    if (changed) engine.SetBassDb(newVal);

    // 4. MIDDLE (650 Hz)
    DrawRotaryKnob(3, knobStartX + 3 * knobStep, knobCy, knobRadius, "MIDDLE",
                   TextFormat("%+.1fdB", engine.MidDb()), engine.MidDb(),
                   cfg::kEqMinDb, cfg::kEqMaxDb, 0.0f, cfg::kEqStepDb,
                   activeKnobId, newVal, changed);
    if (changed) engine.SetMidDb(newVal);

    // 5. TREBLE (3.2 kHz)
    DrawRotaryKnob(4, knobStartX + 4 * knobStep, knobCy, knobRadius, "TREBLE",
                   TextFormat("%+.1fdB", engine.TrebleDb()), engine.TrebleDb(),
                   cfg::kEqMinDb, cfg::kEqMaxDb, 0.0f, cfg::kEqStepDb,
                   activeKnobId, newVal, changed);
    if (changed) engine.SetTrebleDb(newVal);

    // 6. INPUT TRIM (dB domain -20..+40 dB)
    const float trimDb = 20.0f * std::log10(std::max(engine.Trim(), cfg::kTrimMin));
    DrawRotaryKnob(5, knobStartX + 5 * knobStep, knobCy, knobRadius, "PRE TRIM",
                   TextFormat("%+.1fdB", trimDb), trimDb,
                   -20.0f, 40.0f, 0.0f, cfg::kTrimStepDb,
                   activeKnobId, newVal, changed);
    if (changed) engine.SetTrim(DbToLin(newVal));

    // Right 1/4" Input Jack Graphic (clickable to cycle IN 1+2 / IN 1 / IN 2)
    const int jackX = inX + inW - 38;
    const int jackY = knobCy;
    DrawText("INPUT", jackX - 16, swY - 13, 10, Color{ 0x1A, 0x12, 0x06, 0xFF });
    DrawCircle(jackX, jackY, 14.0f, Color{ 0x28, 0x26, 0x22, 0xFF });
    DrawCircle(jackX, jackY, 10.0f, Color{ 0x88, 0x84, 0x78, 0xFF });
    DrawCircle(jackX, jackY, 5.0f,  Color{ 0x08, 0x08, 0x0A, 0xFF });
    // Instrument cable curving down
    DrawLineEx({ static_cast<float>(jackX), static_cast<float>(jackY + 4) },
               { static_cast<float>(jackX + 8), static_cast<float>(fpY + faceplateH) },
               5.0f, Color{ 0x12, 0x12, 0x16, 0xFF });
    const char* inCh = engine.Input() == InputSelect::In1 ? "IN 1"
                     : engine.Input() == InputSelect::In2 ? "IN 2" : "1+2";
    if (DrawButton(jackX - 20, knobCy + knobRadius + 4, 40, 14, inCh, 10,
                   ui_col::kPiping, Color{ 0x18, 0x14, 0x0E, 0xF0 }, ui_col::kBrassDark)) {
        engine.CycleInput();
    }
}

// -------------------------------------------------------------------------------------
//  3. Lower Signal Chain Strip (compact rack before full stompbox stage)
// -------------------------------------------------------------------------------------
static void DrawSignalChainStage(AudioEngine& engine, int x, int y, int w, int h,
                                 int& dragSlot, float& dragStartX, bool& dragMoved) {
    (void)dragStartX;
    (void)dragMoved;
    DrawRectangle(x, y, w, h, ui_col::kStageBg);
    DrawLine(x, y, x + w, y, ui_col::kGrid);
    const auto ord = engine.GetChainOrder();
    int sx = x + 24;
    const int sy = y + h / 2 - 12;
    for (int s = 0; s < cfg::kNumDspSlots; ++s) {
        if (DrawButton(sx + s * 96, sy, 84, 24, DspSlotName(ord[s]), 11,
                       LIGHTGRAY, ui_col::kPanelBg, ui_col::kBrassMid, engine.IsNodeActive(ord[s]))) {
            dragSlot = s;
        }
    }
}

static void DrawRightSidebar(AudioEngine& engine, TunerTracker& tunerTracker,
                             const TunerReading& tuner, float smoothedCents,
                             float smoothedHz, int x, int y, int w, int h) {
    (void)engine;
    (void)tunerTracker;
    DrawRectangle(x, y, w, h, ui_col::kSidebarBg);
    DrawLine(x, y, x, y + h, ui_col::kGrid);
    const char* noteText = tuner.active ? TextFormat("%s%d (%.1f Hz, %+.1f ct)",
                                                     tuner.noteName, tuner.octave, smoothedHz, smoothedCents)
                                        : "TUNER: PLAY A NOTE";
    DrawText(noteText, x + 16, y + 20, 14, tuner.active ? ui_col::kTuneGreen : GRAY);
}

static void DrawBottomMeterBar(const AudioEngine& engine, int screenW, int screenH,
                               float inDb, float outDb, bool limitLed, bool adcClipLed) {
    (void)limitLed;
    (void)adcClipLed;
    constexpr int kBarH = 46;
    const int     y     = screenH - kBarH;
    DrawRectangle(0, y, screenW, kBarH, ui_col::kTopBarBg);
    DrawLine(0, y, screenW, y, ui_col::kGrid);
    DrawText(TextFormat("IN: %.1f dB   |   COMP GR: -%.1f dB   |   OUT: %.1f dB",
                        inDb, engine.CompGainReductionDb(), outDb),
             20, y + 16, 14, LIGHTGRAY);
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
    InitWindow(cfg::kWidth, cfg::kHeight, "SubHz - Studio Bass DSP Rig & Visualizer");
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
    std::array<float, cfg::kNumBars> smoothedNormBars{};

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

    int   activeKnobId  = -1;
    int   dragRackSlot  = -1;
    float dragStartX    = 0.0f;
    bool  dragMoved     = false;

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

        // 1. Keyboard Controls
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
        if (IsKeyPressed(KEY_X))       engine->CycleOverdriveOversample();
        if (IsKeyPressed(KEY_S))       engine->SaveRigPresetJson(cfg::kPresetJsonFile);
        if (IsKeyPressed(KEY_L))       engine->LoadRigPresetJson(cfg::kPresetJsonFile);
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

        // Poll engine status notices
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

        // 3. DRY tap -> tuner
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
            const float norm = std::clamp((db - cfg::kMinDb) / (cfg::kMaxDb - cfg::kMinDb), 0.0f, 1.0f);
            smoothedNormBars[b] = TimeConstantLerp(smoothedNormBars[b], norm, cfg::kSpectrumSmoothRate, dt);
        }

        // ---- Layout Geometry (AmpliTube-inspired 4-zone studio workspace) -----------
        constexpr int kTopBarH = 38;
        constexpr int kBotBarH = 46;
        const int     sideW    = std::clamp(static_cast<int>(screenW * 0.24f), 276, 330);
        const int     stageW   = screenW - sideW;
        const int     bodyY    = kTopBarH;
        const int     bodyH    = screenH - kTopBarH - kBotBarH;
        const int     ampH     = std::clamp(static_cast<int>(bodyH * 0.64f), 260, bodyH - 150);
        const int     chainY   = bodyY + ampH;
        const int     chainH   = bodyH - ampH;

        // ---- Draw -------------------------------------------------------------------
        BeginDrawing();
        ClearBackground(ui_col::kBackground);

        DrawAmpVisualizerCabinet(*engine, 8, bodyY + 6, stageW - 16, ampH - 8,
                                 wetSamples, smoothedNormBars, tuner, smoothedHz, smoothedCents,
                                 wavePoints, activeKnobId, bannerText, bannerWarning, bannerTimer);

        DrawSignalChainStage(*engine, 0, chainY, stageW, chainH,
                             dragRackSlot, dragStartX, dragMoved);

        DrawRightSidebar(*engine, tunerTracker, tuner, smoothedCents, smoothedHz,
                         stageW, bodyY, sideW, bodyH);

        DrawTopToolbar(*engine, screenW, vsyncOn);
        DrawBottomMeterBar(*engine, screenW, screenH, inMeterDb, outMeterDb,
                           limitLedTime > 0.0f, adcClipTime > 0.0f);

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
