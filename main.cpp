// =====================================================================================
//  SubHz - Phase 3: Real-Time Chromatic Bass Tuner & FFT Spectrum Visualizer
//  Real-time bass guitar visualizer / tuner (C++20, raylib, miniaudio)
//
//  - 1280x720 window @ 180 FPS, background #121212
//  - miniaudio capture device: f32, mono, 48 kHz -> thread-safe 4096-sample ring buffer
//  - Hann window + custom in-place Cooley-Tukey Radix-2 FFT (<complex>, <vector>)
//  - Targeted bass tuner (30 - 400 Hz) with noise gate, Hann-calibrated parabolic
//    bin interpolation, 12*log2(f/440) pitch math, and top-center tuning dial
//  - Logarithmic spectrum (20 Hz - 10 kHz) with std::lerp temporal smoothing
// =====================================================================================

// -------------------------------------------------------------------------------------
//  HEADER GUARDS: raylib.h + miniaudio.h in the same translation unit (Windows)
// -------------------------------------------------------------------------------------
#if defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #define Rectangle   Win32_Rectangle
    #define CloseWindow Win32_CloseWindow
    #define ShowCursor  Win32_ShowCursor
#endif

// Trim miniaudio down to the device I/O layer only and keep linkage internal (MA_API
// static) so symbols never collide with raylib's bundled raudio module.
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

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstring>
#include <mutex>
#include <vector>

// -------------------------------------------------------------------------------------
//  Config
// -------------------------------------------------------------------------------------
namespace cfg {
constexpr int       kWidth          = 1280;
constexpr int       kHeight         = 720;
constexpr int       kTargetFps      = 180;
constexpr ma_uint32 kSampleRate     = 48000;
constexpr ma_uint32 kChannels       = 1;
constexpr size_t    kFftSize        = 4096;            // Power of 2 (bin width = 11.72 Hz)
constexpr size_t    kHalfBins       = kFftSize / 2;    // Up to Nyquist (24 kHz)

// Gain controls
constexpr float     kGainMin        = 0.1f;            // -20 dB
constexpr float     kGainMax        = 100.0f;          // +40 dB
constexpr float     kGainStepDb     = 1.0f;

// Spectrum analyzer settings (20 Hz - 10 kHz log scale devotes ~74% width to 20 - 2000 Hz)
constexpr int       kNumBars        = 160;
constexpr float     kMinFreq        = 20.0f;
constexpr float     kMaxFreq        = 10000.0f;        // Ignore bins above 10 kHz
constexpr float     kMinDb          = -65.0f;          // Noise floor cutoff
constexpr float     kMaxDb          = 0.0f;            // Full-scale ceiling
constexpr float     kSmoothing      = 0.15f;           // std::lerp factor per frame

// Chromatic bass tuner settings
constexpr float     kTunerMinHz     = 30.0f;           // Covers low B0 (30.87 Hz)
constexpr float     kTunerMaxHz     = 400.0f;          // Bass fundamental upper bound
constexpr float     kTunerGateMag   = 0.006f;          // Noise-floor gate (~ -44.4 dBFS)
constexpr float     kInTuneCents    = 5.0f;            // +/- 5 cents -> bright green
constexpr float     kTunerLerp      = 0.20f;           // Needle smoothing per frame

constexpr double    kPi             = 3.14159265358979323846;

const     Color     kBackground     = { 0x12, 0x12, 0x12, 0xFF };   // #121212
const     Color     kPanelBg        = { 0x1A, 0x1A, 0x1A, 0xFF };
const     Color     kNeon           = { 0x00, 0xFF, 0xC8, 0xFF };   // neon cyan-green
const     Color     kNeonLow        = { 0x00, 0xD2, 0xFF, 0xFF };   // sub-bass cyan
const     Color     kNeonHot        = { 0xAD, 0xFF, 0x2F, 0xFF };   // upper-mid lime
const     Color     kTuneGreen      = { 0x00, 0xFF, 0x66, 0xFF };   // in-tune bright green
const     Color     kTuneOrange     = { 0xFF, 0x9F, 0x1C, 0xFF };   // slightly sharp/flat
const     Color     kTuneRed        = { 0xFF, 0x3B, 0x30, 0xFF };   // far out of tune
const     Color     kGrid           = { 0x2A, 0x2A, 0x2A, 0xFF };
}  // namespace cfg

// -------------------------------------------------------------------------------------
//  Thread-safe circular buffer (4096 floats)
// -------------------------------------------------------------------------------------
template <size_t N>
class RingBuffer {
public:
    void Push(const float* src, size_t count) {
        if (src == nullptr || count == 0) return;
        if (count > N) {
            src  += count - N;
            count = N;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const size_t first = std::min(count, N - writePos_);
        std::memcpy(&data_[writePos_], src, first * sizeof(float));
        std::memcpy(&data_[0], src + first, (count - first) * sizeof(float));
        writePos_ = (writePos_ + count) % N;
    }

    void Snapshot(std::array<float, N>& out) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const size_t tail = N - writePos_;
        std::memcpy(out.data(), &data_[writePos_], tail * sizeof(float));
        std::memcpy(out.data() + tail, &data_[0], writePos_ * sizeof(float));
    }

private:
    mutable std::mutex   mutex_;
    std::array<float, N> data_{};
    size_t               writePos_ = 0;
};

using SampleRing = RingBuffer<cfg::kFftSize>;

// -------------------------------------------------------------------------------------
//  miniaudio capture callback (real-time audio thread)
// -------------------------------------------------------------------------------------
static void OnAudioData(ma_device* device, void* /*output*/, const void* input,
                        ma_uint32 frameCount) {
    auto* ring = static_cast<SampleRing*>(device->pUserData);
    ring->Push(static_cast<const float*>(input), frameCount * cfg::kChannels);
}

// -------------------------------------------------------------------------------------
//  Self-contained iterative Radix-2 Cooley-Tukey FFT (in-place, zero heap allocs/frame)
//  Input size N must be a power of 2.
// -------------------------------------------------------------------------------------
static void CooleyTukeyFFT(std::vector<std::complex<float>>& a) {
    const size_t n = a.size();

    // 1. Bit-reversal permutation
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap(a[i], a[j]);
        }
    }

    // 2. Danielson-Lanczos butterfly stages
    for (size_t len = 2; len <= n; len <<= 1) {
        const double angle = -2.0 * cfg::kPi / static_cast<double>(len);
        const std::complex<double> wLen(std::cos(angle), std::sin(angle));
        const size_t half = len >> 1;

        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (size_t j = 0; j < half; ++j) {
                const std::complex<float> twiddle(static_cast<float>(w.real()),
                                                  static_cast<float>(w.imag()));
                const std::complex<float> u = a[i + j];
                const std::complex<float> v = a[i + j + half] * twiddle;
                a[i + j]        = u + v;
                a[i + j + half] = u - v;
                w *= wLen;
            }
        }
    }
}

// -------------------------------------------------------------------------------------
//  Tuner DSP: Targeted Peak Detection (30 - 400 Hz), Parabolic Interpolation, Pitch Math
// -------------------------------------------------------------------------------------
struct TunerReading {
    bool        active      = false;
    float       frequencyHz = 0.0f;
    const char* noteName    = "--";
    int         octave      = 0;
    int         midiNote    = -1;
    float       cents       = 0.0f;    // [-50, +50]
};

static TunerReading DetectBassPitch(const std::vector<float>& magnitudes,
                                    float binWidthHz) {
    static constexpr const char* kNoteNames[12] = {
        "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
    };

    TunerReading out{};
    if (magnitudes.size() <= 2 || binWidthHz <= 0.0f) return out;

    // Scan strictly between 30 Hz and 400 Hz (keep 1-bin margin for peak +/- 1).
    const size_t minBin = std::max<size_t>(
        1, static_cast<size_t>(std::ceil(cfg::kTunerMinHz / binWidthHz)));
    const size_t maxBin = std::min<size_t>(
        magnitudes.size() - 2,
        static_cast<size_t>(std::floor(cfg::kTunerMaxHz / binWidthHz)));
    if (minBin > maxBin) return out;

    size_t peakBin = minBin;
    float  peakMag = magnitudes[minBin];
    for (size_t k = minBin + 1; k <= maxBin; ++k) {
        if (magnitudes[k] > peakMag) {
            peakMag = magnitudes[k];
            peakBin = k;
        }
    }

    // Noise gate: ignore peaks below threshold so room noise doesn't trigger the tuner.
    if (peakMag < cfg::kTunerGateMag) return out;

    // Parabolic interpolation across (peakBin - 1, peakBin, peakBin + 1).
    // Raising Hann-windowed magnitudes to p = 0.23 makes the Hann main lobe closely
    // parabolic, cutting sub-cent interpolation bias down from ~30 ct to < 1 ct at 30 Hz.
    constexpr double kHannPower = 0.23;
    const double y1 = std::pow(std::max(static_cast<double>(magnitudes[peakBin - 1]), 1e-12), kHannPower);
    const double y2 = std::pow(std::max(static_cast<double>(magnitudes[peakBin]),     1e-12), kHannPower);
    const double y3 = std::pow(std::max(static_cast<double>(magnitudes[peakBin + 1]), 1e-12), kHannPower);

    const double denom = y1 - 2.0 * y2 + y3;
    double delta = 0.0;
    if (std::fabs(denom) > 1e-12) {
        delta = std::clamp(0.5 * (y1 - y3) / denom, -0.5, 0.5);
    }

    const double interpFreq = (static_cast<double>(peakBin) + delta) *
                              static_cast<double>(binWidthHz);
    if (interpFreq < cfg::kTunerMinHz || interpFreq > cfg::kTunerMaxHz) return out;

    // Pitch math: n = 12 * log2(freq / 440.0) semitones from A4 (MIDI 69).
    const double n               = 12.0 * std::log2(interpFreq / 440.0);
    const int    nearestSemitone = static_cast<int>(std::lround(n));
    const float  cents           = static_cast<float>((n - nearestSemitone) * 100.0);

    const int midiNote  = nearestSemitone + 69;
    const int noteIndex = ((midiNote % 12) + 12) % 12;
    const int octave    = static_cast<int>(std::floor(midiNote / 12.0)) - 1;

    out.active      = true;
    out.frequencyHz = static_cast<float>(interpFreq);
    out.noteName    = kNoteNames[noteIndex];
    out.octave      = octave;
    out.midiNote    = midiNote;
    out.cents       = std::clamp(cents, -50.0f, 50.0f);
    return out;
}

// Helper: map frequency in [kMinFreq, kMaxFreq] to normalized [0, 1] log X coordinate.
static float FreqToNormLogX(float freqHz) {
    const float logMin = std::log10(cfg::kMinFreq);
    const float logMax = std::log10(cfg::kMaxFreq);
    return (std::log10(freqHz) - logMin) / (logMax - logMin);
}

// Helper: linear color blend (works on both Raylib 5.0 and 6.0).
static Color BlendColor(Color a, Color b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return Color{
        static_cast<unsigned char>(std::round(std::lerp(static_cast<float>(a.r), static_cast<float>(b.r), t))),
        static_cast<unsigned char>(std::round(std::lerp(static_cast<float>(a.g), static_cast<float>(b.g), t))),
        static_cast<unsigned char>(std::round(std::lerp(static_cast<float>(a.b), static_cast<float>(b.b), t))),
        static_cast<unsigned char>(std::round(std::lerp(static_cast<float>(a.a), static_cast<float>(b.a), t))),
    };
}

// -------------------------------------------------------------------------------------
//  Tuner UI Overlay (top center)
// -------------------------------------------------------------------------------------
static void DrawTunerOverlay(const TunerReading& tuner, float smoothedCents,
                             float smoothedHz) {
    constexpr int kBoxW = 420;
    constexpr int kBoxH = 118;
    constexpr int kBoxX = (cfg::kWidth - kBoxW) / 2;   // 430
    constexpr int kBoxY = 10;
    constexpr int kCenterX = cfg::kWidth / 2;          // 640

    Color statusCol = DARKGRAY;
    if (tuner.active) {
        const float absCents = std::fabs(smoothedCents);
        if (absCents <= cfg::kInTuneCents) {
            statusCol = cfg::kTuneGreen;
        } else {
            // Smoothly shift from orange (just outside +/-5 ct) to red (>= 25 ct)
            const float severity = std::clamp((absCents - cfg::kInTuneCents) / 20.0f,
                                              0.0f, 1.0f);
            statusCol = BlendColor(cfg::kTuneOrange, cfg::kTuneRed, severity);
        }
    }

    // Panel background & status-colored border
    DrawRectangle(kBoxX, kBoxY, kBoxW, kBoxH, cfg::kPanelBg);
    DrawRectangleLines(kBoxX, kBoxY, kBoxW, kBoxH,
                       tuner.active ? Fade(statusCol, 0.85f) : cfg::kGrid);

    // Large centered Note + Octave text (e.g. "E1", "A#1")
    const char* noteText = tuner.active
                               ? TextFormat("%s%d", tuner.noteName, tuner.octave)
                               : "--";
    constexpr int kNoteFontSize = 44;
    const int noteW = MeasureText(noteText, kNoteFontSize);
    DrawText(noteText, kCenterX - noteW / 2, kBoxY + 10, kNoteFontSize,
             tuner.active ? statusCol : GRAY);

    // Flanking readouts: Hz on the left, Cents / status on the right
    if (tuner.active) {
        DrawText(TextFormat("%.1f Hz", smoothedHz), kBoxX + 16, kBoxY + 24, 18, LIGHTGRAY);
        const bool inTune = std::fabs(smoothedCents) <= cfg::kInTuneCents;
        const char* centsTxt = inTune ? TextFormat("%+.1f ct  OK", smoothedCents)
                                      : TextFormat("%+.1f ct", smoothedCents);
        const int centsW = MeasureText(centsTxt, 18);
        DrawText(centsTxt, kBoxX + kBoxW - centsW - 16, kBoxY + 24, 18, statusCol);
    } else {
        DrawText("30 - 400 Hz", kBoxX + 16, kBoxY + 26, 16, DARKGRAY);
        const char* idleTxt = "PLAY A NOTE";
        DrawText(idleTxt, kBoxX + kBoxW - MeasureText(idleTxt, 16) - 16,
                 kBoxY + 26, 16, DARKGRAY);
    }

    // Horizontal tuning gauge / dial (-50 to +50 cents)
    constexpr int   kGaugeW    = 350;                  // 3.5 px per cent
    constexpr int   kGaugeX    = kCenterX - kGaugeW / 2;
    constexpr int   kGaugeY    = kBoxY + 76;
    constexpr int   kGaugeH    = 14;
    constexpr float kPxPerCent = kGaugeW / 100.0f;

    // Dial track & +/- 5 cents green target window
    DrawRectangle(kGaugeX, kGaugeY, kGaugeW, kGaugeH, Color{ 0x10, 0x10, 0x10, 0xFF });
    const int inTuneHalfW = static_cast<int>(std::round(cfg::kInTuneCents * kPxPerCent));
    DrawRectangle(kCenterX - inTuneHalfW, kGaugeY, inTuneHalfW * 2, kGaugeH,
                  Fade(cfg::kTuneGreen, tuner.active ? 0.25f : 0.10f));
    DrawRectangleLines(kGaugeX, kGaugeY, kGaugeW, kGaugeH, cfg::kGrid);

    // Dial tick marks (-50, -25, -5, 0, +5, +25, +50)
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

    // Active needle & deviation bar
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
//  main
// -------------------------------------------------------------------------------------
int main() {
    // ---- Window ---------------------------------------------------------------------
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(cfg::kWidth, cfg::kHeight, "SubHz - Bass Tuner & Spectrum Visualizer");
    SetTargetFPS(cfg::kTargetFps);

    // ---- Audio capture --------------------------------------------------------------
    SampleRing ring;
    ma_device  device;
    bool       audioOk = false;
    char       deviceName[MA_MAX_DEVICE_NAME_LENGTH + 1] = "none";

    ma_device_config config   = ma_device_config_init(ma_device_type_capture);
    config.capture.format     = ma_format_f32;
    config.capture.channels   = cfg::kChannels;
    config.sampleRate         = cfg::kSampleRate;
    config.periodSizeInFrames = 256;
    config.dataCallback       = OnAudioData;
    config.pUserData          = &ring;

    if (ma_device_init(nullptr, &config, &device) == MA_SUCCESS) {
        if (ma_device_start(&device) == MA_SUCCESS) {
            audioOk = true;
            ma_device_get_name(&device, ma_device_type_capture, deviceName,
                               sizeof(deviceName), nullptr);
        } else {
            ma_device_uninit(&device);
        }
    }
    if (!audioOk) TraceLog(LOG_WARNING, "SubHz: failed to open capture device");

    // ---- Precomputed Hann window & working buffers ----------------------------------
    std::vector<float> hannWindow(cfg::kFftSize);
    for (size_t i = 0; i < cfg::kFftSize; ++i) {
        hannWindow[i] = static_cast<float>(
            0.5 * (1.0 - std::cos(2.0 * cfg::kPi * static_cast<double>(i) /
                                  static_cast<double>(cfg::kFftSize - 1))));
    }

    std::vector<std::complex<float>> fftBuffer(cfg::kFftSize);
    std::vector<float>               magnitudes(cfg::kHalfBins + 1, 0.0f);
    std::array<float, cfg::kNumBars> previous_frame_heights{};   // Temporal smoothing state

    static std::array<float, cfg::kFftSize>   samples{};
    static std::array<Vector2, cfg::kFftSize> wavePoints{};

    float gain          = 1.0f;
    int   lastMidiNote  = -1;
    float smoothedCents = 0.0f;
    float smoothedHz    = 0.0f;

    const float effectiveRate = audioOk ? static_cast<float>(device.sampleRate)
                                        : static_cast<float>(cfg::kSampleRate);
    const float binWidthHz    = effectiveRate / static_cast<float>(cfg::kFftSize);

    // Layout regions
    constexpr float kMarginX  = 24.0f;
    constexpr float kPlotW    = cfg::kWidth - 2.0f * kMarginX;
    constexpr float kWaveTop  = 142.0f;
    constexpr float kWaveH    = 88.0f;
    constexpr float kSpecTop  = 256.0f;
    constexpr float kSpecBot  = 680.0f;
    constexpr float kSpecMaxH = kSpecBot - kSpecTop;

    // ---- Main loop ------------------------------------------------------------------
    while (!WindowShouldClose()) {
        // 1. Gain control (Up/Down in 1 dB steps, R to reset)
        const float step = std::pow(10.0f, cfg::kGainStepDb / 20.0f);
        if (IsKeyPressed(KEY_UP)   || IsKeyPressedRepeat(KEY_UP))   gain *= step;
        if (IsKeyPressed(KEY_DOWN) || IsKeyPressedRepeat(KEY_DOWN)) gain /= step;
        if (IsKeyPressed(KEY_R)) gain = 1.0f;
        gain = std::clamp(gain, cfg::kGainMin, cfg::kGainMax);

        // 2. Pull latest 4096 samples from ring buffer, apply gain, compute RMS & peak
        ring.Snapshot(samples);
        double sumSq = 0.0;
        float  peak  = 0.0f;
        for (float& s : samples) {
            s *= gain;
            sumSq += static_cast<double>(s) * s;
            peak = std::max(peak, std::fabs(s));
        }
        const float rms   = static_cast<float>(std::sqrt(sumSq / cfg::kFftSize));
        const float rmsDb = 20.0f * std::log10(std::max(rms, 1e-9f));

        // 3. Apply Hann window & run Cooley-Tukey FFT
        for (size_t i = 0; i < cfg::kFftSize; ++i) {
            fftBuffer[i] = { samples[i] * hannWindow[i], 0.0f };
        }
        CooleyTukeyFFT(fftBuffer);

        // 4. Magnitude calculation for the first half of FFT bins (up to Nyquist).
        constexpr float kNormScale = 4.0f / static_cast<float>(cfg::kFftSize);
        magnitudes[0] = std::abs(fftBuffer[0]) * (2.0f / static_cast<float>(cfg::kFftSize));
        for (size_t k = 1; k <= cfg::kHalfBins; ++k) {
            magnitudes[k] = std::abs(fftBuffer[k]) * kNormScale;
        }

        // 5. Targeted bass pitch detection (30 - 400 Hz) + needle smoothing
        const TunerReading tuner = DetectBassPitch(magnitudes, binWidthHz);
        if (tuner.active) {
            if (tuner.midiNote != lastMidiNote) {
                smoothedCents = tuner.cents;
                smoothedHz    = tuner.frequencyHz;
                lastMidiNote  = tuner.midiNote;
            } else {
                smoothedCents = std::lerp(smoothedCents, tuner.cents, cfg::kTunerLerp);
                smoothedHz    = std::lerp(smoothedHz, tuner.frequencyHz, cfg::kTunerLerp);
            }
        } else {
            lastMidiNote = -1;
        }

        // 6. Map log-spaced frequency bands [20 Hz .. 10 kHz] & smooth bar heights
        const float logMin      = std::log10(cfg::kMinFreq);
        const float logMax      = std::log10(cfg::kMaxFreq);
        const float maxValidBin = std::min(static_cast<float>(cfg::kHalfBins),
                                           cfg::kMaxFreq / binWidthHz);

        for (int b = 0; b < cfg::kNumBars; ++b) {
            const float t0 = static_cast<float>(b) / cfg::kNumBars;
            const float t1 = static_cast<float>(b + 1) / cfg::kNumBars;
            const float f0 = std::pow(10.0f, logMin + t0 * (logMax - logMin));
            const float f1 = std::pow(10.0f, logMin + t1 * (logMax - logMin));
            const float fc = 0.5f * (f0 + f1);

            const float bin0 = std::clamp(f0 / binWidthHz, 1.0f, maxValidBin);
            const float bin1 = std::clamp(f1 / binWidthHz, 1.0f, maxValidBin);

            float bandMag = 0.0f;
            if (bin1 - bin0 <= 1.0f) {
                const float binCenter = std::clamp(fc / binWidthHz, 1.0f, maxValidBin);
                const size_t k0 = static_cast<size_t>(std::floor(binCenter));
                const size_t k1 = std::min(k0 + 1, cfg::kHalfBins);
                const float frac = binCenter - static_cast<float>(k0);
                bandMag = std::lerp(magnitudes[k0], magnitudes[k1], frac);
            } else {
                const size_t kStart = static_cast<size_t>(std::floor(bin0));
                const size_t kEnd   = std::min(static_cast<size_t>(std::ceil(bin1)),
                                               cfg::kHalfBins);
                for (size_t k = kStart; k <= kEnd; ++k) {
                    bandMag = std::max(bandMag, magnitudes[k]);
                }
            }

            const float db   = 20.0f * std::log10(std::max(bandMag, 1e-9f));
            const float norm = std::clamp((db - cfg::kMinDb) / (cfg::kMaxDb - cfg::kMinDb),
                                          0.0f, 1.0f);
            const float targetHeight = norm * kSpecMaxH;

            previous_frame_heights[b] =
                std::lerp(previous_frame_heights[b], targetHeight, cfg::kSmoothing);
        }

        // 7. Build compact time-domain oscilloscope strip
        const float waveCenterY = kWaveTop + kWaveH * 0.5f;
        const float waveAmpPx   = kWaveH * 0.45f;
        const float waveDx      = kPlotW / static_cast<float>(cfg::kFftSize - 1);
        for (size_t i = 0; i < cfg::kFftSize; ++i) {
            const float v = std::clamp(samples[i], -1.1f, 1.1f);
            wavePoints[i] = { kMarginX + i * waveDx, waveCenterY - v * waveAmpPx };
        }

        // ---- Draw -------------------------------------------------------------------
        BeginDrawing();
        ClearBackground(cfg::kBackground);

        // HUD (top-left, compact width so it never overlaps the centered tuner box)
        const bool clipping = peak >= 1.0f;
        DrawText(TextFormat("FPS: %d", GetFPS()), 24, 14, 18, RAYWHITE);
        DrawText(TextFormat("RMS: %.1f dBFS (%.4f)", rmsDb, rms), 130, 14, 18,
                 clipping ? RED : cfg::kNeon);
        DrawText(TextFormat("Gain: x%.2f (%+.1f dB) [Up/Down, R]", gain,
                            20.0f * std::log10(gain)), 24, 40, 18, LIGHTGRAY);

        constexpr int kRmsBarW = 360;
        DrawRectangleLines(24, 68, kRmsBarW, 10, cfg::kGrid);
        DrawRectangle(25, 69,
                      static_cast<int>(std::min(rms * 2.0f, 1.0f) * (kRmsBarW - 2)), 8,
                      clipping ? RED : cfg::kNeon);

        if (audioOk) {
            DrawText(TextFormat("Mic: %.36s (%u Hz)", deviceName, device.sampleRate),
                     24, 88, 14, GRAY);
        } else {
            DrawText("NO CAPTURE DEVICE - check mic permissions", 24, 88, 14, RED);
        }
        if (clipping) DrawText("CLIP", cfg::kWidth - 84, 16, 24, RED);

        // Top-center Tuner UI Overlay
        DrawTunerOverlay(tuner, smoothedCents, smoothedHz);

        // Compact oscilloscope panel
        DrawRectangleLines(static_cast<int>(kMarginX), static_cast<int>(kWaveTop),
                           static_cast<int>(kPlotW), static_cast<int>(kWaveH), cfg::kGrid);
        DrawLine(static_cast<int>(kMarginX), static_cast<int>(waveCenterY),
                 static_cast<int>(kMarginX + kPlotW), static_cast<int>(waveCenterY),
                 cfg::kGrid);
        DrawLineStrip(wavePoints.data(), static_cast<int>(cfg::kFftSize),
                      Fade(cfg::kNeon, 0.85f));
        DrawText("TIME DOMAIN (4096 samples)", static_cast<int>(kMarginX) + 8,
                 static_cast<int>(kWaveTop) + 6, 12, GRAY);

        // Spectrum backdrop & subtle highlight on the 20 Hz - 2 kHz bass focus zone
        const float bassZoneW = FreqToNormLogX(2000.0f) * kPlotW;
        DrawRectangle(static_cast<int>(kMarginX), static_cast<int>(kSpecTop),
                      static_cast<int>(bassZoneW), static_cast<int>(kSpecMaxH),
                      Color{ 0x00, 0xFF, 0xC8, 0x07 });
        DrawRectangleLines(static_cast<int>(kMarginX), static_cast<int>(kSpecTop),
                           static_cast<int>(kPlotW), static_cast<int>(kSpecMaxH), cfg::kGrid);

        // Horizontal dB grid lines (-15, -30, -45 dBFS)
        for (int dbTick : { -15, -30, -45 }) {
            const float norm = (static_cast<float>(dbTick) - cfg::kMinDb) /
                               (cfg::kMaxDb - cfg::kMinDb);
            const int y = static_cast<int>(kSpecBot - norm * kSpecMaxH);
            DrawLine(static_cast<int>(kMarginX), y,
                     static_cast<int>(kMarginX + kPlotW), y, Fade(cfg::kGrid, 0.65f));
            DrawText(TextFormat("%d dB", dbTick), static_cast<int>(kMarginX) + 6, y + 3,
                     12, DARKGRAY);
        }

        // Vertical logarithmic frequency ticks & bass string landmarks
        struct FreqTick { float hz; const char* label; };
        constexpr FreqTick kTicks[] = {
            { 20.0f,    "20Hz"  },
            { 30.87f,   "B0"    },
            { 41.20f,   "E1"    },
            { 55.0f,    "A1"    },
            { 73.42f,   "D2"    },
            { 98.0f,    "G2"    },
            { 200.0f,   "200"   },
            { 500.0f,   "500"   },
            { 1000.0f,  "1k"    },
            { 2000.0f,  "2k"    },
            { 5000.0f,  "5k"    },
            { 10000.0f, "10kHz" },
        };
        for (const auto& tick : kTicks) {
            const int x = static_cast<int>(std::round(
                kMarginX + FreqToNormLogX(tick.hz) * kPlotW));
            const bool isNote = (tick.hz > 25.0f && tick.hz < 110.0f);
            DrawLine(x, static_cast<int>(kSpecTop), x, static_cast<int>(kSpecBot),
                     isNote ? Color{ 0x00, 0xFF, 0xC8, 0x22 } : Fade(cfg::kGrid, 0.7f));
            DrawText(tick.label, x - 10, static_cast<int>(kSpecBot) + 8, 14,
                     isNote ? cfg::kNeon : GRAY);
        }
        DrawText("BASS FOCUS ZONE (20 Hz - 2 kHz)", static_cast<int>(kMarginX) + 8,
                 static_cast<int>(kSpecTop) + 6, 12, Fade(cfg::kNeon, 0.65f));
        DrawText("HARMONICS / ATTACK (2k - 10k)",
                 static_cast<int>(kMarginX + bassZoneW) + 8,
                 static_cast<int>(kSpecTop) + 6, 12, GRAY);

        // Spectrum bars via DrawRectangle
        const float slotW = kPlotW / static_cast<float>(cfg::kNumBars);
        const int   barW  = std::max(1, static_cast<int>(std::floor(slotW)) - 2);
        for (int b = 0; b < cfg::kNumBars; ++b) {
            const int barH = static_cast<int>(std::round(previous_frame_heights[b]));
            if (barH <= 0) continue;

            const int x = static_cast<int>(std::round(kMarginX + b * slotW)) + 1;
            const int y = static_cast<int>(kSpecBot) - barH;

            const float freqFrac = static_cast<float>(b) / (cfg::kNumBars - 1);
            const float ampFrac  = std::clamp(barH / kSpecMaxH, 0.0f, 1.0f);
            const Color baseCol  = BlendColor(cfg::kNeonLow, cfg::kNeon,
                                              std::min(freqFrac * 1.4f, 1.0f));
            const Color barCol   = BlendColor(baseCol, cfg::kNeonHot, ampFrac * 0.45f);

            DrawRectangle(x, y, barW, barH, barCol);
        }

        // Tracked fundamental cursor on the spectrum plot
        if (tuner.active) {
            const int peakX = static_cast<int>(std::round(
                kMarginX + FreqToNormLogX(smoothedHz) * kPlotW));
            const bool inTune = std::fabs(smoothedCents) <= cfg::kInTuneCents;
            DrawLine(peakX, static_cast<int>(kSpecTop), peakX, static_cast<int>(kSpecBot),
                     Fade(inTune ? cfg::kTuneGreen : cfg::kTuneOrange, 0.6f));
        }

        EndDrawing();
    }

    // ---- Shutdown -------------------------------------------------------------------
    if (audioOk) ma_device_uninit(&device);
    CloseWindow();
    return 0;
}
