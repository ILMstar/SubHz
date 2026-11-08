// =====================================================================================
//  SubHz - Full-Duplex Audio & Bass DSP Rig, Visualizer & Tuner (C++17, raylib, miniaudio)
//
//  Audio thread (WASAPI duplex, shared mode default for mic APOs, exclusive fallback):
//    IN (1+2 / 1 / 2) -> Trim -> [DcBlocker: HPF 28 Hz]
//                                   |  (DC-blocked, pre-FX) -> dryBuffer -> Tuner
//                                   v
//                              [EnvelopeNoiseGate (G)] -> [CabSim: LPF 4.5 kHz (C)]
//                                   v
//                              Soft-knee tanh limiter -> wetBuffer -> Scope + FFT
//                                   v
//                              Mute ramp -> OUT (dual mono L/R)
//
//  UI thread (raylib, 180 FPS): lock-free snapshots of both taps, never blocks audio.
// =====================================================================================

// -------------------------------------------------------------------------------------
//  HEADER GUARDS: raylib.h + miniaudio.h in the same translation unit (Windows)
//  <windows.h> (pulled in by miniaudio) declares Rectangle/CloseWindow/ShowCursor and
//  DrawText/DrawTextEx/LoadImage/PlaySound macros that clash with raylib's API.
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
#include <atomic>
#include <cmath>
#include <complex>
#include <cstddef>
#include <memory>
#include <vector>

#if defined(__SSE__) || defined(_M_X64) || defined(_M_IX86)
    #include <xmmintrin.h>
    #define SUBHZ_HAS_SSE 1
#endif

// -------------------------------------------------------------------------------------
//  Config
// -------------------------------------------------------------------------------------
namespace cfg {
// Window
constexpr int       kWidth          = 1280;
constexpr int       kHeight         = 720;
constexpr int       kTargetFps      = 180;

// Audio device (WASAPI full duplex)
constexpr ma_uint32 kSampleRate     = 48000;
constexpr ma_uint32 kChannelsIn     = 2;
constexpr ma_uint32 kChannelsOut    = 2;
constexpr ma_uint32 kPeriodFrames   = 128;
constexpr ma_uint32 kPeriods        = 1;

// Analysis
constexpr size_t    kFftSize        = 4096;            // Power of 2 (bin width = 11.72 Hz)
constexpr size_t    kHalfBins       = kFftSize / 2;    // Up to Nyquist (24 kHz)

// Input trim
constexpr float     kTrimMin        = 0.1f;            // -20 dB
constexpr float     kTrimMax        = 100.0f;          // +40 dB
constexpr float     kTrimStepDb     = 1.0f;

// DSP chain
constexpr float     kDcBlockHz      = 28.0f;
constexpr float     kCabLpfHz       = 4500.0f;
constexpr float     kCabToggleMs    = 10.0f;           // click-free cab crossfade
constexpr float     kGateOpenDb     = -54.0f;          // hysteresis: open above ...
constexpr float     kGateCloseDb    = -62.0f;          // ... close below (8 dB window)
constexpr float     kGateAttackMs   = 1.0f;
constexpr float     kGateReleaseMs  = 80.0f;
constexpr float     kLimiterKnee    = 0.70f;           // linear below (-3.1 dBFS)
constexpr float     kLimiterCeiling = 0.98f;           // asymptotic tanh ceiling
constexpr float     kMuteRampMs     = 5.0f;

// Spectrum analyzer (20 Hz - 10 kHz log scale devotes ~74% width to 20 - 2000 Hz)
constexpr int       kNumBars        = 160;
constexpr float     kMinFreq        = 20.0f;
constexpr float     kMaxFreq        = 10000.0f;        // Ignore bins above 10 kHz
constexpr float     kMinDb          = -65.0f;
constexpr float     kMaxDb          = 0.0f;
constexpr float     kSmoothing      = 0.15f;           // Lerp factor per frame

// Chromatic tuner (30 - 1200 Hz covers low B0 bass through full vocal range)
constexpr float     kTunerMinHz     = 30.0f;           // Covers low B0 (30.87 Hz)
constexpr float     kTunerMaxHz     = 1200.0f;         // Covers bass + voice up to D6
constexpr float     kTunerGateMag   = 0.002f;          // ~ -54.0 dBFS
constexpr float     kInTuneCents    = 5.0f;
constexpr float     kTunerLerp      = 0.20f;

// Meters
constexpr float     kMeterFloorDb       = -60.0f;
constexpr float     kMeterFallDbPerSec  = 24.0f;
constexpr float     kLimitLedHoldSec    = 0.25f;

constexpr double    kPi             = 3.14159265358979323846;

const     Color     kBackground     = { 0x12, 0x12, 0x12, 0xFF };   // #121212
const     Color     kStatusBg       = { 0x0B, 0x0B, 0x0B, 0xFF };
const     Color     kPanelBg        = { 0x1A, 0x1A, 0x1A, 0xFF };
const     Color     kNeon           = { 0x00, 0xFF, 0xC8, 0xFF };
const     Color     kNeonLow        = { 0x00, 0xD2, 0xFF, 0xFF };
const     Color     kNeonHot        = { 0xAD, 0xFF, 0x2F, 0xFF };
const     Color     kTuneGreen      = { 0x00, 0xFF, 0x66, 0xFF };
const     Color     kTuneOrange     = { 0xFF, 0x9F, 0x1C, 0xFF };
const     Color     kTuneRed        = { 0xFF, 0x3B, 0x30, 0xFF };
const     Color     kGrid           = { 0x2A, 0x2A, 0x2A, 0xFF };
}  // namespace cfg

// -------------------------------------------------------------------------------------
//  Small math helpers (C++17: no std::lerp)
// -------------------------------------------------------------------------------------
template <typename T>
constexpr T Lerp(T a, T b, T t) noexcept { return a + t * (b - a); }

inline float DbToLin(float db) noexcept { return std::pow(10.0f, db / 20.0f); }

// One-pole smoothing coefficient reaching ~63% of a step in `ms` milliseconds.
inline float OnePoleCoeff(float ms, float sampleRate) noexcept {
    return 1.0f - std::exp(-1.0f / (ms * 0.001f * sampleRate));
}

// Flush denormals to zero on the audio thread (IIR tails decaying into denormals
// can cost 100x CPU on x86).
struct ScopedDenormalGuard {
#if defined(SUBHZ_HAS_SSE)
    unsigned int previous;
    ScopedDenormalGuard() noexcept : previous(_mm_getcsr()) { _mm_setcsr(previous | 0x8040u); }
    ~ScopedDenormalGuard() { _mm_setcsr(previous); }
#endif
};

// -------------------------------------------------------------------------------------
//  LockFreeHistoryBuffer<N>: wait-free SPSC sliding window (no mutex, no try_lock)
//
//  - Producer (audio thread) writes blocks; a monotonically increasing 64-bit
//    writeIndex is published with memory_order_release.
//  - Consumer (UI thread) loads writeIndex with memory_order_acquire and copies the
//    latest N samples in chronological order.
//  - Backing store is 2N slots, so the producer must advance a full N samples (85 ms
//    at 48 kHz) during one copy before it can touch the window being read. A seqlock
//    style claim index detects that case and the reader simply retries.
//  - Slots are std::atomic<float> with relaxed ordering (plain MOVs on x86), so the
//    concurrent read/overwrite is well-defined rather than a data race.
// -------------------------------------------------------------------------------------
template <size_t N>
class LockFreeHistoryBuffer {
    static_assert(N > 0 && (N & (N - 1)) == 0, "N must be a power of two");
    static_assert(std::atomic<float>::is_always_lock_free, "atomic<float> must be lock-free");
    static_assert(std::atomic<size_t>::is_always_lock_free, "atomic<size_t> must be lock-free");

public:
    static constexpr size_t kWindow   = N;
    static constexpr size_t kCapacity = 2 * N;
    static constexpr size_t kMask     = kCapacity - 1;

    LockFreeHistoryBuffer() noexcept {
        for (auto& slot : slots_) slot.store(0.0f, std::memory_order_relaxed);
    }

    // Producer only (audio thread). Wait-free.
    void Write(const float* src, size_t count) noexcept {
        if (src == nullptr || count == 0) return;
        if (count > N) {                     // only the newest N samples can be observed
            src  += count - N;
            count = N;
        }
        const size_t w = writeIndex_.load(std::memory_order_relaxed);   // sole writer
        claimIndex_.store(w + count, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);            // claim before data
        for (size_t i = 0; i < count; ++i) {
            slots_[(w + i) & kMask].store(src[i], std::memory_order_relaxed);
        }
        writeIndex_.store(w + count, std::memory_order_release);        // publish data
    }

    // Consumer only (UI thread). Never blocks the producer. Returns false only if the
    // producer lapped the reader on every attempt (out may then contain torn data).
    bool Snapshot(std::array<float, N>& out) const noexcept {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const size_t w     = writeIndex_.load(std::memory_order_acquire);
            const size_t start = w - N;                   // modular; zeros before first fill
            for (size_t i = 0; i < N; ++i) {
                out[i] = slots_[(start + i) & kMask].load(std::memory_order_relaxed);
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            const size_t claimed = claimIndex_.load(std::memory_order_relaxed);
            // Writing absolute index j overwrites absolute index j - kCapacity, so the
            // window [start, w) is intact iff no index beyond start + kCapacity was claimed.
            if (claimed - start <= kCapacity) return true;
        }
        return false;
    }

    size_t TotalWritten() const noexcept { return writeIndex_.load(std::memory_order_acquire); }

private:
    alignas(64) std::atomic<size_t>             writeIndex_{0};
    alignas(64) std::atomic<size_t>             claimIndex_{0};
    alignas(64) std::array<std::atomic<float>, kCapacity> slots_;
};

// -------------------------------------------------------------------------------------
//  DSP node interface
// -------------------------------------------------------------------------------------
class IDspNode {
public:
    virtual ~IDspNode() = default;
    virtual float process(float sample, float sampleRate) noexcept = 0;
};

// -------------------------------------------------------------------------------------
//  DcBlockerNode: 1st-order DC-blocking HPF (28 Hz). Removes sub-sonic rumble and ADC
//  DC offset while remaining transparent across both bass (B0 = 30.87 Hz) and voice.
// -------------------------------------------------------------------------------------
class DcBlockerNode final : public IDspNode {
public:
    float process(float sample, float sampleRate) noexcept override {
        if (sampleRate != cachedRate_) {
            cachedRate_ = sampleRate;
            hpR_ = std::exp(-2.0 * cfg::kPi * cfg::kDcBlockHz / sampleRate);
        }
        const double hp = static_cast<double>(sample) - hpX1_ + hpR_ * hpY1_;
        hpX1_ = sample;
        hpY1_ = hp;
        lastOut_ = static_cast<float>(hp);
        return lastOut_;
    }

    float LastOutput() const noexcept { return lastOut_; }

private:
    float  cachedRate_ = 0.0f;
    double hpR_ = 0.0, hpX1_ = 0.0, hpY1_ = 0.0;
    float  lastOut_    = 0.0f;
};

// -------------------------------------------------------------------------------------
//  EnvelopeNoiseGateNode: peak envelope (1 ms attack / 80 ms release) drives a
//  hysteretic open/close decision (open > -54 dBFS, close < -62 dBFS). Placed BEFORE
//  the 4.5 kHz cab LPF so high-frequency vocal consonants trigger the detector cleanly.
//  Defaults to bypassed for clean voice capture; toggleable click-free with [G].
// -------------------------------------------------------------------------------------
class EnvelopeNoiseGateNode final : public IDspNode {
public:
    float process(float sample, float sampleRate) noexcept override {
        if (sampleRate != cachedRate_) UpdateCoefficients(sampleRate);

        const float level    = std::fabs(sample);
        const float envCoeff = level > envelope_ ? attackCoeff_ : releaseCoeff_;
        envelope_ += (level - envelope_) * envCoeff;

        const bool wasOpen = open_;
        if (!open_ && envelope_ >= openThreshold_)      open_ = true;
        else if (open_ && envelope_ < closeThreshold_)  open_ = false;
        if (open_ != wasOpen) isOpen_.store(open_, std::memory_order_relaxed);

        const bool  enabled   = enabled_.load(std::memory_order_relaxed);
        const float target    = (!enabled || open_) ? 1.0f : 0.0f;
        const float gainCoeff = target > gain_ ? attackCoeff_ : releaseCoeff_;
        gain_ += (target - gain_) * gainCoeff;
        return sample * gain_;
    }

    void  SetEnabled(bool on) noexcept { enabled_.store(on, std::memory_order_relaxed); }
    bool  Enabled() const noexcept { return enabled_.load(std::memory_order_relaxed); }
    bool  IsOpen()  const noexcept { return isOpen_.load(std::memory_order_relaxed); }
    float Gain()    const noexcept { return gain_; }     // audio-thread / offline use

private:
    void UpdateCoefficients(float sampleRate) noexcept {
        cachedRate_     = sampleRate;
        attackCoeff_    = OnePoleCoeff(cfg::kGateAttackMs, sampleRate);
        releaseCoeff_   = OnePoleCoeff(cfg::kGateReleaseMs, sampleRate);
        openThreshold_  = DbToLin(cfg::kGateOpenDb);
        closeThreshold_ = DbToLin(cfg::kGateCloseDb);
    }

    std::atomic<bool> enabled_{false};
    std::atomic<bool> isOpen_{false};
    float cachedRate_     = 0.0f;
    float attackCoeff_    = 1.0f;
    float releaseCoeff_   = 1.0f;
    float openThreshold_  = 1.0f;
    float closeThreshold_ = 1.0f;
    float envelope_       = 0.0f;
    float gain_           = 1.0f;
    bool  open_           = false;
};

// -------------------------------------------------------------------------------------
//  CabSimNode: toggleable 2nd-order Butterworth LPF "cab sim" (4.5 kHz). Defaults OFF
//  for full-range voice clarity; LPF state always runs warm for 10 ms click-free [C]
//  crossfades when engaging bass cabinet roll-off.
// -------------------------------------------------------------------------------------
class CabSimNode final : public IDspNode {
public:
    float process(float sample, float sampleRate) noexcept override {
        if (sampleRate != cachedRate_) UpdateCoefficients(sampleRate);

        const double x  = static_cast<double>(sample);
        const double lp = b0_ * x + z1_;
        z1_ = b1_ * x - a1_ * lp + z2_;
        z2_ = b2_ * x - a2_ * lp;

        const float target = cabEnabled_.load(std::memory_order_relaxed) ? 1.0f : 0.0f;
        cabMix_ += (target - cabMix_) * mixCoeff_;
        return static_cast<float>(x + cabMix_ * (lp - x));
    }

    void SetCabEnabled(bool on) noexcept { cabEnabled_.store(on, std::memory_order_relaxed); }
    bool CabEnabled() const noexcept { return cabEnabled_.load(std::memory_order_relaxed); }

private:
    void UpdateCoefficients(float sampleRate) noexcept {
        cachedRate_ = sampleRate;
        const double w0    = 2.0 * cfg::kPi * cfg::kCabLpfHz / sampleRate;
        const double cosw  = std::cos(w0);
        const double alpha = std::sin(w0) / (2.0 * 0.70710678118654752);   // Q = 1/sqrt(2)
        const double a0    = 1.0 + alpha;
        b0_ = (1.0 - cosw) * 0.5 / a0;
        b1_ = (1.0 - cosw) / a0;
        b2_ = b0_;
        a1_ = -2.0 * cosw / a0;
        a2_ = (1.0 - alpha) / a0;

        mixCoeff_ = OnePoleCoeff(cfg::kCabToggleMs, sampleRate);
    }

    std::atomic<bool> cabEnabled_{false};
    float  cachedRate_ = 0.0f;
    double b0_ = 0.0, b1_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0, z1_ = 0.0, z2_ = 0.0;
    float  cabMix_     = 0.0f;
    float  mixCoeff_   = 1.0f;
};

// -------------------------------------------------------------------------------------
//  Soft-knee safety limiter: linear below the knee, tanh saturation above it,
//  asymptotically approaching +/-0.98. C1-continuous at the knee (slope 1).
// -------------------------------------------------------------------------------------
inline float SoftKneeLimit(float x, bool& engaged) noexcept {
    constexpr float kRange = cfg::kLimiterCeiling - cfg::kLimiterKnee;
    const float ax = std::fabs(x);
    if (ax <= cfg::kLimiterKnee) return x;
    engaged = true;
    return std::copysign(cfg::kLimiterKnee + kRange * std::tanh((ax - cfg::kLimiterKnee) / kRange), x);
}

// -------------------------------------------------------------------------------------
//  AudioEngine: WASAPI full duplex, DSP chain, dual-tap history buffers, telemetry
// -------------------------------------------------------------------------------------
enum class InputSelect : int { In1 = 0, In2 = 1, Sum = 2 };

struct LatencyInfo {
    ma_uint32 periodFrames = 0;
    ma_uint32 periods      = 0;
    ma_uint32 sampleRate   = 0;
    float     periodMs     = 0.0f;
    float     roundTripMs  = 0.0f;   // estimate: capture period + playback buffer
};

class AudioEngine {
public:
    using History = LockFreeHistoryBuffer<cfg::kFftSize>;

    AudioEngine() {
        // Chain is fully built (and never resized) before the device starts:
        // [DcBlocker 28 Hz] -> [EnvelopeNoiseGate] -> [CabSim 4.5 kHz LPF]
        auto dcBlocker = std::make_unique<DcBlockerNode>();
        auto gate      = std::make_unique<EnvelopeNoiseGateNode>();
        auto cab       = std::make_unique<CabSimNode>();
        dcBlocker_ = dcBlocker.get();
        gate_      = gate.get();
        cab_       = cab.get();
        dspChain_.reserve(3);
        dspChain_.push_back(std::move(dcBlocker));
        dspChain_.push_back(std::move(gate));
        dspChain_.push_back(std::move(cab));
        SetSampleRate(static_cast<float>(cfg::kSampleRate));
    }
    ~AudioEngine() { Stop(); }
    AudioEngine(const AudioEngine&)            = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    // Opens WASAPI full duplex: shared mode first (preserves mic-array beamforming,
    // boost, and acoustic echo/noise APOs), exclusive mode as fallback.
    bool Start(bool preferExclusive = false) {
        if (running_) return true;
#if defined(_WIN32)
        const ma_backend backends[] = { ma_backend_wasapi };
        if (ma_context_init(backends, 1, nullptr, &context_) != MA_SUCCESS) return false;
#else
        if (ma_context_init(nullptr, 0, nullptr, &context_) != MA_SUCCESS) return false;
#endif
        contextOk_ = true;
        const ma_share_mode first  = preferExclusive ? ma_share_mode_exclusive : ma_share_mode_shared;
        const ma_share_mode second = preferExclusive ? ma_share_mode_shared    : ma_share_mode_exclusive;
        if (TryStart(first)) {
            exclusive_ = (first == ma_share_mode_exclusive);
        } else if (TryStart(second)) {
            exclusive_ = (second == ma_share_mode_exclusive);
        } else {
            ma_context_uninit(&context_);
            contextOk_ = false;
            return false;
        }
        running_ = true;
        return true;
    }

    bool ToggleDriverMode() {
        const bool targetExclusive = !exclusive_;
        Stop();
        return Start(targetExclusive);
    }

    void Stop() noexcept {
        if (running_) {
            ma_device_uninit(&device_);   // joins the audio thread
            running_ = false;
        }
        if (contextOk_) {
            ma_context_uninit(&context_);
            contextOk_ = false;
        }
    }

    // Real-time processing. Called from the miniaudio callback; public for offline tests.
    void ProcessInterleaved(float* output, const float* input, ma_uint32 frameCount) noexcept {
        ScopedDenormalGuard denormalGuard;
        const int   select     = inputSelect_.load(std::memory_order_relaxed);
        const float trim       = trim_.load(std::memory_order_relaxed);
        const float muteTarget = muted_.load(std::memory_order_relaxed) ? 0.0f : 1.0f;
        const float sr         = sampleRate_;

        float inPeak  = 0.0f;
        float outPeak = 0.0f;
        bool  limited = false;

        ma_uint32 done = 0;
        while (done < frameCount) {
            const ma_uint32 n = std::min<ma_uint32>(frameCount - done, kScratchFrames);
            for (ma_uint32 i = 0; i < n; ++i) {
                const ma_uint32 frame = done + i;

                float x = 0.0f;
                if (input != nullptr) {
                    const float l = input[frame * cfg::kChannelsIn];
                    const float r = input[frame * cfg::kChannelsIn + 1];
                    x = (select == 0) ? l : (select == 1) ? r : 0.5f * (l + r);
                }
                x *= trim;
                if (!std::isfinite(x)) x = 0.0f;
                inPeak = std::max(inPeak, std::fabs(x));

                for (const auto& node : dspChain_) x = node->process(x, sr);

                dryScratch_[i] = dcBlocker_->LastOutput();        // Pre-FX tap
                const float y  = SoftKneeLimit(x, limited);
                wetScratch_[i] = y;                               // Post-FX tap (pre-mute)

                muteGain_ += (muteTarget - muteGain_) * muteCoeff_;
                const float o = y * muteGain_;
                outPeak = std::max(outPeak, std::fabs(o));
                if (output != nullptr) {
                    output[frame * cfg::kChannelsOut]     = o;    // dual mono
                    output[frame * cfg::kChannelsOut + 1] = o;
                }
            }
            dryBuffer_.Write(dryScratch_.data(), n);
            wetBuffer_.Write(wetScratch_.data(), n);
            done += n;
        }

        AtomicMax(inputPeak_, inPeak);
        AtomicMax(outputPeak_, outPeak);
        if (limited) limitEvent_.store(true, std::memory_order_relaxed);
    }

    // ---- UI controls (UI thread) ----------------------------------------------------
    void CycleInput() noexcept {
        inputSelect_.store((inputSelect_.load(std::memory_order_relaxed) + 1) % 3,
                           std::memory_order_relaxed);
    }
    InputSelect Input() const noexcept {
        return static_cast<InputSelect>(inputSelect_.load(std::memory_order_relaxed));
    }
    void  ToggleMute() noexcept { muted_.store(!Muted(), std::memory_order_relaxed); }
    bool  Muted() const noexcept { return muted_.load(std::memory_order_relaxed); }
    void  ToggleCab() noexcept { cab_->SetCabEnabled(!cab_->CabEnabled()); }
    bool  CabEnabled() const noexcept { return cab_->CabEnabled(); }
    void  ToggleGate() noexcept { gate_->SetEnabled(!gate_->Enabled()); }
    bool  GateEnabled() const noexcept { return gate_->Enabled(); }
    void  SetTrim(float g) noexcept {
        trim_.store(std::clamp(g, cfg::kTrimMin, cfg::kTrimMax), std::memory_order_relaxed);
    }
    float Trim() const noexcept { return trim_.load(std::memory_order_relaxed); }
    bool  GateOpen() const noexcept { return gate_->IsOpen(); }

    // ---- Telemetry (UI thread consumes and resets) ----------------------------------
    float ConsumeInputPeak() noexcept  { return inputPeak_.exchange(0.0f, std::memory_order_relaxed); }
    float ConsumeOutputPeak() noexcept { return outputPeak_.exchange(0.0f, std::memory_order_relaxed); }
    bool  ConsumeLimitEvent() noexcept { return limitEvent_.exchange(false, std::memory_order_relaxed); }

    const History& DryTap() const noexcept { return dryBuffer_; }
    const History& WetTap() const noexcept { return wetBuffer_; }

    // ---- Device info (valid after Start) --------------------------------------------
    bool        Running() const noexcept   { return running_; }
    bool        Exclusive() const noexcept { return exclusive_; }
    float       SampleRate() const noexcept { return sampleRate_; }
    LatencyInfo Latency() const noexcept   { return latency_; }
    const char* CaptureName() const noexcept  { return captureName_; }
    const char* PlaybackName() const noexcept { return playbackName_; }

private:
    static constexpr ma_uint32 kScratchFrames = 512;

    static void DataCallback(ma_device* device, void* output, const void* input,
                             ma_uint32 frameCount) {
        static_cast<AudioEngine*>(device->pUserData)
            ->ProcessInterleaved(static_cast<float*>(output),
                                 static_cast<const float*>(input), frameCount);
    }

    static void AtomicMax(std::atomic<float>& target, float value) noexcept {
        float current = target.load(std::memory_order_relaxed);
        while (value > current &&
               !target.compare_exchange_weak(current, value, std::memory_order_relaxed)) {
        }
    }

    void SetSampleRate(float sampleRate) noexcept {
        sampleRate_ = sampleRate;
        muteCoeff_  = OnePoleCoeff(cfg::kMuteRampMs, sampleRate);
    }

    bool TryStart(ma_share_mode shareMode) {
        ma_device_config config         = ma_device_config_init(ma_device_type_duplex);
        config.capture.format           = ma_format_f32;
        config.capture.channels         = cfg::kChannelsIn;
        config.capture.shareMode        = shareMode;
        config.playback.format          = ma_format_f32;
        config.playback.channels        = cfg::kChannelsOut;
        config.playback.shareMode       = shareMode;
        config.sampleRate               = cfg::kSampleRate;
        config.periodSizeInFrames       = cfg::kPeriodFrames;
        config.periods                  = cfg::kPeriods;
        config.performanceProfile       = ma_performance_profile_low_latency;
        config.wasapi.usage             = ma_wasapi_usage_pro_audio;
        config.wasapi.noAutoConvertSRC  = (shareMode == ma_share_mode_exclusive) ? MA_TRUE : MA_FALSE;
        config.dataCallback             = DataCallback;
        config.pUserData                = this;

        if (ma_device_init(&context_, &config, &device_) != MA_SUCCESS) return false;
        SetSampleRate(static_cast<float>(device_.sampleRate));   // before the thread runs
        if (ma_device_start(&device_) != MA_SUCCESS) {
            ma_device_uninit(&device_);
            return false;
        }

        ma_device_get_name(&device_, ma_device_type_capture, captureName_,
                           sizeof(captureName_), nullptr);
        ma_device_get_name(&device_, ma_device_type_playback, playbackName_,
                           sizeof(playbackName_), nullptr);

        const auto& cap = device_.capture;
        const auto& pb  = device_.playback;
        latency_.periodFrames = pb.internalPeriodSizeInFrames;
        latency_.periods      = pb.internalPeriods;
        latency_.sampleRate   = pb.internalSampleRate;
        if (pb.internalSampleRate > 0 && cap.internalSampleRate > 0) {
            latency_.periodMs    = 1000.0f * pb.internalPeriodSizeInFrames / pb.internalSampleRate;
            latency_.roundTripMs = latency_.periodMs * pb.internalPeriods +
                                   1000.0f * cap.internalPeriodSizeInFrames / cap.internalSampleRate;
        }
        return true;
    }

    // DSP chain (owned) + non-owning handles for UI control / dry tap.
    std::vector<std::unique_ptr<IDspNode>> dspChain_;
    DcBlockerNode*                         dcBlocker_ = nullptr;
    EnvelopeNoiseGateNode*                 gate_      = nullptr;
    CabSimNode*                            cab_       = nullptr;

    History dryBuffer_;   // Pre-FX tap  -> tuner
    History wetBuffer_;   // Post-FX tap -> scope + spectrum

    // Audio-thread-only state
    std::array<float, kScratchFrames> dryScratch_{};
    std::array<float, kScratchFrames> wetScratch_{};
    float sampleRate_ = static_cast<float>(cfg::kSampleRate);
    float muteCoeff_  = 1.0f;
    float muteGain_   = 0.0f;

    // Cross-thread controls & telemetry
    std::atomic<int>   inputSelect_{static_cast<int>(InputSelect::Sum)};
    std::atomic<float> trim_{1.0f};
    std::atomic<bool>  muted_{true};   // start muted: protects against mic/speaker feedback
    std::atomic<float> inputPeak_{0.0f};
    std::atomic<float> outputPeak_{0.0f};
    std::atomic<bool>  limitEvent_{false};

    // Device
    ma_context  context_{};
    ma_device   device_{};
    bool        contextOk_ = false;
    bool        running_   = false;
    bool        exclusive_ = false;
    LatencyInfo latency_{};
    char        captureName_[MA_MAX_DEVICE_NAME_LENGTH + 1]  = "none";
    char        playbackName_[MA_MAX_DEVICE_NAME_LENGTH + 1] = "none";
};

// -------------------------------------------------------------------------------------
//  Self-contained iterative Radix-2 Cooley-Tukey FFT (in-place, no heap allocs/frame)
// -------------------------------------------------------------------------------------
static void CooleyTukeyFFT(std::vector<std::complex<float>>& a) {
    const size_t n = a.size();

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

// Hann-window the snapshot, FFT it, and compute single-sided magnitudes (0 dBFS = 1.0).
static void ComputeMagnitudes(const std::array<float, cfg::kFftSize>& samples,
                              const std::vector<float>& hann,
                              std::vector<std::complex<float>>& fftBuffer,
                              std::vector<float>& magnitudes) {
    for (size_t i = 0; i < cfg::kFftSize; ++i) {
        fftBuffer[i] = { samples[i] * hann[i], 0.0f };
    }
    CooleyTukeyFFT(fftBuffer);
    constexpr float kNormScale = 4.0f / static_cast<float>(cfg::kFftSize);
    magnitudes[0] = std::abs(fftBuffer[0]) * (2.0f / static_cast<float>(cfg::kFftSize));
    for (size_t k = 1; k <= cfg::kHalfBins; ++k) {
        magnitudes[k] = std::abs(fftBuffer[k]) * kNormScale;
    }
}

// -------------------------------------------------------------------------------------
//  Tuner DSP: Targeted Peak Detection (30 - 1200 Hz), Harmonic Fundamental Lock,
//  Parabolic Interpolation, and Pitch Math
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

    // Scan bins covering [30 Hz, 1200 Hz] (keeping 1-bin margin for peak +/- 1) and
    // require a true local maximum so skirts of out-of-band tones never falsely trigger.
    const size_t minBin = std::max<size_t>(
        1, static_cast<size_t>(std::floor(cfg::kTunerMinHz / binWidthHz)));
    const size_t maxBin = std::min<size_t>(
        magnitudes.size() - 2,
        static_cast<size_t>(std::ceil(cfg::kTunerMaxHz / binWidthHz)));
    if (minBin > maxBin) return out;

    // Raising Hann magnitudes to p = 0.23 makes the main lobe closely parabolic (< 1.3 ct bias).
    constexpr double kHannPower = 0.23;
    auto interpBinHz = [&](size_t k) -> double {
        const double y1 = std::pow(std::max(static_cast<double>(magnitudes[k - 1]), 1e-12), kHannPower);
        const double y2 = std::pow(std::max(static_cast<double>(magnitudes[k]),     1e-12), kHannPower);
        const double y3 = std::pow(std::max(static_cast<double>(magnitudes[k + 1]), 1e-12), kHannPower);
        const double denom = y1 - 2.0 * y2 + y3;
        double delta = 0.0;
        if (std::fabs(denom) > 1e-12) {
            delta = std::clamp(0.5 * (y1 - y3) / denom, -0.5, 0.5);
        }
        return (static_cast<double>(k) + delta) * static_cast<double>(binWidthHz);
    };

    float  bestMag  = 0.0f;
    double bestFreq = 0.0;
    for (size_t k = minBin; k <= maxBin; ++k) {
        const float m = magnitudes[k];
        if (m < cfg::kTunerGateMag || m <= bestMag) continue;
        if (m < magnitudes[k - 1] || m < magnitudes[k + 1]) continue;

        const double candFreq = interpBinHz(k);
        if (candFreq >= cfg::kTunerMinHz && candFreq <= cfg::kTunerMaxHz) {
            bestMag  = m;
            bestFreq = candFreq;
        }
    }

    if (bestMag < cfg::kTunerGateMag) return out;

    // Subharmonic lock: in voice (vowel formants) and neck/bridge bass pickups, 2*f0 or
    // 3*f0 often exceeds f0. If a lower local peak sits at bestFreq / h (h = 2..4) with
    // >= 25% (-12 dB) of bestMag, lock onto the true fundamental f0 instead of jumping octaves.
    const float subMinMag = std::max(cfg::kTunerGateMag, 0.25f * bestMag);
    for (size_t k = minBin; k <= maxBin; ++k) {
        const float m = magnitudes[k];
        if (m < subMinMag) continue;
        if (m < magnitudes[k - 1] || m < magnitudes[k + 1]) continue;

        const double candFreq = interpBinHz(k);
        if (candFreq < cfg::kTunerMinHz) continue;
        if (candFreq >= bestFreq * 0.75) break;

        bool isFundamental = false;
        for (int h = 2; h <= 4; ++h) {
            if (std::fabs(candFreq * h - bestFreq) <= 0.04 * bestFreq) {
                isFundamental = true;
                break;
            }
        }
        if (isFundamental) {
            bestFreq = candFreq;
            break;
        }
    }

    const double interpFreq = bestFreq;

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

// Peak-hold meter ballistics: instant rise, linear dB fall.
static void UpdatePeakMeter(float& displayDb, float peakLin, float dt) {
    const float db     = 20.0f * std::log10(std::max(peakLin, 1e-6f));
    const float fallen = displayDb - cfg::kMeterFallDbPerSec * dt;
    displayDb = std::max(db, std::max(fallen, cfg::kMeterFloorDb));
}

// Draws a status-bar chip and returns its width.
static int DrawChip(int x, int y, const char* text, int fontSize, Color fg, Color bg) {
    constexpr int kPadX = 8;
    const int w = MeasureText(text, fontSize) + 2 * kPadX;
    DrawRectangle(x, y, w, fontSize + 8, bg);
    DrawText(text, x + kPadX, y + 4, fontSize, fg);
    return w;
}

static void DrawStatusBar(const AudioEngine& engine) {
    constexpr int kBarH = 34;
    constexpr int kFont = 16;
    constexpr int kChipY = (kBarH - (kFont + 8)) / 2;
    constexpr int kGap   = 8;
    const Color chipBg   = { 0x1C, 0x1C, 0x1C, 0xFF };

    DrawRectangle(0, 0, cfg::kWidth, kBarH, cfg::kStatusBg);
    DrawLine(0, kBarH, cfg::kWidth, kBarH, cfg::kGrid);

    int x = 14;
    DrawText("SubHz", x, kChipY + 3, 20, cfg::kNeon);
    x += MeasureText("SubHz", 20) + 14;

    if (!engine.Running()) {
        DrawChip(x, kChipY, "NO AUDIO DEVICE (WASAPI)", kFont, RAYWHITE, cfg::kTuneRed);
    } else {
        x += DrawChip(x, kChipY, engine.Exclusive() ? "WASAPI EXCLUSIVE" : "WASAPI SHARED",
                      kFont, cfg::kBackground,
                      engine.Exclusive() ? cfg::kNeon : cfg::kTuneGreen) + kGap;

        const LatencyInfo lat = engine.Latency();
        x += DrawChip(x, kChipY,
                      TextFormat("%ux%u @ %.1fk  %.2f ms  RT~%.1f ms", lat.periodFrames,
                                 lat.periods, lat.sampleRate / 1000.0f, lat.periodMs,
                                 lat.roundTripMs),
                      kFont, LIGHTGRAY, chipBg) + kGap;

        const char* inLabel = engine.Input() == InputSelect::In1 ? "IN 1 [I]"
                            : engine.Input() == InputSelect::In2 ? "IN 2 [I]"
                                                                 : "IN 1+2 [I]";
        x += DrawChip(x, kChipY, inLabel, kFont, cfg::kNeon, chipBg) + kGap;

        x += DrawChip(x, kChipY, engine.GateEnabled() ? "GATE: ON [G]" : "GATE: OFF [G]",
                      kFont, engine.GateEnabled() ? cfg::kTuneGreen : GRAY, chipBg) + kGap;

        x += DrawChip(x, kChipY, engine.CabEnabled() ? "CAB: 4.5kHz LPF [C]" : "CAB: OFF [C]",
                      kFont, engine.CabEnabled() ? cfg::kTuneGreen : GRAY, chipBg) + kGap;

        DrawChip(x, kChipY, engine.Muted() ? "MUTED [SPACE]" : "LIVE [SPACE]", kFont,
                 engine.Muted() ? RAYWHITE : cfg::kBackground,
                 engine.Muted() ? cfg::kTuneRed : cfg::kTuneGreen);
    }

    const char* fps = TextFormat("FPS %d", GetFPS());
    DrawText(fps, cfg::kWidth - MeasureText(fps, kFont) - 14, kChipY + 4, kFont, GRAY);
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

    // Three-zone fill: green < -12 dBFS, amber < -3 dBFS, red above.
    struct Zone { float lo, hi; Color c; };
    const Zone zones[] = { { cfg::kMeterFloorDb, -12.0f, cfg::kTuneGreen },
                           { -12.0f, -3.0f, cfg::kTuneOrange },
                           { -3.0f, 0.0f, cfg::kTuneRed } };
    for (const auto& z : zones) {
        if (db <= z.lo) break;
        const int x0 = dbToX(z.lo);
        const int x1 = dbToX(std::min(db, z.hi));
        if (x1 > x0) DrawRectangle(x0, y + 1, x1 - x0, h - 2, z.c);
    }
    for (float tick : { -48.0f, -36.0f, -24.0f, -12.0f, -6.0f, -3.0f }) {
        const int tx = dbToX(tick);
        DrawLine(tx, y, tx, y + h, Fade(cfg::kBackground, 0.8f));
    }
    DrawRectangleLines(barX, y, barW, h, cfg::kGrid);

    const char* value = db <= cfg::kMeterFloorDb + 0.05f ? "-inf dB" : TextFormat("%.1f dB", db);
    DrawText(value, barX + barW + 8, y, 14, GRAY);
}

static void DrawTelemetryPanel(const AudioEngine& engine, float inDb, float outDb,
                               bool limitLed) {
    constexpr int kX = 24;
    constexpr int kY = 46;
    constexpr int kW = 392;

    DrawText(TextFormat("TRIM x%.2f (%+.1f dB)  [Up/Down, R]", engine.Trim(),
                        20.0f * std::log10(engine.Trim())),
             kX, kY, 16, LIGHTGRAY);
    DrawPeakMeter(kX, kY + 28, kW, 14, "IN", inDb);
    DrawPeakMeter(kX, kY + 50, kW, 14, "OUT", outDb);

    // LIMIT LED
    const int ledY = kY + 78;
    DrawRectangle(kX, ledY, 14, 14, limitLed ? cfg::kTuneRed : Color{ 0x3A, 0x10, 0x10, 0xFF });
    DrawRectangleLines(kX, ledY, 14, 14, cfg::kGrid);
    DrawText("LIMIT", kX + 20, ledY, 14, limitLed ? cfg::kTuneRed : DARKGRAY);

    // Gate LED
    const bool gateEnabled = engine.GateEnabled();
    const bool gateOpen    = engine.GateOpen();
    const Color gateLedCol = !gateEnabled ? Color{ 0x26, 0x26, 0x26, 0xFF }
                           : gateOpen     ? cfg::kTuneGreen
                                          : Color{ 0x10, 0x30, 0x18, 0xFF };
    DrawRectangle(kX + 90, ledY, 14, 14, gateLedCol);
    DrawRectangleLines(kX + 90, ledY, 14, 14, cfg::kGrid);
    const char* gateTxt = !gateEnabled ? "GATE BYPASS" : (gateOpen ? "GATE OPEN" : "GATE CLOSED");
    DrawText(gateTxt, kX + 110, ledY, 14,
             !gateEnabled ? GRAY : (gateOpen ? cfg::kTuneGreen : DARKGRAY));

    DrawText(TextFormat("open %.0f / close %.0f dBFS", cfg::kGateOpenDb, cfg::kGateCloseDb),
             kX + 224, ledY + 1, 12, DARKGRAY);
}

static void DrawSignalFlowPanel(const AudioEngine& engine) {
    constexpr int kX = 864;
    constexpr int kY = 46;
    DrawText("SIGNAL FLOW", kX, kY, 14, GRAY);
    DrawText("IN > TRIM > HPF 28Hz > GATE > CAB > LIMIT > OUT", kX, kY + 20, 12, LIGHTGRAY);
    DrawText("DRY tap (DC-blocked, pre-FX)  -> TUNER", kX, kY + 38, 12, cfg::kNeonLow);
    DrawText("WET tap (post-limiter)        -> SCOPE + FFT", kX, kY + 54, 12, cfg::kNeon);
    DrawText(TextFormat("In:  %.44s", engine.CaptureName()), kX, kY + 76, 12, DARKGRAY);
    DrawText(TextFormat("Out: %.44s", engine.PlaybackName()), kX, kY + 92, 12, DARKGRAY);
}

// -------------------------------------------------------------------------------------
//  Tuner UI Overlay (top center)
// -------------------------------------------------------------------------------------
static void DrawTunerOverlay(const TunerReading& tuner, float smoothedCents,
                             float smoothedHz) {
    constexpr int kBoxW    = 420;
    constexpr int kBoxH    = 118;
    constexpr int kBoxX    = (cfg::kWidth - kBoxW) / 2;
    constexpr int kBoxY    = 44;
    constexpr int kCenterX = cfg::kWidth / 2;

    Color statusCol = DARKGRAY;
    if (tuner.active) {
        const float absCents = std::fabs(smoothedCents);
        if (absCents <= cfg::kInTuneCents) {
            statusCol = cfg::kTuneGreen;
        } else {
            const float severity = std::clamp((absCents - cfg::kInTuneCents) / 20.0f,
                                              0.0f, 1.0f);
            statusCol = BlendColor(cfg::kTuneOrange, cfg::kTuneRed, severity);
        }
    }

    DrawRectangle(kBoxX, kBoxY, kBoxW, kBoxH, cfg::kPanelBg);
    DrawRectangleLines(kBoxX, kBoxY, kBoxW, kBoxH,
                       tuner.active ? Fade(statusCol, 0.85f) : cfg::kGrid);

    const char* noteText = tuner.active
                               ? TextFormat("%s%d", tuner.noteName, tuner.octave)
                               : "--";
    constexpr int kNoteFontSize = 44;
    const int noteW = MeasureText(noteText, kNoteFontSize);
    DrawText(noteText, kCenterX - noteW / 2, kBoxY + 10, kNoteFontSize,
             tuner.active ? statusCol : GRAY);

    if (tuner.active) {
        DrawText(TextFormat("%.1f Hz", smoothedHz), kBoxX + 16, kBoxY + 24, 18, LIGHTGRAY);
        const bool inTune = std::fabs(smoothedCents) <= cfg::kInTuneCents;
        const char* centsTxt = inTune ? TextFormat("%+.1f ct  OK", smoothedCents)
                                      : TextFormat("%+.1f ct", smoothedCents);
        const int centsW = MeasureText(centsTxt, 18);
        DrawText(centsTxt, kBoxX + kBoxW - centsW - 16, kBoxY + 24, 18, statusCol);
    } else {
        DrawText("DRY 30-1200 Hz", kBoxX + 16, kBoxY + 26, 16, DARKGRAY);
        const char* idleTxt = "PLAY A NOTE";
        DrawText(idleTxt, kBoxX + kBoxW - MeasureText(idleTxt, 16) - 16,
                 kBoxY + 26, 16, DARKGRAY);
    }

    constexpr int   kGaugeW    = 350;
    constexpr int   kGaugeX    = kCenterX - kGaugeW / 2;
    constexpr int   kGaugeY    = kBoxY + 76;
    constexpr int   kGaugeH    = 14;
    constexpr float kPxPerCent = kGaugeW / 100.0f;

    DrawRectangle(kGaugeX, kGaugeY, kGaugeW, kGaugeH, Color{ 0x10, 0x10, 0x10, 0xFF });
    const int inTuneHalfW = static_cast<int>(std::round(cfg::kInTuneCents * kPxPerCent));
    DrawRectangle(kCenterX - inTuneHalfW, kGaugeY, inTuneHalfW * 2, kGaugeH,
                  Fade(cfg::kTuneGreen, tuner.active ? 0.25f : 0.10f));
    DrawRectangleLines(kGaugeX, kGaugeY, kGaugeW, kGaugeH, cfg::kGrid);

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
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(cfg::kWidth, cfg::kHeight, "SubHz - Full-Duplex Bass DSP Rig");
    SetTargetFPS(cfg::kTargetFps);

    // ---- Audio engine ---------------------------------------------------------------
    auto engine = std::make_unique<AudioEngine>();
    if (engine->Start()) {
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
    std::array<float, cfg::kNumBars> previous_frame_heights{};

    static std::array<float, cfg::kFftSize>   drySamples{};
    static std::array<float, cfg::kFftSize>   wetSamples{};
    static std::array<Vector2, cfg::kFftSize> wavePoints{};

    TunerReading lastTuner{};
    int   lastMidiNote  = -1;
    float smoothedCents = 0.0f;
    float smoothedHz    = 0.0f;

    float inMeterDb    = cfg::kMeterFloorDb;
    float outMeterDb   = cfg::kMeterFloorDb;
    float limitLedTime = 0.0f;

    const float binWidthHz = engine->SampleRate() / static_cast<float>(cfg::kFftSize);

    // Layout regions
    constexpr float kMarginX  = 24.0f;
    constexpr float kPlotW    = cfg::kWidth - 2.0f * kMarginX;
    constexpr float kWaveTop  = 172.0f;
    constexpr float kWaveH    = 82.0f;
    constexpr float kSpecTop  = 268.0f;
    constexpr float kSpecBot  = 676.0f;
    constexpr float kSpecMaxH = kSpecBot - kSpecTop;

    // ---- Main loop ------------------------------------------------------------------
    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        // 1. Controls
        const float step = std::pow(10.0f, cfg::kTrimStepDb / 20.0f);
        if (IsKeyPressed(KEY_UP)   || IsKeyPressedRepeat(KEY_UP))   engine->SetTrim(engine->Trim() * step);
        if (IsKeyPressed(KEY_DOWN) || IsKeyPressedRepeat(KEY_DOWN)) engine->SetTrim(engine->Trim() / step);
        if (IsKeyPressed(KEY_R))     engine->SetTrim(1.0f);
        if (IsKeyPressed(KEY_G))     engine->ToggleGate();
        if (IsKeyPressed(KEY_C))     engine->ToggleCab();
        if (IsKeyPressed(KEY_I))     engine->CycleInput();
        if (IsKeyPressed(KEY_SPACE)) engine->ToggleMute();
        if (IsKeyPressed(KEY_E))     engine->ToggleDriverMode();

        // 2. Telemetry
        UpdatePeakMeter(inMeterDb,  engine->ConsumeInputPeak(),  dt);
        UpdatePeakMeter(outMeterDb, engine->ConsumeOutputPeak(), dt);
        limitLedTime = engine->ConsumeLimitEvent() ? cfg::kLimitLedHoldSec
                                                   : std::max(0.0f, limitLedTime - dt);

        // 3. DRY tap -> tuner (keep last reading if the producer lapped us)
        TunerReading tuner = lastTuner;
        if (engine->DryTap().Snapshot(drySamples)) {
            ComputeMagnitudes(drySamples, hannWindow, fftBuffer, dryMagnitudes);
            tuner = DetectBassPitch(dryMagnitudes, binWidthHz);
        }
        lastTuner = tuner;
        if (tuner.active) {
            if (tuner.midiNote != lastMidiNote) {
                smoothedCents = tuner.cents;
                smoothedHz    = tuner.frequencyHz;
                lastMidiNote  = tuner.midiNote;
            } else {
                smoothedCents = Lerp(smoothedCents, tuner.cents, cfg::kTunerLerp);
                smoothedHz    = Lerp(smoothedHz, tuner.frequencyHz, cfg::kTunerLerp);
            }
        } else {
            lastMidiNote = -1;
        }

        // 4. WET tap -> scope + spectrum (a torn frame is visual-only and self-heals)
        engine->WetTap().Snapshot(wetSamples);
        ComputeMagnitudes(wetSamples, hannWindow, fftBuffer, wetMagnitudes);

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
                // Sub-bin resolution in the bass: interpolate neighbouring bins.
                const float binCenter = std::clamp(fc / binWidthHz, 1.0f, maxValidBin);
                const size_t k0 = static_cast<size_t>(std::floor(binCenter));
                const size_t k1 = std::min(k0 + 1, cfg::kHalfBins);
                const float frac = binCenter - static_cast<float>(k0);
                bandMag = Lerp(wetMagnitudes[k0], wetMagnitudes[k1], frac);
            } else {
                // Multi-bin bands: take the peak so narrow harmonics stay sharp.
                const size_t kStart = static_cast<size_t>(std::floor(bin0));
                const size_t kEnd   = std::min(static_cast<size_t>(std::ceil(bin1)),
                                               cfg::kHalfBins);
                for (size_t k = kStart; k <= kEnd; ++k) {
                    bandMag = std::max(bandMag, wetMagnitudes[k]);
                }
            }

            const float db   = 20.0f * std::log10(std::max(bandMag, 1e-9f));
            const float norm = std::clamp((db - cfg::kMinDb) / (cfg::kMaxDb - cfg::kMinDb),
                                          0.0f, 1.0f);
            previous_frame_heights[b] =
                Lerp(previous_frame_heights[b], norm * kSpecMaxH, cfg::kSmoothing);
        }

        const float waveCenterY = kWaveTop + kWaveH * 0.5f;
        const float waveAmpPx   = kWaveH * 0.45f;
        const float waveDx      = kPlotW / static_cast<float>(cfg::kFftSize - 1);
        for (size_t i = 0; i < cfg::kFftSize; ++i) {
            const float v = std::clamp(wetSamples[i], -1.1f, 1.1f);
            wavePoints[i] = { kMarginX + i * waveDx, waveCenterY - v * waveAmpPx };
        }

        // ---- Draw -------------------------------------------------------------------
        BeginDrawing();
        ClearBackground(cfg::kBackground);

        DrawStatusBar(*engine);
        DrawTelemetryPanel(*engine, inMeterDb, outMeterDb, limitLedTime > 0.0f);
        DrawSignalFlowPanel(*engine);
        DrawTunerOverlay(tuner, smoothedCents, smoothedHz);

        // Oscilloscope (WET)
        DrawRectangleLines(static_cast<int>(kMarginX), static_cast<int>(kWaveTop),
                           static_cast<int>(kPlotW), static_cast<int>(kWaveH), cfg::kGrid);
        DrawLine(static_cast<int>(kMarginX), static_cast<int>(waveCenterY),
                 static_cast<int>(kMarginX + kPlotW), static_cast<int>(waveCenterY),
                 cfg::kGrid);
        DrawLineStrip(wavePoints.data(), static_cast<int>(cfg::kFftSize),
                      Fade(cfg::kNeon, 0.85f));
        DrawText("WET SCOPE (post-FX, 4096 samples)", static_cast<int>(kMarginX) + 8,
                 static_cast<int>(kWaveTop) + 6, 12, GRAY);

        // Spectrum (WET)
        const float bassZoneW = FreqToNormLogX(2000.0f) * kPlotW;
        DrawRectangle(static_cast<int>(kMarginX), static_cast<int>(kSpecTop),
                      static_cast<int>(bassZoneW), static_cast<int>(kSpecMaxH),
                      Color{ 0x00, 0xFF, 0xC8, 0x07 });
        DrawRectangleLines(static_cast<int>(kMarginX), static_cast<int>(kSpecTop),
                           static_cast<int>(kPlotW), static_cast<int>(kSpecMaxH), cfg::kGrid);

        for (int dbTick : { -15, -30, -45 }) {
            const float norm = (static_cast<float>(dbTick) - cfg::kMinDb) /
                               (cfg::kMaxDb - cfg::kMinDb);
            const int y = static_cast<int>(kSpecBot - norm * kSpecMaxH);
            DrawLine(static_cast<int>(kMarginX), y,
                     static_cast<int>(kMarginX + kPlotW), y, Fade(cfg::kGrid, 0.65f));
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
                     isNote ? Color{ 0x00, 0xFF, 0xC8, 0x22 } : Fade(cfg::kGrid, 0.7f));
            DrawText(tick.label, x - 10, static_cast<int>(kSpecBot) + 8, 14,
                     isNote ? cfg::kNeon : GRAY);
        }
        DrawText("WET SPECTRUM - BASS FOCUS (20 Hz - 2 kHz)", static_cast<int>(kMarginX) + 8,
                 static_cast<int>(kSpecTop) + 6, 12, Fade(cfg::kNeon, 0.65f));
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
            const Color baseCol  = BlendColor(cfg::kNeonLow, cfg::kNeon,
                                              std::min(freqFrac * 1.4f, 1.0f));
            DrawRectangle(x, y, barW, barH, BlendColor(baseCol, cfg::kNeonHot, ampFrac * 0.45f));
        }

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
    engine->Stop();        // stop the audio thread before the engine is destroyed
    engine.reset();
    CloseWindow();
    return 0;
}
