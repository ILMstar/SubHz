// =====================================================================================
//  subhz_config.h - Constants, math helpers, lock-free SPSC history buffer & config I/O
// =====================================================================================
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>

#if defined(__SSE__) || defined(_M_X64) || defined(_M_IX86)
    #include <xmmintrin.h>
    #define SUBHZ_HAS_SSE 1
#endif

namespace cfg {
// Window
constexpr int    kWidth              = 1280;
constexpr int    kHeight             = 720;
constexpr int    kMinWidth           = 960;
constexpr int    kMinHeight          = 540;
constexpr int    kTargetFps          = 180;

// Audio device (WASAPI full duplex)
constexpr unsigned int kSampleRate   = 48000;
constexpr unsigned int kChannelsIn   = 2;
constexpr unsigned int kChannelsOut  = 2;
constexpr unsigned int kPeriodFrames = 128;
constexpr unsigned int kPeriods      = 1;

// Analysis
constexpr size_t kFftSize            = 4096;            // Power of 2 (11.72 Hz @ 48 kHz)
constexpr size_t kHalfBins           = kFftSize / 2;    // Up to Nyquist
constexpr size_t kScopeSamples       = 2048;            // Triggered scope window (42.7 ms)
constexpr size_t kLowBandDecim       = 4;               // 4x decimated low-band FFT (2.93 Hz bins)
constexpr float  kLowBandCrossHz     = 900.0f;          // Use 2.93 Hz low-band FFT below 900 Hz

// Input trim
constexpr float  kTrimMin            = 0.1f;            // -20 dB
constexpr float  kTrimMax            = 100.0f;          // +40 dB
constexpr float  kTrimStepDb         = 1.0f;

// DSP chain
constexpr float  kDcBlockHz          = 10.0f;           // 10 Hz HPF (-0.43 dB @ B0 30.87 Hz)
constexpr float  kOdDcBlockHz        = 15.0f;           // Post-overdrive DC blocker on highBand
constexpr float  kCabLpfHz           = 4500.0f;
constexpr float  kCabToggleMs        = 10.0f;           // Click-free cab/bypass crossfade
constexpr float  kParamSmoothMs      = 15.0f;           // Click-free preset/knob parameter smoothing
constexpr float  kGateOpenDb         = -54.0f;          // Hysteresis: open above ...
constexpr float  kGateCloseDb        = -62.0f;          // ... close below (8 dB window)
constexpr float  kGateAttackMs       = 1.0f;
constexpr float  kGateReleaseMs      = 80.0f;
constexpr float  kCompAttackMs       = 10.0f;           // Optical-style feed-forward attack
constexpr float  kCompReleaseMs      = 120.0f;          // Nominal 37% optical release (120 ms)
constexpr float  kCompFilterRelMs    = 114.0f;          // + 6.67 ms peak hold = 120.0 ms to 37%
constexpr float  kCompRatio          = 4.0f;            // 4:1 bass compression
constexpr float  kCompDefaultThreshDb = -18.0f;
constexpr float  kCompDefaultMakeupDb = 1.5f;
constexpr float  kOverdriveCrossHz   = 150.0f;          // Split-band crossover (<150 Hz clean)
constexpr int    kOdOversampleDefault = 4;              // 4x oversampling (192 kHz @ 48 kHz)
constexpr int    kNumDspSlots        = 5;               // Reorderable rack: GATE, COMP, OD, EQ, CAB
constexpr float  kDriveMin           = 1.0f;
constexpr float  kDriveMax           = 25.0f;
constexpr float  kDriveStep          = 0.5f;
constexpr float  kBlendMin           = 0.0f;
constexpr float  kBlendMax           = 1.0f;
constexpr float  kBlendStep          = 0.05f;
constexpr float  kEqMinDb            = -15.0f;
constexpr float  kEqMaxDb            = 15.0f;
constexpr float  kEqStepDb           = 0.5f;
constexpr float  kEqBassHz           = 80.0f;           // Low-shelf
constexpr float  kEqBassQ            = 0.70710678f;
constexpr float  kEqMidHz            = 650.0f;          // Peaking
constexpr float  kEqMidQ             = 1.0f;
constexpr float  kEqTrebleHz         = 3200.0f;         // High-shelf
constexpr float  kEqTrebleQ          = 0.70710678f;
constexpr float  kLimiterKnee        = 0.70f;           // Linear below (-3.1 dBFS)
constexpr float  kLimiterCeiling     = 0.98f;           // Asymptotic tanh ceiling
constexpr float  kMuteRampMs         = 5.0f;

// Spectrum analyzer (20 Hz - 10 kHz log scale devotes ~74% width to 20 - 2000 Hz)
constexpr int    kNumBars            = 160;
constexpr float  kMinFreq            = 20.0f;
constexpr float  kMaxFreq            = 10000.0f;        // Ignore bins above 10 kHz
constexpr float  kMinDb              = -65.0f;
constexpr float  kMaxDb              = 0.0f;
constexpr float  kSmoothing          = 0.15f;           // Legacy nominal factor at 120 FPS
constexpr float  kSpectrumSmoothRate = 18.0f;           // Frame-rate independent rate (1/s)

// Chromatic tuner (30 - 1200 Hz covers low B0 bass through full vocal range)
constexpr float  kTunerMinHz         = 30.0f;           // Covers low B0 (30.87 Hz)
constexpr float  kTunerMaxHz         = 1200.0f;         // Covers bass + voice up to D6
constexpr float  kTunerDefaultGateDb = -44.0f;          // Rejects -46 dBFS mains hum by default
constexpr float  kTunerMinGateDb     = -60.0f;
constexpr float  kTunerMaxGateDb     = -20.0f;
constexpr float  kTunerGateStepDb    = 2.0f;
constexpr float  kTunerGateMag       = 0.00630957f;     // -44.0 dBFS linear amplitude
constexpr float  kInTuneCents        = 5.0f;
constexpr float  kTunerLerp          = 0.20f;
constexpr float  kTunerSmoothRate    = 22.0f;           // Frame-rate independent needle rate (1/s)
constexpr int    kTunerLockFrames    = 3;               // Consecutive stable frames to lock note

// Meters
constexpr float  kMeterFloorDb       = -60.0f;
constexpr float  kMeterFallDbPerSec  = 24.0f;
constexpr float  kLimitLedHoldSec    = 0.25f;

constexpr double kPi                 = 3.14159265358979323846;
constexpr const char* kConfigFile    = "subhz.cfg";
constexpr const char* kPresetJsonFile = "subhz_preset.json";
}  // namespace cfg

// -------------------------------------------------------------------------------------
//  Reorderable DSP rack slot identifiers & validation
// -------------------------------------------------------------------------------------
enum DspSlot : int {
    kSlotGate = 0,
    kSlotComp = 1,
    kSlotOd   = 2,
    kSlotEq   = 3,
    kSlotCab  = 4
};

inline constexpr std::array<int, cfg::kNumDspSlots> kDefaultChainOrder = {
    kSlotGate, kSlotComp, kSlotOd, kSlotEq, kSlotCab
};

inline const char* DspSlotName(int slot) noexcept {
    static constexpr const char* kNames[cfg::kNumDspSlots] = { "GATE", "COMP", "OD", "EQ", "CAB" };
    return (slot >= 0 && slot < cfg::kNumDspSlots) ? kNames[slot] : "?";
}

inline int DspSlotFromName(const char* s) noexcept {
    if (!s) return -1;
    for (int i = 0; i < cfg::kNumDspSlots; ++i) {
        if (std::strcmp(s, DspSlotName(i)) == 0) return i;
    }
    if (s[0] >= '0' && s[0] < ('0' + cfg::kNumDspSlots) && s[1] == '\0') return s[0] - '0';
    return -1;
}

inline bool IsValidChainOrder(const std::array<int, cfg::kNumDspSlots>& order) noexcept {
    unsigned int mask = 0;
    for (int idx : order) {
        if (idx < 0 || idx >= cfg::kNumDspSlots) return false;
        mask |= (1u << idx);
    }
    return mask == ((1u << cfg::kNumDspSlots) - 1u);
}

inline int ClampOversample(int factor) noexcept {
    return (factor >= 4) ? 4 : (factor >= 2) ? 2 : 1;
}

// -------------------------------------------------------------------------------------
//  Small math helpers (C++17: no std::lerp)
// -------------------------------------------------------------------------------------
template <typename T>
constexpr T Lerp(T a, T b, T t) noexcept { return a + t * (b - a); }

inline float DbToLin(float db) noexcept { return std::pow(10.0f, db / 20.0f); }

// Frame-rate independent exponential smoothing: reaches identical state after elapsed
// wall-clock time `T` regardless of whether rendering runs at 60, 144, or 180 FPS.
inline float TimeConstantLerp(float current, float target, float ratePerSec, float dt) noexcept {
    if (dt <= 0.0f) return current;
    const float alpha = 1.0f - std::exp(-ratePerSec * std::min(dt, 0.25f));
    return Lerp(current, target, alpha);
}

// One-pole smoothing coefficient reaching ~63.2% of a step in `ms` milliseconds.
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
        if (count > N) {
            src  += count - N;
            count = N;
        }
        const size_t w = writeIndex_.load(std::memory_order_relaxed);
        claimIndex_.store(w + count, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        for (size_t i = 0; i < count; ++i) {
            slots_[(w + i) & kMask].store(src[i], std::memory_order_relaxed);
        }
        writeIndex_.store(w + count, std::memory_order_release);
    }

    // Consumer only (UI thread). Never blocks the producer.
    bool Snapshot(std::array<float, N>& out) const noexcept {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const size_t w     = writeIndex_.load(std::memory_order_acquire);
            const size_t start = w - N;
            for (size_t i = 0; i < N; ++i) {
                out[i] = slots_[(start + i) & kMask].load(std::memory_order_relaxed);
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            const size_t claimed = claimIndex_.load(std::memory_order_relaxed);
            if (claimed - start <= kCapacity) return true;
        }
        return false;
    }

    size_t TotalWritten() const noexcept { return writeIndex_.load(std::memory_order_acquire); }

private:
    alignas(64) std::atomic<size_t>                       writeIndex_{0};
    alignas(64) std::atomic<size_t>                       claimIndex_{0};
    alignas(64) std::array<std::atomic<float>, kCapacity> slots_;
};

// -------------------------------------------------------------------------------------
//  AppSettings: lightweight key=value persistence (subhz.cfg)
// -------------------------------------------------------------------------------------
struct AppSettings {
    float trim          = 1.0f;
    int   inputSelect   = 2;     // InputSelect::Sum
    int   presetIndex   = 0;
    bool  gateEnabled   = false;
    bool  cabEnabled    = false;
    bool  odBypass      = true;
    float odDrive       = 2.0f;
    float odBlend       = 0.0f;
    int   odOversample  = cfg::kOdOversampleDefault;
    float bassDb        = 0.0f;
    float midDb         = 0.0f;
    float trebleDb      = 0.0f;
    float tunerGateDb   = cfg::kTunerDefaultGateDb;
    bool  vsync         = true;
    bool  exclusive     = false;
    std::array<int, cfg::kNumDspSlots> chainOrder = kDefaultChainOrder;
};

inline bool SaveSettings(const char* path, const AppSettings& s) noexcept {
    if (path == nullptr) return false;
    std::FILE* fp = std::fopen(path, "w");
    if (!fp) return false;
    std::fprintf(fp, "trim=%.6f\n",        s.trim);
    std::fprintf(fp, "inputSelect=%d\n",   s.inputSelect);
    std::fprintf(fp, "presetIndex=%d\n",   s.presetIndex);
    std::fprintf(fp, "gateEnabled=%d\n",   s.gateEnabled ? 1 : 0);
    std::fprintf(fp, "cabEnabled=%d\n",    s.cabEnabled  ? 1 : 0);
    std::fprintf(fp, "odBypass=%d\n",      s.odBypass    ? 1 : 0);
    std::fprintf(fp, "odDrive=%.4f\n",     s.odDrive);
    std::fprintf(fp, "odBlend=%.4f\n",     s.odBlend);
    std::fprintf(fp, "odOversample=%d\n",  ClampOversample(s.odOversample));
    std::fprintf(fp, "bassDb=%.4f\n",      s.bassDb);
    std::fprintf(fp, "midDb=%.4f\n",       s.midDb);
    std::fprintf(fp, "trebleDb=%.4f\n",    s.trebleDb);
    std::fprintf(fp, "tunerGateDb=%.4f\n", s.tunerGateDb);
    std::fprintf(fp, "vsync=%d\n",         s.vsync       ? 1 : 0);
    std::fprintf(fp, "exclusive=%d\n",     s.exclusive   ? 1 : 0);
    std::fprintf(fp, "chainOrder=%d,%d,%d,%d,%d\n",
                 s.chainOrder[0], s.chainOrder[1], s.chainOrder[2], s.chainOrder[3], s.chainOrder[4]);
    std::fclose(fp);
    return true;
}

inline bool LoadSettings(const char* path, AppSettings& s) noexcept {
    if (path == nullptr) return false;
    std::FILE* fp = std::fopen(path, "r");
    if (!fp) return false;
    char line[128]{};
    while (std::fgets(line, sizeof(line), fp)) {
        int c0 = 0, c1 = 0, c2 = 0, c3 = 0, c4 = 0;
        if (std::sscanf(line, "chainOrder=%d,%d,%d,%d,%d", &c0, &c1, &c2, &c3, &c4) == 5) {
            const std::array<int, cfg::kNumDspSlots> cand = { c0, c1, c2, c3, c4 };
            if (IsValidChainOrder(cand)) s.chainOrder = cand;
            continue;
        }
        char  key[64]{};
        float val = 0.0f;
        if (std::sscanf(line, "%63[^=]=%f", key, &val) == 2) {
            if (std::strcmp(key, "trim") == 0)              s.trim         = std::clamp(val, cfg::kTrimMin, cfg::kTrimMax);
            else if (std::strcmp(key, "inputSelect") == 0)  s.inputSelect  = std::clamp(static_cast<int>(val), 0, 2);
            else if (std::strcmp(key, "presetIndex") == 0)  s.presetIndex  = std::clamp(static_cast<int>(val), 0, 3);
            else if (std::strcmp(key, "gateEnabled") == 0)  s.gateEnabled  = (static_cast<int>(val) != 0);
            else if (std::strcmp(key, "cabEnabled") == 0)   s.cabEnabled   = (static_cast<int>(val) != 0);
            else if (std::strcmp(key, "odBypass") == 0)     s.odBypass     = (static_cast<int>(val) != 0);
            else if (std::strcmp(key, "odDrive") == 0)      s.odDrive      = std::clamp(val, cfg::kDriveMin, cfg::kDriveMax);
            else if (std::strcmp(key, "odBlend") == 0)      s.odBlend      = std::clamp(val, cfg::kBlendMin, cfg::kBlendMax);
            else if (std::strcmp(key, "odOversample") == 0) s.odOversample = ClampOversample(static_cast<int>(val));
            else if (std::strcmp(key, "bassDb") == 0)       s.bassDb       = std::clamp(val, cfg::kEqMinDb, cfg::kEqMaxDb);
            else if (std::strcmp(key, "midDb") == 0)        s.midDb        = std::clamp(val, cfg::kEqMinDb, cfg::kEqMaxDb);
            else if (std::strcmp(key, "trebleDb") == 0)     s.trebleDb     = std::clamp(val, cfg::kEqMinDb, cfg::kEqMaxDb);
            else if (std::strcmp(key, "tunerGateDb") == 0)  s.tunerGateDb  = std::clamp(val, cfg::kTunerMinGateDb, cfg::kTunerMaxGateDb);
            else if (std::strcmp(key, "vsync") == 0)        s.vsync        = (static_cast<int>(val) != 0);
            else if (std::strcmp(key, "exclusive") == 0)    s.exclusive    = (static_cast<int>(val) != 0);
        }
    }
    std::fclose(fp);
    return true;
}

// -------------------------------------------------------------------------------------
//  RigPreset: JSON serialization for user presets & custom DSP chain routing
// -------------------------------------------------------------------------------------
struct RigPreset {
    char                               name[64]     = "Custom Rig";
    float                              trim         = 1.0f;
    bool                               gateEnabled  = false;
    float                              compThreshDb = cfg::kCompDefaultThreshDb;
    float                              compMakeupDb = cfg::kCompDefaultMakeupDb;
    bool                               odBypass     = true;
    float                              odDrive      = 2.0f;
    float                              odBlend      = 0.0f;
    int                                odOversample = cfg::kOdOversampleDefault;
    float                              bassDb       = 0.0f;
    float                              midDb        = 0.0f;
    float                              trebleDb     = 0.0f;
    bool                               cabEnabled   = false;
    std::array<int, cfg::kNumDspSlots> chainOrder   = kDefaultChainOrder;
};

inline bool SavePresetJson(const char* path, const RigPreset& p) noexcept {
    if (path == nullptr) return false;
    std::FILE* fp = std::fopen(path, "w");
    if (!fp) return false;

    char safeName[128]{};
    size_t j = 0;
    for (size_t i = 0; p.name[i] != '\0' && j + 2 < sizeof(safeName); ++i) {
        const char c = p.name[i];
        if (c == '"' || c == '\\') safeName[j++] = '\\';
        if (static_cast<unsigned char>(c) >= 32) safeName[j++] = c;
    }

    const auto& ord = IsValidChainOrder(p.chainOrder) ? p.chainOrder : kDefaultChainOrder;
    std::fprintf(fp, "{\n");
    std::fprintf(fp, "  \"name\": \"%s\",\n", safeName);
    std::fprintf(fp, "  \"trim\": %.4f,\n", std::clamp(p.trim, cfg::kTrimMin, cfg::kTrimMax));
    std::fprintf(fp, "  \"gateEnabled\": %s,\n", p.gateEnabled ? "true" : "false");
    std::fprintf(fp, "  \"compThreshDb\": %.4f,\n", std::clamp(p.compThreshDb, -48.0f, 0.0f));
    std::fprintf(fp, "  \"compMakeupDb\": %.4f,\n", std::clamp(p.compMakeupDb, -12.0f, 24.0f));
    std::fprintf(fp, "  \"odBypass\": %s,\n", p.odBypass ? "true" : "false");
    std::fprintf(fp, "  \"odDrive\": %.4f,\n", std::clamp(p.odDrive, cfg::kDriveMin, cfg::kDriveMax));
    std::fprintf(fp, "  \"odBlend\": %.4f,\n", std::clamp(p.odBlend, cfg::kBlendMin, cfg::kBlendMax));
    std::fprintf(fp, "  \"odOversample\": %d,\n", ClampOversample(p.odOversample));
    std::fprintf(fp, "  \"bassDb\": %.4f,\n", std::clamp(p.bassDb, cfg::kEqMinDb, cfg::kEqMaxDb));
    std::fprintf(fp, "  \"midDb\": %.4f,\n", std::clamp(p.midDb, cfg::kEqMinDb, cfg::kEqMaxDb));
    std::fprintf(fp, "  \"trebleDb\": %.4f,\n", std::clamp(p.trebleDb, cfg::kEqMinDb, cfg::kEqMaxDb));
    std::fprintf(fp, "  \"cabEnabled\": %s,\n", p.cabEnabled ? "true" : "false");
    std::fprintf(fp, "  \"chainOrder\": [\"%s\", \"%s\", \"%s\", \"%s\", \"%s\"]\n",
                 DspSlotName(ord[0]), DspSlotName(ord[1]), DspSlotName(ord[2]),
                 DspSlotName(ord[3]), DspSlotName(ord[4]));
    std::fprintf(fp, "}\n");
    std::fclose(fp);
    return true;
}

inline bool LoadPresetJson(const char* path, RigPreset& p) noexcept {
    if (path == nullptr) return false;
    std::FILE* fp = std::fopen(path, "rb");
    if (!fp) return false;
    char buf[4096]{};
    const size_t nRead = std::fread(buf, 1, sizeof(buf) - 1, fp);
    std::fclose(fp);
    if (nRead == 0) return false;
    buf[nRead] = '\0';

    auto findKey = [&](const char* key) -> const char* {
        char pat[64]{};
        std::snprintf(pat, sizeof(pat), "\"%s\"", key);
        const char* pos = std::strstr(buf, pat);
        if (!pos) return nullptr;
        pos = std::strchr(pos + std::strlen(pat), ':');
        if (!pos) return nullptr;
        ++pos;
        while (*pos == ' ' || *pos == '\t' || *pos == '\r' || *pos == '\n') ++pos;
        return pos;
    };

    auto readFloat = [&](const char* key, float& out, float lo, float hi) {
        if (const char* v = findKey(key)) {
            char* end = nullptr;
            const float f = std::strtof(v, &end);
            if (end != v && std::isfinite(f)) out = std::clamp(f, lo, hi);
        }
    };

    auto readBool = [&](const char* key, bool& out) {
        if (const char* v = findKey(key)) {
            if (std::strncmp(v, "true", 4) == 0 || *v == '1') out = true;
            else if (std::strncmp(v, "false", 5) == 0 || *v == '0') out = false;
        }
    };

    if (const char* v = findKey("name")) {
        if (*v == '"') {
            ++v;
            size_t i = 0;
            while (*v != '\0' && *v != '"' && i + 1 < sizeof(p.name)) {
                if (*v == '\\' && *(v + 1) != '\0') ++v;
                p.name[i++] = *v++;
            }
            p.name[i] = '\0';
        }
    }

    readFloat("trim",         p.trim,         cfg::kTrimMin,  cfg::kTrimMax);
    readBool ("gateEnabled",  p.gateEnabled);
    readFloat("compThreshDb", p.compThreshDb, -48.0f,         0.0f);
    readFloat("compMakeupDb", p.compMakeupDb, -12.0f,         24.0f);
    readBool ("odBypass",     p.odBypass);
    readFloat("odDrive",      p.odDrive,      cfg::kDriveMin, cfg::kDriveMax);
    readFloat("odBlend",      p.odBlend,      cfg::kBlendMin, cfg::kBlendMax);
    float osVal = static_cast<float>(p.odOversample);
    readFloat("odOversample", osVal,          1.0f,           4.0f);
    p.odOversample = ClampOversample(static_cast<int>(std::lround(osVal)));
    readFloat("bassDb",       p.bassDb,       cfg::kEqMinDb,  cfg::kEqMaxDb);
    readFloat("midDb",        p.midDb,        cfg::kEqMinDb,  cfg::kEqMaxDb);
    readFloat("trebleDb",     p.trebleDb,     cfg::kEqMinDb,  cfg::kEqMaxDb);
    readBool ("cabEnabled",   p.cabEnabled);

    if (const char* v = findKey("chainOrder")) {
        if (*v == '[') {
            ++v;
            std::array<int, cfg::kNumDspSlots> cand{};
            int count = 0;
            while (*v != '\0' && *v != ']' && count < cfg::kNumDspSlots) {
                while (*v == ' ' || *v == '\t' || *v == '\r' || *v == '\n' || *v == ',') ++v;
                if (*v == '"') {
                    ++v;
                    char tok[16]{};
                    size_t ti = 0;
                    while (*v != '\0' && *v != '"' && ti + 1 < sizeof(tok)) tok[ti++] = *v++;
                    if (*v == '"') ++v;
                    cand[count++] = DspSlotFromName(tok);
                } else if (*v >= '0' && *v <= '9') {
                    cand[count++] = *v - '0';
                    ++v;
                } else {
                    break;
                }
            }
            if (count == cfg::kNumDspSlots && IsValidChainOrder(cand)) {
                p.chainOrder = cand;
            }
        }
    }
    return true;
}
