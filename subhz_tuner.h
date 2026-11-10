// =====================================================================================
//  subhz_tuner.h - FFT, Hybrid YIN/Spectral Chromatic Bass Tuner & Scope Trigger
// =====================================================================================
#pragma once

#include "subhz_config.h"
#include <complex>
#include <vector>

// -------------------------------------------------------------------------------------
//  Self-contained iterative Radix-2 Cooley-Tukey FFT (in-place, no heap allocs/frame)
// -------------------------------------------------------------------------------------
inline void CooleyTukeyFFT(std::vector<std::complex<float>>& a) {
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

namespace detail {
struct LastTimeSnapshot {
    std::array<float, cfg::kFftSize> samples{};
    std::array<float, 6>             magSig{};
    bool                             valid = false;
};
inline LastTimeSnapshot& GetLastTimeSnapshot() noexcept {
    thread_local LastTimeSnapshot tls{};
    return tls;
}
}  // namespace detail

// Hann-window the snapshot, FFT it, and compute single-sided magnitudes (0 dBFS = 1.0).
inline void ComputeMagnitudes(const std::array<float, cfg::kFftSize>& samples,
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

    auto& cache   = detail::GetLastTimeSnapshot();
    cache.samples = samples;
    for (size_t i = 0; i < cache.magSig.size() && (i + 2) < magnitudes.size(); ++i) {
        cache.magSig[i] = magnitudes[i + 2];
    }
    cache.valid = true;
}

// -------------------------------------------------------------------------------------
//  Tuner DSP: Hybrid Time-Domain YIN Lag Verification + Spectral Harmonic Interpolation
// -------------------------------------------------------------------------------------
struct TunerReading {
    bool        active      = false;
    float       frequencyHz = 0.0f;
    const char* noteName    = "--";
    int         octave      = 0;
    int         midiNote    = -1;
    float       cents       = 0.0f;    // [-50, +50]
    float       confidence  = 0.0f;    // [0, 1] periodicity confidence
};

// Theoretical normalized Hann window kernel |W(delta)| for bin offset `delta`
inline double HannKernelMag(double delta) noexcept {
    const double ad = std::fabs(delta);
    if (ad < 1e-7) return 1.0;
    if (std::fabs(ad - 1.0) < 1e-7) return 0.5;
    const double num = std::sin(cfg::kPi * ad);
    const double den = cfg::kPi * ad * std::fabs(1.0 - ad * ad);
    return std::fabs(num / den);
}

// Normalized square difference d'(tau) at integer lag tau in [0, 2]; 0 = strictly periodic.
inline double NormalizedLagDiff(const float* x, size_t w, size_t tau) noexcept {
    double diff = 0.0;
    double e0   = 0.0;
    double e1   = 0.0;
    for (size_t i = 0; i < w; ++i) {
        const double a = x[i];
        const double b = x[i + tau];
        const double d = a - b;
        diff += d * d;
        e0   += a * a;
        e1   += b * b;
    }
    const double denom = e0 + e1;
    return denom > 1e-18 ? (diff / denom) : 1.0;
}

inline TunerReading DetectBassPitch(const std::vector<float>& magnitudes,
                                    float binWidthHz,
                                    const std::array<float, cfg::kFftSize>* timeSamples,
                                    float gateDb = cfg::kTunerDefaultGateDb) {
    static constexpr const char* kNoteNames[12] = {
        "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
    };

    TunerReading out{};
    if (magnitudes.size() <= 4 || binWidthHz <= 0.0f) return out;

    // If caller invoked the 2-arg signature after ComputeMagnitudes, recover time-domain samples
    if (timeSamples == nullptr) {
        const auto& cache = detail::GetLastTimeSnapshot();
        if (cache.valid && magnitudes.size() > 8) {
            bool match = true;
            for (size_t i = 0; i < cache.magSig.size(); ++i) {
                if (magnitudes[i + 2] != cache.magSig[i]) {
                    match = false;
                    break;
                }
            }
            if (match) timeSamples = &cache.samples;
        }
    }

    const float gateLinear = DbToLin(gateDb);
    const float sampleRate = binWidthHz * static_cast<float>(cfg::kFftSize);

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
    size_t bestBin  = 0;
    double bestFreq = 0.0;
    double magSum   = 0.0;
    size_t binCount = 0;

    for (size_t k = minBin; k <= maxBin; ++k) {
        const float m = magnitudes[k];
        magSum += m;
        ++binCount;
        if (m < gateLinear || m <= bestMag) continue;
        if (m < magnitudes[k - 1] || m < magnitudes[k + 1]) continue;

        const double candFreq = interpBinHz(k);
        if (candFreq >= cfg::kTunerMinHz && candFreq <= cfg::kTunerMaxHz) {
            bestMag  = m;
            bestBin  = k;
            bestFreq = candFreq;
        }
    }

    if (bestMag < gateLinear) return out;

    // Reject broadband / white noise: require dominant spectral peak to stand well above mean bin level
    const double meanMag = binCount > 0 ? (magSum / static_cast<double>(binCount)) : 0.0;
    if (bestMag < 4.5 * meanMag) return out;

    int    chosenHarmonic = 1;
    double interpFreq     = bestFreq;
    float  confidence     = std::clamp(
        static_cast<float>((bestMag / std::max(meanMag, 1e-9)) / 20.0), 0.0f, 1.0f);

    if (timeSamples != nullptr) {
        // Time-domain YIN / Normalized Square Difference subharmonic lock:
        // Directly tests candidate fundamental periods T_h = h * (fs / bestFreq) for h = 2..4.
        // Immune to FFT bin merging on 5-string low B0 (30.87 Hz, bin 2.63) and tracks weak
        // fundamentals down to -20 dB below the 2nd harmonic without octave jumping.
        const size_t maxLag = static_cast<size_t>(std::ceil(sampleRate / (cfg::kTunerMinHz - 2.0f)));
        const size_t yinW   = (cfg::kFftSize > maxLag + 64) ? (cfg::kFftSize - maxLag - 8)
                                                            : (cfg::kFftSize / 2);

        auto refineLagMin = [&](double approxLag, double& outBestLag) -> double {
            const int center = static_cast<int>(std::lround(approxLag));
            const int lo     = std::max(16, center - 4);
            const int hi     = std::min(static_cast<int>(cfg::kFftSize - yinW - 2), center + 4);
            int    bestTau   = center;
            double minVal    = 2.0;
            for (int tau = lo; tau <= hi; ++tau) {
                const double d = NormalizedLagDiff(timeSamples->data(), yinW, static_cast<size_t>(tau));
                if (d < minVal) {
                    minVal  = d;
                    bestTau = tau;
                }
            }
            const double dPrev = (bestTau > lo)
                ? NormalizedLagDiff(timeSamples->data(), yinW, static_cast<size_t>(bestTau - 1))
                : minVal;
            const double dNext = (bestTau < hi)
                ? NormalizedLagDiff(timeSamples->data(), yinW, static_cast<size_t>(bestTau + 1))
                : minVal;
            const double denom = dPrev - 2.0 * minVal + dNext;
            double delta = 0.0;
            if (std::fabs(denom) > 1e-12) {
                delta = std::clamp(0.5 * (dPrev - dNext) / denom, -0.5, 0.5);
            }
            outBestLag = static_cast<double>(bestTau) + delta;
            return std::max(0.0, minVal - 0.25 * (dPrev - dNext) * delta);
        };

        const double baseLag    = static_cast<double>(sampleRate) / bestFreq;
        double       lag1       = baseLag;
        const double d1         = refineLagMin(baseLag, lag1);
        double       chosenLag  = lag1;
        double       chosenDiff = d1;

        for (int h = 2; h <= 4; ++h) {
            const double candFreq = bestFreq / static_cast<double>(h);
            if (candFreq < cfg::kTunerMinHz) break;

            double lagH = baseLag * static_cast<double>(h);
            const double dH = refineLagMin(lagH, lagH);
            if (dH < 0.10 && (d1 - dH) > 0.008) {
                chosenHarmonic = h;
                chosenLag      = lagH;
                chosenDiff     = dH;
                break;
            }
        }

        if (chosenDiff > 0.30) return out;
        confidence = static_cast<float>(std::clamp(1.0 - chosenDiff, 0.0, 1.0));
        interpFreq = bestFreq / static_cast<double>(chosenHarmonic);
        if (chosenHarmonic > 1 && interpFreq < 45.0) {
            // Below 45 Hz (e.g. B0 = 30.87 Hz), time-domain parabolic lag interpolation at
            // ~1555 samples avoids any main-lobe skirt pull between bins 2.6 and 5.3.
            interpFreq = static_cast<double>(sampleRate) / chosenLag;
        }
    } else {
        // Spectral-only fallback (when magnitudes are synthesized without ComputeMagnitudes):
        // Subtracts the dominant peak's Hann main-lobe skirt and locks subharmonics down to -20 dB.
        const double peakBinFrac = bestFreq / static_cast<double>(binWidthHz);
        const float  subMinMag   = std::max(0.0005f, 0.09f * bestMag);

        for (int h = 4; h >= 2; --h) {
            const double targetSubFreq = bestFreq / static_cast<double>(h);
            if (targetSubFreq < cfg::kTunerMinHz) continue;

            const double targetBinFrac = targetSubFreq / static_cast<double>(binWidthHz);
            const size_t kCenter       = static_cast<size_t>(std::lround(targetBinFrac));
            if (kCenter < 1 || kCenter + 1 >= bestBin) continue;

            auto cleanMag = [&](size_t k) -> float {
                const float skirt = static_cast<float>(
                    bestMag * HannKernelMag(static_cast<double>(k) - peakBinFrac));
                return std::max(0.0f, magnitudes[k] - 0.85f * skirt);
            };

            const float localPeak = std::max(
                cleanMag(kCenter), std::max(cleanMag(kCenter - 1), cleanMag(kCenter + 1)));
            if (localPeak >= subMinMag && localPeak > 3.0f * static_cast<float>(meanMag)) {
                chosenHarmonic = h;
                interpFreq     = bestFreq / static_cast<double>(h);
                break;
            }
        }
    }

    // Low-level 50 Hz ("G1 +35 ct") / 60 Hz ("B1 -49 ct") AC mains hum rejection:
    // A faint 50/60 Hz sinusoid (< -32 dBFS) with negligible 2nd/3rd harmonics is power-line hum.
    const bool nearMains = (std::fabs(interpFreq - 50.0) <= 0.65) ||
                           (std::fabs(interpFreq - 60.0) <= 0.65);
    if (nearMains && bestMag < DbToLin(-32.0f)) {
        const size_t bin2 = static_cast<size_t>(std::lround(2.0 * interpFreq / binWidthHz));
        const size_t bin3 = static_cast<size_t>(std::lround(3.0 * interpFreq / binWidthHz));
        const float  h2   = (bin2 < magnitudes.size()) ? magnitudes[bin2] : 0.0f;
        const float  h3   = (bin3 < magnitudes.size()) ? magnitudes[bin3] : 0.0f;
        if ((h2 + h3) < 0.08f * bestMag) return out;
    }

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
    out.confidence  = confidence;
    return out;
}

// Overload preserving 2-argument call compatibility
inline TunerReading DetectBassPitch(const std::vector<float>& magnitudes,
                                    float binWidthHz,
                                    float gateDb = cfg::kTunerDefaultGateDb) {
    return DetectBassPitch(magnitudes, binWidthHz, nullptr, gateDb);
}

// Overload accepting time-domain samples first
inline TunerReading DetectBassPitch(const std::array<float, cfg::kFftSize>& samples,
                                    const std::vector<float>& magnitudes,
                                    float binWidthHz,
                                    float gateDb = cfg::kTunerDefaultGateDb) {
    return DetectBassPitch(magnitudes, binWidthHz, &samples, gateDb);
}

// -------------------------------------------------------------------------------------
//  TunerTracker: Multi-frame pitch stability gate & adjustable threshold ([ / ])
//  Requires `kTunerLockFrames` consecutive frames with consistent pitch (within 35 cents)
//  and high periodicity confidence before engaging the display, eliminating transient
//  noise spikes and hum flicker.
// -------------------------------------------------------------------------------------
class TunerTracker {
public:
    TunerReading Update(const std::array<float, cfg::kFftSize>& samples,
                        const std::vector<float>& magnitudes,
                        float binWidthHz) noexcept {
        const TunerReading raw = DetectBassPitch(samples, magnitudes, binWidthHz, gateDb_);
        if (!raw.active || raw.confidence < 0.70f) {
            stableCount_ = 0;
            lastFreqHz_  = 0.0f;
            return TunerReading{};
        }

        if (lastFreqHz_ > 0.0f) {
            const float diffCents = std::fabs(1200.0f * std::log2(raw.frequencyHz / lastFreqHz_));
            if (diffCents <= 35.0f) {
                ++stableCount_;
            } else {
                stableCount_ = 1;
            }
        } else {
            stableCount_ = 1;
        }
        lastFreqHz_ = raw.frequencyHz;

        if (stableCount_ >= cfg::kTunerLockFrames) {
            return raw;
        }
        return TunerReading{};
    }

    void  SetGateDb(float db) noexcept {
        gateDb_ = std::clamp(db, cfg::kTunerMinGateDb, cfg::kTunerMaxGateDb);
    }
    float GateDb() const noexcept { return gateDb_; }

    void  Reset() noexcept {
        stableCount_ = 0;
        lastFreqHz_  = 0.0f;
    }

private:
    float gateDb_      = cfg::kTunerDefaultGateDb;
    int   stableCount_ = 0;
    float lastFreqHz_  = 0.0f;
};

// -------------------------------------------------------------------------------------
//  Triggered Oscilloscope: Schmitt-triggered rising-edge zero-crossing finder
//  Scans the first half of the buffer for an armed negative-to-positive zero crossing
//  so periodic waveforms lock stationary on screen instead of crawling.
// -------------------------------------------------------------------------------------
inline size_t FindScopeTriggerIndex(const std::array<float, cfg::kFftSize>& samples,
                                    size_t viewSamples = cfg::kScopeSamples,
                                    float* outSubSampleFrac = nullptr) noexcept {
    if (outSubSampleFrac) *outSubSampleFrac = 0.0f;
    if (viewSamples >= cfg::kFftSize) return 0;

    const size_t maxSearch = cfg::kFftSize - viewSamples;
    float peak = 0.0f;
    for (size_t i = 0; i < maxSearch; ++i) {
        peak = std::max(peak, std::fabs(samples[i]));
    }
    if (peak < 1e-4f) return 0;

    const float hyst = std::max(0.0015f, 0.10f * peak);
    bool armed = false;

    // Start search near the middle of the history window so the freshest triggered wave is shown
    for (size_t i = 4; i < maxSearch; ++i) {
        const float s = samples[i];
        if (!armed) {
            if (s <= -hyst) armed = true;
        } else if (s >= 0.0f && samples[i - 1] < 0.0f) {
            if (outSubSampleFrac) {
                const float denom = s - samples[i - 1];
                *outSubSampleFrac = (denom > 1e-9f) ? (-samples[i - 1] / denom) : 0.0f;
            }
            return i;
        }
    }
    return 0;
}
