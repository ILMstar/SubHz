// =====================================================================================
//  subhz_dsp.h - Modular OOP DSP chain nodes, click-free parameter smoothing & presets
// =====================================================================================
#pragma once

#include "subhz_config.h"

// -------------------------------------------------------------------------------------
//  DSP node interface
// -------------------------------------------------------------------------------------
class IDspNode {
public:
    virtual ~IDspNode() = default;
    virtual float process(float sample, float sampleRate) noexcept = 0;
};

// -------------------------------------------------------------------------------------
//  DcBlockerNode: 1st-order DC-blocking HPF (10 Hz). Removes ADC DC offset and sub-5 Hz
//  rumble with < 0.45 dB loss at low B0 (30.87 Hz) and < 0.25 dB at E1 (41.20 Hz).
// -------------------------------------------------------------------------------------
class DcBlockerNode final : public IDspNode {
public:
    float process(float sample, float sampleRate) noexcept override {
        if (sampleRate != cachedRate_) {
            cachedRate_ = sampleRate;
            hpR_ = std::exp(-2.0 * cfg::kPi * cfg::kDcBlockHz / sampleRate);
        }
        const double x  = std::isfinite(sample) ? static_cast<double>(sample) : 0.0;
        const double hp = x - hpX1_ + hpR_ * hpY1_;
        hpX1_    = x;
        hpY1_    = std::isfinite(hp) ? hp : 0.0;
        lastOut_ = static_cast<float>(hpY1_);
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
//  hysteretic open/close decision (open > -54 dBFS, close < -62 dBFS).
// -------------------------------------------------------------------------------------
class EnvelopeNoiseGateNode final : public IDspNode {
public:
    float process(float sample, float sampleRate) noexcept override {
        if (sampleRate != cachedRate_) UpdateCoefficients(sampleRate);

        const float level    = std::isfinite(sample) ? std::fabs(sample) : 0.0f;
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
    float Gain()    const noexcept { return gain_; }

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
//  CompressorNode: Optical-style feed-forward bass compressor (10 ms attack, 120 ms
//  release to 37%, 4:1 ratio).
//  - Uses a 6.67 ms (320-sample) bucketed peak bridge into a SINGLE dB-domain ballistic
//    stage so release is genuinely 120 ms (avoiding 245 ms double-smoother lag).
//  - Smooths thresholdDb and makeupGainDb transitions over 15 ms so preset switches
//    never click, then caches linear gains with zero trig/pow calls at steady state.
// -------------------------------------------------------------------------------------
class CompressorNode final : public IDspNode {
public:
    float process(float sample, float sampleRate) noexcept override {
        const float targetThreshDb = thresholdDb_.load(std::memory_order_relaxed);
        const float targetMakeupDb = makeupGainDb_.load(std::memory_order_relaxed);

        if (sampleRate != cachedRate_) {
            cachedRate_     = sampleRate;
            attackCoeff_    = OnePoleCoeff(cfg::kCompAttackMs, sampleRate);
            releaseCoeff_   = OnePoleCoeff(cfg::kCompFilterRelMs, sampleRate);
            paramCoeff_     = OnePoleCoeff(cfg::kParamSmoothMs, sampleRate);
            smoothThreshDb_ = targetThreshDb;
            smoothMakeupDb_ = targetMakeupDb;
            thresholdLin_   = DbToLin(smoothThreshDb_);
            makeupLin_      = DbToLin(smoothMakeupDb_);
        }

        if (smoothThreshDb_ != targetThreshDb || smoothMakeupDb_ != targetMakeupDb) {
            smoothThreshDb_ += (targetThreshDb - smoothThreshDb_) * paramCoeff_;
            smoothMakeupDb_ += (targetMakeupDb - smoothMakeupDb_) * paramCoeff_;
            if (std::fabs(targetThreshDb - smoothThreshDb_) <= 1e-4f) smoothThreshDb_ = targetThreshDb;
            if (std::fabs(targetMakeupDb - smoothMakeupDb_) <= 1e-4f) smoothMakeupDb_ = targetMakeupDb;
            thresholdLin_ = DbToLin(smoothThreshDb_);
            makeupLin_    = DbToLin(smoothMakeupDb_);
        }

        // 6.67 ms sliding peak bridge (16 buckets x 20 samples = 320 samples @ 48 kHz)
        const float absX = std::isfinite(sample) ? std::fabs(sample) : 0.0f;
        bucketMax_ = std::max(bucketMax_, absX);
        if (++bucketSamples_ >= kBucketLen) {
            peakRing_[ringIdx_] = bucketMax_;
            ringIdx_            = (ringIdx_ + 1) & (kNumBuckets - 1);
            bucketMax_          = 0.0f;
            bucketSamples_      = 0;
        }
        float env = bucketMax_;
        for (float p : peakRing_) env = std::max(env, p);

        float targetGrDb = 0.0f;
        if (env > thresholdLin_ && thresholdLin_ > 1e-6f) {
            constexpr float kSlope = 1.0f - (1.0f / cfg::kCompRatio);   // 0.75 for 4:1
            const float overDb = 20.0f * std::log10(env / thresholdLin_);
            targetGrDb = kSlope * overDb;
        }

        const float grCoeff = targetGrDb > grMeterDb_ ? attackCoeff_ : releaseCoeff_;
        grMeterDb_ += (targetGrDb - grMeterDb_) * grCoeff;
        if (!std::isfinite(grMeterDb_) || grMeterDb_ < 1e-6f) grMeterDb_ = 0.0f;
        gainReductionDb_.store(grMeterDb_, std::memory_order_relaxed);

        const float compGain = grMeterDb_ > 1e-4f ? DbToLin(-grMeterDb_) : 1.0f;
        return sample * (compGain * makeupLin_);
    }

    void  SetThresholdDb(float db) noexcept {
        thresholdDb_.store(std::clamp(db, -48.0f, 0.0f), std::memory_order_relaxed);
    }
    float ThresholdDb() const noexcept { return thresholdDb_.load(std::memory_order_relaxed); }

    void  SetMakeupGainDb(float db) noexcept {
        makeupGainDb_.store(std::clamp(db, -12.0f, 24.0f), std::memory_order_relaxed);
    }
    float MakeupGainDb() const noexcept { return makeupGainDb_.load(std::memory_order_relaxed); }

    float GainReductionDb() const noexcept {
        return gainReductionDb_.load(std::memory_order_relaxed);
    }

private:
    static constexpr size_t kNumBuckets = 16;
    static constexpr size_t kBucketLen  = 20;

    std::atomic<float> thresholdDb_{cfg::kCompDefaultThreshDb};
    std::atomic<float> makeupGainDb_{cfg::kCompDefaultMakeupDb};
    std::atomic<float> gainReductionDb_{0.0f};

    std::array<float, kNumBuckets> peakRing_{};
    size_t ringIdx_        = 0;
    size_t bucketSamples_  = 0;
    float  bucketMax_      = 0.0f;

    float  cachedRate_     = 0.0f;
    float  smoothThreshDb_ = cfg::kCompDefaultThreshDb;
    float  smoothMakeupDb_ = cfg::kCompDefaultMakeupDb;
    float  attackCoeff_    = 1.0f;
    float  releaseCoeff_   = 1.0f;
    float  paramCoeff_     = 1.0f;
    float  thresholdLin_   = 1.0f;
    float  makeupLin_      = 1.0f;
    float  grMeterDb_      = 0.0f;
};

// -------------------------------------------------------------------------------------
//  BassOverdriveNode: 1st-order complementary crossover at 150 Hz splits the signal into
//  a clean tight lowBand (<150 Hz) and a highBand (>150 Hz) driven through asymmetric
//  tube-style soft clipping: tanh(highBand * drive + 0.1) - tanh(0.1) with automatic
//  gain compensation, followed by a 15 Hz post-clipping DC blocker so asymmetric
//  rectification never injects DC offset into downstream EQ/Cab/Limiter stages.
// -------------------------------------------------------------------------------------
class BassOverdriveNode final : public IDspNode {
public:
    float process(float sample, float sampleRate) noexcept override {
        const float targetDrive = drive_.load(std::memory_order_relaxed);
        if (sampleRate != cachedRate_) {
            cachedRate_  = sampleRate;
            lpCoeff_     = static_cast<float>(
                1.0 - std::exp(-2.0 * cfg::kPi * cfg::kOverdriveCrossHz / sampleRate));
            dcR_         = std::exp(-2.0 * cfg::kPi * cfg::kOdDcBlockHz / sampleRate);
            mixCoeff_    = OnePoleCoeff(cfg::kCabToggleMs, sampleRate);
            paramCoeff_  = OnePoleCoeff(cfg::kParamSmoothMs, sampleRate);
            smoothDrive_ = targetDrive;
            driveComp_   = 1.0f / std::pow(std::max(smoothDrive_, 1.0f), 0.55f);
        }

        if (smoothDrive_ != targetDrive) {
            smoothDrive_ += (targetDrive - smoothDrive_) * paramCoeff_;
            if (std::fabs(targetDrive - smoothDrive_) <= 1e-4f) smoothDrive_ = targetDrive;
            driveComp_ = 1.0f / std::pow(std::max(smoothDrive_, 1.0f), 0.55f);
        }

        const float x = std::isfinite(sample) ? sample : 0.0f;

        // Complementary 1st-order crossover at 150 Hz (lowBand + highBand == x)
        lowBand_ += lpCoeff_ * (x - lowBand_);
        if (!std::isfinite(lowBand_)) lowBand_ = 0.0f;
        const float lowBand  = lowBand_;
        const float highBand = x - lowBand;

        // Asymmetric tube-style saturation on highBand (>150 Hz) + auto gain compensation
        constexpr float kBias     = 0.1f;
        const float     kBiasTanh = std::tanh(kBias);
        const float     rawHi     = (std::tanh(highBand * smoothDrive_ + kBias) - kBiasTanh) * driveComp_;

        // Post-saturation 15 Hz DC blocker removes rectified DC offset from asymmetric tanh
        const double dcClean = static_cast<double>(rawHi) - dcX1_ + dcR_ * dcY1_;
        dcX1_ = rawHi;
        dcY1_ = std::isfinite(dcClean) ? dcClean : 0.0;
        const float drivenHi   = static_cast<float>(dcY1_);
        const float recombined = lowBand + drivenHi;

        const bool  isBypassed  = bypass_.load(std::memory_order_relaxed);
        const float targetBlend = isBypassed ? 0.0f : blend_.load(std::memory_order_relaxed);
        smoothBlend_ += (targetBlend - smoothBlend_) * mixCoeff_;

        return x + smoothBlend_ * (recombined - x);
    }

    void  SetBypass(bool b) noexcept { bypass_.store(b, std::memory_order_relaxed); }
    bool  Bypass() const noexcept { return bypass_.load(std::memory_order_relaxed); }
    void  SetDrive(float d) noexcept {
        drive_.store(std::clamp(d, cfg::kDriveMin, cfg::kDriveMax), std::memory_order_relaxed);
    }
    float Drive() const noexcept { return drive_.load(std::memory_order_relaxed); }
    void  SetBlend(float b) noexcept {
        blend_.store(std::clamp(b, cfg::kBlendMin, cfg::kBlendMax), std::memory_order_relaxed);
    }
    float Blend() const noexcept { return blend_.load(std::memory_order_relaxed); }

private:
    std::atomic<bool>  bypass_{true};
    std::atomic<float> drive_{2.0f};
    std::atomic<float> blend_{0.0f};

    float  cachedRate_  = 0.0f;
    float  smoothDrive_ = 2.0f;
    float  lpCoeff_     = 1.0f;
    float  mixCoeff_    = 1.0f;
    float  paramCoeff_  = 1.0f;
    float  driveComp_   = 1.0f;
    float  lowBand_     = 0.0f;
    float  smoothBlend_ = 0.0f;
    double dcR_ = 0.0, dcX1_ = 0.0, dcY1_ = 0.0;
};

// -------------------------------------------------------------------------------------
//  ThreeBandEqNode: 3 cascaded RBJ Audio EQ Cookbook biquad filters in Transposed
//  Direct Form II (Bass low-shelf 80 Hz Q=0.707, Mid peaking 650 Hz Q=1.0, Treble
//  high-shelf 3200 Hz Q=0.707).
//  - Smooths target dB transitions over 15 ms for click-free preset switching, then
//    snaps to exact target so zero sin/cos/pow calls occur during steady-state audio.
// -------------------------------------------------------------------------------------
class ThreeBandEqNode final : public IDspNode {
public:
    float process(float sample, float sampleRate) noexcept override {
        const float targetBass   = bassDb_.load(std::memory_order_relaxed);
        const float targetMid    = midDb_.load(std::memory_order_relaxed);
        const float targetTreble = trebleDb_.load(std::memory_order_relaxed);
        const bool  rateChanged  = (sampleRate != cachedRate_);

        if (rateChanged) {
            cachedRate_     = sampleRate;
            paramCoeff_     = OnePoleCoeff(cfg::kParamSmoothMs, sampleRate);
            smoothBassDb_   = targetBass;
            smoothMidDb_    = targetMid;
            smoothTrebleDb_ = targetTreble;
            UpdateLowShelf(bands_[0], cfg::kEqBassHz, cfg::kEqBassQ, smoothBassDb_, sampleRate);
            UpdatePeaking(bands_[1], cfg::kEqMidHz, cfg::kEqMidQ, smoothMidDb_, sampleRate);
            UpdateHighShelf(bands_[2], cfg::kEqTrebleHz, cfg::kEqTrebleQ, smoothTrebleDb_, sampleRate);
        } else {
            if (smoothBassDb_ != targetBass) {
                smoothBassDb_ += (targetBass - smoothBassDb_) * paramCoeff_;
                if (std::fabs(targetBass - smoothBassDb_) <= 1e-4f) smoothBassDb_ = targetBass;
                UpdateLowShelf(bands_[0], cfg::kEqBassHz, cfg::kEqBassQ, smoothBassDb_, sampleRate);
            }
            if (smoothMidDb_ != targetMid) {
                smoothMidDb_ += (targetMid - smoothMidDb_) * paramCoeff_;
                if (std::fabs(targetMid - smoothMidDb_) <= 1e-4f) smoothMidDb_ = targetMid;
                UpdatePeaking(bands_[1], cfg::kEqMidHz, cfg::kEqMidQ, smoothMidDb_, sampleRate);
            }
            if (smoothTrebleDb_ != targetTreble) {
                smoothTrebleDb_ += (targetTreble - smoothTrebleDb_) * paramCoeff_;
                if (std::fabs(targetTreble - smoothTrebleDb_) <= 1e-4f) smoothTrebleDb_ = targetTreble;
                UpdateHighShelf(bands_[2], cfg::kEqTrebleHz, cfg::kEqTrebleQ, smoothTrebleDb_, sampleRate);
            }
        }

        if (!std::isfinite(sample)) {
            for (Biquad& bq : bands_) {
                bq.z1 = 0.0;
                bq.z2 = 0.0;
            }
            return 0.0f;
        }

        double x = static_cast<double>(sample);
        for (Biquad& bq : bands_) {
            const double y = bq.b0 * x + bq.z1;
            bq.z1 = bq.b1 * x - bq.a1 * y + bq.z2;
            bq.z2 = bq.b2 * x - bq.a2 * y;
            if (!std::isfinite(y) || !std::isfinite(bq.z1) || !std::isfinite(bq.z2)) {
                bq.z1 = 0.0;
                bq.z2 = 0.0;
                x     = 0.0;
            } else {
                x = y;
            }
        }
        return static_cast<float>(x);
    }

    void  SetBassDb(float db) noexcept {
        bassDb_.store(std::clamp(db, cfg::kEqMinDb, cfg::kEqMaxDb), std::memory_order_relaxed);
    }
    float BassDb() const noexcept { return bassDb_.load(std::memory_order_relaxed); }

    void  SetMidDb(float db) noexcept {
        midDb_.store(std::clamp(db, cfg::kEqMinDb, cfg::kEqMaxDb), std::memory_order_relaxed);
    }
    float MidDb() const noexcept { return midDb_.load(std::memory_order_relaxed); }

    void  SetTrebleDb(float db) noexcept {
        trebleDb_.store(std::clamp(db, cfg::kEqMinDb, cfg::kEqMaxDb), std::memory_order_relaxed);
    }
    float TrebleDb() const noexcept { return trebleDb_.load(std::memory_order_relaxed); }

private:
    struct Biquad {
        double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
        double z1 = 0.0, z2 = 0.0;
    };

    static void UpdateLowShelf(Biquad& bq, float fc, float q, float gainDb, float sr) noexcept {
        const double A     = std::pow(10.0, static_cast<double>(gainDb) / 40.0);
        const double w0    = 2.0 * cfg::kPi * static_cast<double>(fc) / static_cast<double>(sr);
        const double cosw  = std::cos(w0);
        const double sinw  = std::sin(w0);
        const double alpha = sinw / (2.0 * static_cast<double>(q));
        const double beta  = 2.0 * std::sqrt(A) * alpha;

        const double a0 = (A + 1.0) + (A - 1.0) * cosw + beta;
        bq.b0 = (A * ((A + 1.0) - (A - 1.0) * cosw + beta)) / a0;
        bq.b1 = (2.0 * A * ((A - 1.0) - (A + 1.0) * cosw)) / a0;
        bq.b2 = (A * ((A + 1.0) - (A - 1.0) * cosw - beta)) / a0;
        bq.a1 = (-2.0 * ((A - 1.0) + (A + 1.0) * cosw)) / a0;
        bq.a2 = ((A + 1.0) + (A - 1.0) * cosw - beta) / a0;
    }

    static void UpdatePeaking(Biquad& bq, float fc, float q, float gainDb, float sr) noexcept {
        const double A     = std::pow(10.0, static_cast<double>(gainDb) / 40.0);
        const double w0    = 2.0 * cfg::kPi * static_cast<double>(fc) / static_cast<double>(sr);
        const double cosw  = std::cos(w0);
        const double sinw  = std::sin(w0);
        const double alpha = sinw / (2.0 * static_cast<double>(q));

        const double a0 = 1.0 + alpha / A;
        bq.b0 = (1.0 + alpha * A) / a0;
        bq.b1 = (-2.0 * cosw) / a0;
        bq.b2 = (1.0 - alpha * A) / a0;
        bq.a1 = bq.b1;
        bq.a2 = (1.0 - alpha / A) / a0;
    }

    static void UpdateHighShelf(Biquad& bq, float fc, float q, float gainDb, float sr) noexcept {
        const double A     = std::pow(10.0, static_cast<double>(gainDb) / 40.0);
        const double w0    = 2.0 * cfg::kPi * static_cast<double>(fc) / static_cast<double>(sr);
        const double cosw  = std::cos(w0);
        const double sinw  = std::sin(w0);
        const double alpha = sinw / (2.0 * static_cast<double>(q));
        const double beta  = 2.0 * std::sqrt(A) * alpha;

        const double a0 = (A + 1.0) - (A - 1.0) * cosw + beta;
        bq.b0 = (A * ((A + 1.0) + (A - 1.0) * cosw + beta)) / a0;
        bq.b1 = (-2.0 * A * ((A - 1.0) + (A + 1.0) * cosw)) / a0;
        bq.b2 = (A * ((A + 1.0) + (A - 1.0) * cosw - beta)) / a0;
        bq.a1 = (2.0 * ((A - 1.0) - (A + 1.0) * cosw)) / a0;
        bq.a2 = ((A + 1.0) - (A - 1.0) * cosw - beta) / a0;
    }

    std::atomic<float> bassDb_{0.0f};
    std::atomic<float> midDb_{0.0f};
    std::atomic<float> trebleDb_{0.0f};

    float  cachedRate_     = 0.0f;
    float  paramCoeff_     = 1.0f;
    float  smoothBassDb_   = 0.0f;
    float  smoothMidDb_    = 0.0f;
    float  smoothTrebleDb_ = 0.0f;
    std::array<Biquad, 3> bands_{};
};

// -------------------------------------------------------------------------------------
//  CabSimNode: toggleable 2nd-order Butterworth LPF "cab sim" (4.5 kHz) with 10 ms
//  click-free crossfade.
// -------------------------------------------------------------------------------------
class CabSimNode final : public IDspNode {
public:
    float process(float sample, float sampleRate) noexcept override {
        if (sampleRate != cachedRate_) UpdateCoefficients(sampleRate);

        const double x  = std::isfinite(sample) ? static_cast<double>(sample) : 0.0;
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
        const double alpha = std::sin(w0) / (2.0 * 0.70710678118654752);
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
    if (!std::isfinite(x)) return 0.0f;
    constexpr float kRange = cfg::kLimiterCeiling - cfg::kLimiterKnee;
    const float ax = std::fabs(x);
    if (ax <= cfg::kLimiterKnee) return x;
    engaged = true;
    return std::copysign(cfg::kLimiterKnee + kRange * std::tanh((ax - cfg::kLimiterKnee) / kRange), x);
}

// -------------------------------------------------------------------------------------
//  Built-In Bass Tone Presets
// -------------------------------------------------------------------------------------
struct TonePreset {
    const char* name;
    float       compThreshDb;
    float       compMakeupDb;
    bool        odBypass;
    float       odDrive;
    float       odBlend;
    float       bassDb;
    float       midDb;
    float       trebleDb;
    bool        cabEnabled;
};

static constexpr std::array<TonePreset, 4> kPresets = {{
    { "Clean Studio DI",        -18.0f, 1.5f, true,   2.0f, 0.00f,  0.0f,  0.0f,  0.0f, false },
    { "Warm Motown Vintage",    -20.0f, 2.5f, true,   3.0f, 0.25f,  4.0f,  2.0f, -6.0f, true  },
    { "Modern Slap Scoop",      -26.0f, 5.0f, true,   4.0f, 0.30f,  5.0f, -6.0f,  6.0f, false },
    { "Darkglass Clank / Grit", -22.0f, 3.5f, false, 12.0f, 0.65f,  4.0f,  3.0f,  4.0f, true  },
}};
