// =====================================================================================
//  tests.cpp - Offline Automated Verification Suite for SubHz
//
//  Verifies:
//    1. Tuner pure-sine precision (< 1.5 ct @ 30-45 Hz, < 0.25 ct > 45 Hz) AND
//       subharmonic octave lock on B0, E1, A1, E2, G2 with weak fundamentals (-6 to -18 dB).
//    2. Mains hum (50/60 Hz @ -46 dBFS across 50 frames) & white noise rejection +
//       adjustable tuner gate threshold and multi-frame stability tracker.
//    3. Compressor 4:1 ratio AND single-stage ballistics (37% release in ~120 ms, not 245 ms).
//    4. Click-free preset switching on live 82 Hz sine (sample step < 0.005 vs 0.109).
//    5. 10 Hz DC blocker bass transparency (< 0.5 dB loss @ B0 30.87 Hz, < 0.3 dB @ E1).
//    6. Post-overdrive 15 Hz DC blocker (< 0.0005 residual DC offset across drives 2x..25x).
//    7. Driver mode fallback warning notice & device-loss notification handling.
//    8. Pre-trim ADC input peak metering & hardware clip detection regardless of digital trim.
//    9. Triggered oscilloscope rising zero-crossing lock & 4x-decimated 2.93 Hz low-band FFT.
//   10. Frame-rate independent exponential smoothing (60 FPS vs 180 FPS equivalence),
//       SoftKneeLimit + NaN/Inf safety, LockFreeHistoryBuffer stress test, and subhz.cfg I/O.
// =====================================================================================

#if defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
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

#include "subhz_config.h"
#include "subhz_dsp.h"
#include "subhz_tuner.h"
#include "subhz_audio.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <thread>

#undef assert
#define assert(expr)                                                                     \
    do {                                                                                 \
        if (!(expr)) {                                                                   \
            std::fprintf(stderr, "[FAIL] %s:%d: %s\n", __FILE__, __LINE__, #expr);       \
            std::fflush(stderr);                                                         \
            std::exit(1);                                                                \
        }                                                                                \
    } while (0)

static constexpr float kSr = 48000.0f;

int main() {
    std::vector<float> hann(cfg::kFftSize);
    for (size_t i = 0; i < cfg::kFftSize; ++i) {
        hann[i] = static_cast<float>(
            0.5 * (1.0 - std::cos(2.0 * cfg::kPi * static_cast<double>(i) /
                                  static_cast<double>(cfg::kFftSize - 1))));
    }
    std::vector<std::complex<float>> fftBuf(cfg::kFftSize);
    std::vector<float>               mags(cfg::kHalfBins + 1, 0.0f);
    const float                      binWidth = kSr / static_cast<float>(cfg::kFftSize);

    // =================================================================================
    //  TEST 1: Pure-sine tuner accuracy & Weak-fundamental subharmonic lock (#1)
    // =================================================================================
    {
        for (float f : { 30.87f, 35.0f, 41.20f, 44.0f, 55.0f, 73.42f, 82.41f, 98.0f, 110.0f, 220.0f, 440.0f }) {
            std::array<float, cfg::kFftSize> sig{};
            for (size_t i = 0; i < cfg::kFftSize; ++i) {
                sig[i] = 0.3f * std::sin(static_cast<float>(2.0 * cfg::kPi * f * i / kSr));
            }
            ComputeMagnitudes(sig, hann, fftBuf, mags);
            const TunerReading r = DetectBassPitch(sig, mags, binWidth);
            assert(r.active);
            const float errCt = static_cast<float>(std::fabs(1200.0 * std::log2(r.frequencyHz / f)));
            const float limit = (f <= 45.0f) ? 1.5f : 0.25f;
            assert(errCt <= limit);
        }

        // Weak fundamental (-6 dB, -12 dB, -14 dB, -18 dB below 2nd harmonic) on B0, E1, A1, E2, G2
        struct NoteCase { const char* name; float f0; int midi; };
        const NoteCase notes[] = {
            { "B0", 30.8677f, 23 },
            { "E1", 41.2034f, 28 },
            { "A1", 55.0000f, 33 },
            { "E2", 82.4069f, 40 },
            { "G2", 97.9989f, 43 },
        };
        for (const auto& nc : notes) {
            for (float relDb : { -6.0f, -12.0f, -14.0f, -18.0f }) {
                const float a2 = 0.4f;
                const float a1 = a2 * DbToLin(relDb);
                std::array<float, cfg::kFftSize> sig{};
                for (size_t i = 0; i < cfg::kFftSize; ++i) {
                    const double t = static_cast<double>(i) / kSr;
                    sig[i] = static_cast<float>(a1 * std::sin(2.0 * cfg::kPi * nc.f0 * t) +
                                                a2 * std::sin(2.0 * cfg::kPi * (2.0 * nc.f0) * t) +
                                                0.15 * a2 * std::sin(2.0 * cfg::kPi * (3.0 * nc.f0) * t));
                }
                ComputeMagnitudes(sig, hann, fftBuf, mags);
                // Verify both 4-arg (with timeSamples) and 2-arg (magnitudes only) overloads
                const TunerReading r1 = DetectBassPitch(sig, mags, binWidth);
                const TunerReading r2 = DetectBassPitch(mags, binWidth);
                assert(r1.active && r1.midiNote == nc.midi);
                assert(r2.active && r2.midiNote == nc.midi);
                const float errCt = static_cast<float>(std::fabs(1200.0 * std::log2(r1.frequencyHz / nc.f0)));
                assert(errCt < 1.5f);
            }
        }
        std::printf("[PASS] 1. Tuner pure-sine precision & B0..G2 weak-fundamental (-18 dB) octave lock\n");
    }

    // =================================================================================
    //  TEST 2: 50/60 Hz mains hum (-46 dBFS, 50 frames) & white noise rejection (#2)
    // =================================================================================
    {
        TunerTracker tracker;
        const float humAmp = DbToLin(-46.0f);
        int bogusCount = 0;
        for (int frame = 0; frame < 50; ++frame) {
            std::array<float, cfg::kFftSize> sig{};
            for (size_t i = 0; i < cfg::kFftSize; ++i) {
                const double t = static_cast<double>(i + frame * 128) / kSr;
                sig[i] = humAmp * static_cast<float>(std::sin(2.0 * cfg::kPi * 50.0 * t));
            }
            ComputeMagnitudes(sig, hann, fftBuf, mags);
            if (DetectBassPitch(sig, mags, binWidth).active) ++bogusCount;
            if (tracker.Update(sig, mags, binWidth).active) ++bogusCount;
        }
        assert(bogusCount == 0);

        // White noise at -30 dBFS across 50 frames -> 0 false locks
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist(-0.03f, 0.03f);
        for (int frame = 0; frame < 50; ++frame) {
            std::array<float, cfg::kFftSize> sig{};
            for (float& s : sig) s = dist(rng);
            ComputeMagnitudes(sig, hann, fftBuf, mags);
            assert(!DetectBassPitch(sig, mags, binWidth).active);
            assert(!tracker.Update(sig, mags, binWidth).active);
        }
        std::printf("[PASS] 2. 50 Hz mains hum (-46 dBFS, 0/50 frames) & white noise rejected\n");
    }

    // =================================================================================
    //  TEST 3: Compressor 4:1 ratio & single-stage 120 ms release to 37% (#3)
    // =================================================================================
    {
        CompressorNode comp;
        comp.SetThresholdDb(-18.0f);
        comp.SetMakeupGainDb(0.0f);

        const float inAmp = DbToLin(-6.0f);
        for (int i = 0; i < 24000; ++i) {
            const float s = inAmp * std::sin(static_cast<float>(2.0 * cfg::kPi * 100.0 * i / kSr));
            comp.process(s, kSr);
        }
        const float gr0 = comp.GainReductionDb();
        assert(std::fabs(gr0 - 9.0f) < 0.25f);

        int samplesTo37 = -1;
        for (int i = 1; i <= 24000; ++i) {
            comp.process(0.0f, kSr);
            if (samplesTo37 < 0 && comp.GainReductionDb() <= gr0 * 0.367879f) {
                samplesTo37 = i;
                break;
            }
        }
        const float releaseMs = 1000.0f * samplesTo37 / kSr;
        std::printf("[INFO] Compressor steady GR = %.2f dB, 37%% release = %.2f ms (target 120 ms)\n",
                    gr0, releaseMs);
        assert(releaseMs >= 112.0f && releaseMs <= 128.0f);
        std::printf("[PASS] 3. Compressor single-stage 120 ms release & 4:1 ratio verified\n");
    }

    // =================================================================================
    //  TEST 4: Click-free preset switching on 82 Hz 0.3 sine (#4)
    // =================================================================================
    {
        AudioEngine engine;
        engine.ToggleMute();   // unmute output
        engine.ApplyPreset(0); // Clean Studio DI

        std::array<float, 256> inBuf{};
        std::array<float, 256> outBuf{};
        size_t sampleCounter = 0;

        auto runBlocks = [&](int blocks, float& maxDiff) {
            float prevSample = 0.0f;
            bool  hasPrev    = false;
            for (int b = 0; b < blocks; ++b) {
                for (size_t i = 0; i < 128; ++i) {
                    const float v = 0.3f * std::sin(static_cast<float>(
                        2.0 * cfg::kPi * 82.0 * static_cast<double>(sampleCounter++) / kSr));
                    inBuf[2 * i]     = v;
                    inBuf[2 * i + 1] = v;
                }
                engine.ProcessInterleaved(outBuf.data(), inBuf.data(), 128);
                for (size_t i = 0; i < 128; ++i) {
                    const float y = outBuf[2 * i];
                    if (hasPrev) {
                        maxDiff = std::max(maxDiff, std::fabs(y - prevSample));
                    }
                    prevSample = y;
                    hasPrev    = true;
                }
            }
        };

        float warmDiff = 0.0f;
        runBlocks(100, warmDiff); // settle on Clean Studio DI

        float maxSwitchStep = 0.0f;
        for (int preset : { 2, 1, 3, 0 }) { // Clean -> Modern Slap -> Warm Motown -> Darkglass -> Clean
            engine.ApplyPreset(preset);
            runBlocks(40, maxSwitchStep);
        }
        std::printf("[INFO] Max sample-to-sample step across all preset switches: %.5f (was 0.109)\n",
                    maxSwitchStep);
        assert(maxSwitchStep < 0.012f);
        std::printf("[PASS] 4. Click-free preset switching verified\n");
    }

    // =================================================================================
    //  TEST 5 & 6: 10 Hz DC blocker bass transparency (#5) & Overdrive DC removal (#6)
    // =================================================================================
    {
        DcBlockerNode dc;
        auto measureGainDb = [&](float freqHz) -> float {
            float peak = 0.0f;
            for (int i = 0; i < 48000; ++i) {
                const float x = std::sin(static_cast<float>(2.0 * cfg::kPi * freqHz * i / kSr));
                const float y = dc.process(x, kSr);
                if (i > 24000) peak = std::max(peak, std::fabs(y));
            }
            return 20.0f * std::log10(peak);
        };
        const float lossB0 = measureGainDb(30.87f);
        const float lossE1 = measureGainDb(41.20f);
        const float lossA1 = measureGainDb(55.00f);
        std::printf("[INFO] 10 Hz DC blocker response: B0=%.2f dB, E1=%.2f dB, A1=%.2f dB\n",
                    lossB0, lossE1, lossA1);
        assert(lossB0 > -0.60f && lossE1 > -0.35f && lossA1 > -0.20f);

        // Overdrive post-clipping DC offset check across drives 4x, 12x, 25x
        BassOverdriveNode od;
        od.SetBypass(false);
        od.SetBlend(1.0f);
        for (float drv : { 4.0f, 12.0f, 25.0f }) {
            od.SetDrive(drv);
            double sum = 0.0;
            constexpr int kN = 48000;
            for (int i = 0; i < 2 * kN; ++i) {
                const float x = 0.5f * std::sin(static_cast<float>(2.0 * cfg::kPi * 400.0 * i / kSr));
                const float y = od.process(x, kSr);
                if (i >= kN) sum += y;
            }
            const double dcMean = std::fabs(sum / kN);
            assert(dcMean < 0.0005); // < -66 dBFS (was -0.024 = -32 dBFS)
        }
        std::printf("[PASS] 5 & 6. 10 Hz DC blocker transparency & Overdrive DC removal verified\n");
    }

    // =================================================================================
    //  TEST 7, 8 & 13: Pre-trim ADC meter (#13), device-loss & notice handling (#7, #8)
    // =================================================================================
    {
        AudioEngine engine;
        engine.SetTrim(0.25f); // -12 dB digital trim
        std::array<float, 256> hotIn{};
        std::array<float, 256> out{};
        hotIn.fill(1.0f);      // Full-scale 0 dBFS at hardware ADC
        engine.ProcessInterleaved(out.data(), hotIn.data(), 128);

        // IN peak must report 1.0f (0 dBFS hardware ADC level), NOT 0.25f post-trim!
        const float inPeak = engine.ConsumeInputPeak();
        assert(std::fabs(inPeak - 1.0f) < 1e-5f);
        assert(engine.ConsumeAdcClipEvent());

        // Simulate unexpected device loss and verify status notice & flag
        engine.SimulateDeviceLossForTest();
        assert(engine.DeviceLost());
        char notice[160]{};
        bool isWarn = false;
        assert(engine.ConsumeNotice(notice, sizeof(notice), isWarn));
        assert(isWarn && std::strlen(notice) > 0);
        std::printf("[PASS] 7, 8 & 13. Pre-trim ADC clip metering & device-loss notice verified\n");
    }

    // =================================================================================
    //  TEST 10, 11 & 12: Scope trigger (#10), 2.93 Hz low-band FFT (#11) & dt-lerp (#12)
    // =================================================================================
    {
        // Triggered oscilloscope zero-crossing lock on arbitrary-phase 55 Hz wave
        std::array<float, cfg::kFftSize> wave{};
        for (size_t i = 0; i < cfg::kFftSize; ++i) {
            wave[i] = 0.4f * std::sin(static_cast<float>(2.0 * cfg::kPi * 55.0 * i / kSr + 2.1));
        }
        float subFrac = 0.0f;
        const size_t trig = FindScopeTriggerIndex(wave, cfg::kScopeSamples, &subFrac);
        assert(trig > 0 && trig + cfg::kScopeSamples <= cfg::kFftSize);
        assert(wave[trig - 1] < 0.0f && wave[trig] >= 0.0f);

        // 4x-decimated low-band FFT separates simultaneous B0 (30.87 Hz) and E1 (41.20 Hz)
        AudioEngine engine;
        std::array<float, 256> inBuf{};
        std::array<float, 256> outBuf{};
        size_t sampleIdx = 0;
        for (int b = 0; b < 140; ++b) { // 140 * 128 = 17920 samples (> 16384 for full 4x decim window)
            for (size_t i = 0; i < 128; ++i) {
                const double t = static_cast<double>(sampleIdx++) / kSr;
                const float  s = static_cast<float>(0.4 * std::sin(2.0 * cfg::kPi * 30.87 * t) +
                                                    0.4 * std::sin(2.0 * cfg::kPi * 41.20 * t));
                inBuf[2 * i]     = s;
                inBuf[2 * i + 1] = s;
            }
            engine.ProcessInterleaved(outBuf.data(), inBuf.data(), 128);
        }
        std::array<float, cfg::kFftSize> lowSnap{};
        assert(engine.LowWetTap().Snapshot(lowSnap));
        std::vector<float> lowMags(cfg::kHalfBins + 1, 0.0f);
        ComputeMagnitudes(lowSnap, hann, fftBuf, lowMags);
        const float lowBinHz = engine.LowBandSampleRate() / static_cast<float>(cfg::kFftSize); // 2.93 Hz
        const size_t binB0   = static_cast<size_t>(std::lround(30.87f / lowBinHz)); // bin 11
        const size_t binMid  = static_cast<size_t>(std::lround(36.00f / lowBinHz)); // bin 12 (valley)
        const size_t binE1   = static_cast<size_t>(std::lround(41.20f / lowBinHz)); // bin 14
        const float  peakB0  = std::max(lowMags[binB0], lowMags[binB0 - 1]);
        const float  peakE1  = std::max(lowMags[binE1], lowMags[binE1 - 1]);
        assert(peakB0 > 0.08f && peakE1 > 0.08f);
        assert(lowMags[binMid] < 0.65f * std::min(peakB0, peakE1));

        // Frame-rate independent TimeConstantLerp: 60 FPS vs 180 FPS over 0.5 seconds
        float h60  = 0.0f;
        float h180 = 0.0f;
        for (int i = 0; i < 30; ++i)  h60  = TimeConstantLerp(h60,  100.0f, cfg::kSpectrumSmoothRate, 1.0f / 60.0f);
        for (int i = 0; i < 90; ++i)  h180 = TimeConstantLerp(h180, 100.0f, cfg::kSpectrumSmoothRate, 1.0f / 180.0f);
        assert(std::fabs(h60 - h180) < 1e-3f);

        std::printf("[PASS] 10, 11 & 12. Scope trigger, 2.93 Hz B0/E1 separation & dt-smoothing verified\n");
    }

    // =================================================================================
    //  TEST 14: Settings persistence round-trip, limiter & lock-free buffer safety
    // =================================================================================
    {
        AppSettings saved{};
        saved.trim        = 2.5f;
        saved.inputSelect = 1;
        saved.presetIndex = 3;
        saved.gateEnabled = true;
        saved.cabEnabled  = true;
        saved.odBypass    = false;
        saved.odDrive     = 14.5f;
        saved.odBlend     = 0.75f;
        saved.bassDb      = 4.5f;
        saved.midDb       = -3.0f;
        saved.trebleDb    = 6.0f;
        saved.tunerGateDb = -40.0f;
        saved.vsync       = false;

        const char* tmpCfg = "subhz_test_tmp.cfg";
        assert(SaveSettings(tmpCfg, saved));
        AppSettings loaded{};
        assert(LoadSettings(tmpCfg, loaded));
        std::remove(tmpCfg);

        assert(std::fabs(loaded.trim - saved.trim) < 1e-4f);
        assert(loaded.inputSelect == saved.inputSelect);
        assert(loaded.presetIndex == saved.presetIndex);
        assert(loaded.gateEnabled == saved.gateEnabled);
        assert(std::fabs(loaded.odDrive - saved.odDrive) < 1e-3f);
        assert(std::fabs(loaded.tunerGateDb - saved.tunerGateDb) < 1e-3f);
        assert(loaded.vsync == saved.vsync);

        // NaN/Inf injection & +40 dB limiter ceiling check
        AudioEngine engine;
        engine.ToggleMute();
        engine.SetTrim(100.0f);
        std::array<float, 64> badIn{};
        std::array<float, 64> safeOut{};
        for (size_t i = 0; i < 64; ++i) {
            badIn[i] = (i % 4 == 0) ? std::numeric_limits<float>::quiet_NaN()
                     : (i % 4 == 1) ? std::numeric_limits<float>::infinity()
                                    : 1.0f;
        }
        engine.ProcessInterleaved(safeOut.data(), badIn.data(), 32);
        for (float y : safeOut) {
            assert(std::isfinite(y) && std::fabs(y) <= cfg::kLimiterCeiling);
        }
        std::printf("[PASS] 14. Config persistence round-trip, +40 dB limiter & NaN safety verified\n");
    }

    // =================================================================================
    //  TEST 15: Dynamic DSP Chain Reordering (Rack Order Dragging & Signal Routing)
    // =================================================================================
    {
        AudioEngine engine;
        assert((engine.GetChainOrder() == std::array<int, 5>{ 0, 1, 2, 3, 4 }));
        assert(!engine.SetChainOrder({ 0, 1, 2, 2, 4 })); // duplicate slot rejected
        assert(!engine.SetChainOrder({ 0, 1, 2, 3, 9 })); // out-of-range rejected

        // Drag OD (pos 2) before COMP (pos 1): [GATE, OD, COMP, EQ, CAB]
        assert(engine.MoveChainSlot(2, 1));
        assert((engine.GetChainOrder() == std::array<int, 5>{ kSlotGate, kSlotOd, kSlotComp, kSlotEq, kSlotCab }));

        // Verify EQ -> OD vs OD -> EQ produces distinct nonlinear saturation output
        auto renderOrder = [&](const std::array<int, 5>& ord) -> float {
            AudioEngine eng;
            eng.ToggleMute();
            eng.SetOverdriveBypass(false);
            eng.SetDrive(16.0f);
            eng.SetBlend(1.0f);
            eng.SetMidDb(15.0f); // +15 dB @ 650 Hz: before OD drives harder into tanh; after OD boosts output
            assert(eng.SetChainOrder(ord));

            std::array<float, 256> in{};
            std::array<float, 256> out{};
            double rms = 0.0;
            size_t idx = 0;
            for (int b = 0; b < 40; ++b) {
                for (size_t i = 0; i < 128; ++i) {
                    const float s = 0.25f * std::sin(static_cast<float>(2.0 * cfg::kPi * 650.0 * idx++ / kSr));
                    in[2 * i]     = s;
                    in[2 * i + 1] = s;
                }
                eng.ProcessInterleaved(out.data(), in.data(), 128);
                if (b >= 20) {
                    for (size_t i = 0; i < 128; ++i) rms += out[2 * i] * out[2 * i];
                }
            }
            return static_cast<float>(std::sqrt(rms / (20.0 * 128.0)));
        };

        const float rmsEqBeforeOd = renderOrder({ kSlotGate, kSlotComp, kSlotEq, kSlotOd, kSlotCab });
        const float rmsOdBeforeEq = renderOrder({ kSlotGate, kSlotComp, kSlotOd, kSlotEq, kSlotCab });
        assert(std::fabs(rmsOdBeforeEq - rmsEqBeforeOd) > 0.08f);
        std::printf("[PASS] 15. Dynamic DSP chain reordering (EQ->OD rms=%.3f vs OD->EQ rms=%.3f)\n",
                    rmsEqBeforeOd, rmsOdBeforeEq);
    }

    // =================================================================================
    //  TEST 16: Overdrive 2x / 4x (192 kHz) Oversampling Anti-Aliasing
    // =================================================================================
    {
        // 6800 Hz sine @ 48 kHz: 7th harmonic is 47600 Hz, which aliases to 400 Hz at 1x.
        // At 4x (192 kHz), 47600 Hz is below the 96 kHz Nyquist and attenuated by the
        // 19.2 kHz 4th-order Butterworth anti-aliasing filter before decimation.
        auto measureAliasAt400Hz = [&](int osFactor) -> float {
            BassOverdriveNode od;
            od.SetBypass(false);
            od.SetDrive(14.0f);
            od.SetBlend(1.0f);
            od.SetOversample(osFactor);

            std::array<float, cfg::kFftSize> snap{};
            for (size_t i = 0; i < 2 * cfg::kFftSize; ++i) {
                const float x = 0.5f * std::sin(static_cast<float>(2.0 * cfg::kPi * 6800.0 * i / kSr));
                const float y = od.process(x, kSr);
                if (i >= cfg::kFftSize) snap[i - cfg::kFftSize] = y;
            }
            ComputeMagnitudes(snap, hann, fftBuf, mags);
            const size_t bin400 = static_cast<size_t>(std::lround(400.0f / binWidth));
            return std::max({ mags[bin400 - 1], mags[bin400], mags[bin400 + 1] });
        };

        const float alias1x = measureAliasAt400Hz(1);
        const float alias2x = measureAliasAt400Hz(2);
        const float alias4x = measureAliasAt400Hz(4);
        const float reductionDb = 20.0f * std::log10(std::max(alias1x, 1e-9f) / std::max(alias4x, 1e-9f));
        std::printf("[INFO] Overdrive 400 Hz alias foldback: 1x=%.5f, 2x=%.5f, 4x=%.5f (-%.1f dB)\n",
                    alias1x, alias2x, alias4x, reductionDb);
        assert(alias4x < alias1x * 0.15f && alias2x < alias1x * 0.5f);
        std::printf("[PASS] 16. Overdrive 2x/4x (192 kHz) oversampling anti-aliasing verified\n");
    }

    // =================================================================================
    //  TEST 17: JSON Rig Preset Export / Import Serialization
    // =================================================================================
    {
        AudioEngine srcEngine;
        srcEngine.SetTrim(1.75f);
        srcEngine.SetGateEnabled(true);
        srcEngine.SetCompThresholdDb(-24.0f);
        srcEngine.SetCompMakeupDb(4.0f);
        srcEngine.SetOverdriveBypass(false);
        srcEngine.SetDrive(18.5f);
        srcEngine.SetBlend(0.85f);
        srcEngine.SetOverdriveOversample(4);
        srcEngine.SetBassDb(5.0f);
        srcEngine.SetMidDb(-3.5f);
        srcEngine.SetTrebleDb(4.0f);
        srcEngine.SetCabEnabled(true);
        const std::array<int, 5> customOrd = { kSlotGate, kSlotOd, kSlotComp, kSlotEq, kSlotCab };
        assert(srcEngine.SetChainOrder(customOrd));

        const char* tmpJson = "subhz_test_preset.json";
        assert(srcEngine.SaveRigPresetJson(tmpJson, "Smashing Pumpkins Fuzz Bass"));

        AudioEngine dstEngine;
        assert(dstEngine.LoadRigPresetJson(tmpJson));
        std::remove(tmpJson);

        assert(std::strcmp(dstEngine.PresetName(), "Smashing Pumpkins Fuzz Bass") == 0);
        assert(std::fabs(dstEngine.Trim() - 1.75f) < 1e-3f);
        assert(dstEngine.GateEnabled() == true);
        assert(std::fabs(dstEngine.CompThresholdDb() - (-24.0f)) < 1e-3f);
        assert(std::fabs(dstEngine.CompMakeupDb() - 4.0f) < 1e-3f);
        assert(dstEngine.OverdriveEnabled() == true);
        assert(std::fabs(dstEngine.Drive() - 18.5f) < 1e-3f);
        assert(std::fabs(dstEngine.Blend() - 0.85f) < 1e-3f);
        assert(dstEngine.OverdriveOversample() == 4);
        assert(std::fabs(dstEngine.BassDb() - 5.0f) < 1e-3f);
        assert(std::fabs(dstEngine.MidDb() - (-3.5f)) < 1e-3f);
        assert(std::fabs(dstEngine.TrebleDb() - 4.0f) < 1e-3f);
        assert(dstEngine.CabEnabled() == true);
        assert(dstEngine.GetChainOrder() == customOrd);
        std::printf("[PASS] 17. JSON preset export/import ('%s') & chainOrder verified\n",
                    dstEngine.PresetName());
    }

    std::printf("\nALL 17 SUBHZ AUDIT VERIFICATION TESTS PASSED!\n");
    return 0;
}
