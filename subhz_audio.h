// =====================================================================================
//  subhz_audio.h - WASAPI Full-Duplex AudioEngine, Device Enumeration & Loss Recovery
// =====================================================================================
#pragma once

#include "subhz_dsp.h"
#include <memory>
#include <string>
#include <vector>

enum class InputSelect : int { In1 = 0, In2 = 1, Sum = 2 };

struct LatencyInfo {
    ma_uint32 periodFrames = 0;
    ma_uint32 periods      = 0;
    ma_uint32 sampleRate   = 0;
    float     periodMs     = 0.0f;
    float     roundTripMs  = 0.0f;   // estimate: capture period + playback buffer
};

struct EndpointEntry {
    ma_device_id id{};
    char         name[MA_MAX_DEVICE_NAME_LENGTH + 1]{};
    bool         isDefault = false;
};

class AudioEngine {
public:
    using History = LockFreeHistoryBuffer<cfg::kFftSize>;

    AudioEngine() {
        // Chain is fully built (and never resized) before the device starts:
        // [DcBlocker 10Hz] -> [NoiseGate] -> [Compressor] -> [Overdrive] -> [3BandEQ] -> [CabSim]
        auto dcBlocker = std::make_unique<DcBlockerNode>();
        auto gate      = std::make_unique<EnvelopeNoiseGateNode>();
        auto comp      = std::make_unique<CompressorNode>();
        auto overdrive = std::make_unique<BassOverdriveNode>();
        auto eq        = std::make_unique<ThreeBandEqNode>();
        auto cab       = std::make_unique<CabSimNode>();
        dcBlocker_ = dcBlocker.get();
        gate_      = gate.get();
        comp_      = comp.get();
        overdrive_ = overdrive.get();
        eq_        = eq.get();
        cab_       = cab.get();
        dspChain_.reserve(6);
        dspChain_.push_back(std::move(dcBlocker));
        dspChain_.push_back(std::move(gate));
        dspChain_.push_back(std::move(comp));
        dspChain_.push_back(std::move(overdrive));
        dspChain_.push_back(std::move(eq));
        dspChain_.push_back(std::move(cab));
        ApplyPreset(0);
        SetSampleRate(static_cast<float>(cfg::kSampleRate));
    }
    ~AudioEngine() { Stop(); }
    AudioEngine(const AudioEngine&)            = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    // Opens WASAPI full duplex: shared mode first by default (preserves mic-array APOs),
    // exclusive mode as fallback (or primary when preferExclusive == true).
    bool Start(bool preferExclusive = false) {
        if (running_) return true;
        deviceLost_.store(false, std::memory_order_relaxed);
        stopping_.store(false, std::memory_order_relaxed);

#if defined(_WIN32)
        const ma_backend backends[] = { ma_backend_wasapi };
        if (ma_context_init(backends, 1, nullptr, &context_) != MA_SUCCESS) {
            SetNotice("FAILED TO INITIALIZE WASAPI CONTEXT", true);
            return false;
        }
#else
        if (ma_context_init(nullptr, 0, nullptr, &context_) != MA_SUCCESS) {
            SetNotice("FAILED TO INITIALIZE AUDIO CONTEXT", true);
            return false;
        }
#endif
        contextOk_ = true;
        EnumerateEndpoints();

        const ma_share_mode first  = preferExclusive ? ma_share_mode_exclusive : ma_share_mode_shared;
        const ma_share_mode second = preferExclusive ? ma_share_mode_shared    : ma_share_mode_exclusive;
        if (TryStart(first)) {
            exclusive_ = (first == ma_share_mode_exclusive);
        } else if (TryStart(second)) {
            exclusive_ = (second == ma_share_mode_exclusive);
            if (preferExclusive && !exclusive_) {
                SetNotice("EXCLUSIVE MODE BUSY/UNSUPPORTED - FALLBACK TO WASAPI SHARED", true);
            }
        } else {
            ma_context_uninit(&context_);
            contextOk_ = false;
            SetNotice("NO DUPLEX AUDIO ENDPOINT AVAILABLE", true);
            return false;
        }
        running_ = true;
        return true;
    }

    // Returns true if the requested driver mode was actually entered; sets an explicit
    // HUD warning banner if Exclusive mode was rejected and fell back to Shared mode.
    bool ToggleDriverMode() {
        const bool targetExclusive = !exclusive_;
        Stop();
        if (!Start(targetExclusive)) return false;
        if (exclusive_ == targetExclusive) {
            SetNotice(exclusive_ ? "DRIVER MODE: WASAPI EXCLUSIVE (PRO AUDIO)"
                                 : "DRIVER MODE: WASAPI SHARED",
                      false);
            return true;
        }
        // Start() already set the fallback warning notice
        return false;
    }

    // Cycle capture (input) hardware endpoint: -1 (System Default) -> 0 -> 1 ...
    bool CycleCaptureDevice() {
        const bool keepExclusive = exclusive_;
        Stop();
        // Briefly init context to refresh device list if needed
#if defined(_WIN32)
        const ma_backend backends[] = { ma_backend_wasapi };
        if (ma_context_init(backends, 1, nullptr, &context_) == MA_SUCCESS) {
            contextOk_ = true;
            EnumerateEndpoints();
            ma_context_uninit(&context_);
            contextOk_ = false;
        }
#endif
        if (captureEndpoints_.empty()) {
            selectedCaptureIdx_ = -1;
        } else {
            selectedCaptureIdx_ += 1;
            if (selectedCaptureIdx_ >= static_cast<int>(captureEndpoints_.size())) {
                selectedCaptureIdx_ = -1;
            }
        }
        const bool ok = Start(keepExclusive);
        if (ok) {
            char msg[160]{};
            std::snprintf(msg, sizeof(msg), "INPUT DEVICE [D]: %.110s", captureName_);
            SetNotice(msg, false);
        }
        return ok;
    }

    // Cycle playback (output) hardware endpoint: -1 (System Default) -> 0 -> 1 ...
    bool CyclePlaybackDevice() {
        const bool keepExclusive = exclusive_;
        Stop();
#if defined(_WIN32)
        const ma_backend backends[] = { ma_backend_wasapi };
        if (ma_context_init(backends, 1, nullptr, &context_) == MA_SUCCESS) {
            contextOk_ = true;
            EnumerateEndpoints();
            ma_context_uninit(&context_);
            contextOk_ = false;
        }
#endif
        if (playbackEndpoints_.empty()) {
            selectedPlaybackIdx_ = -1;
        } else {
            selectedPlaybackIdx_ += 1;
            if (selectedPlaybackIdx_ >= static_cast<int>(playbackEndpoints_.size())) {
                selectedPlaybackIdx_ = -1;
            }
        }
        const bool ok = Start(keepExclusive);
        if (ok) {
            char msg[160]{};
            std::snprintf(msg, sizeof(msg), "OUTPUT DEVICE [F2]: %.110s", playbackName_);
            SetNotice(msg, false);
        }
        return ok;
    }

    void Stop() noexcept {
        stopping_.store(true, std::memory_order_relaxed);
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
        callbackCount_.fetch_add(1, std::memory_order_relaxed);

        const int   select     = inputSelect_.load(std::memory_order_relaxed);
        const float trim       = trim_.load(std::memory_order_relaxed);
        const float muteTarget = muted_.load(std::memory_order_relaxed) ? 0.0f : 1.0f;
        const float sr         = sampleRate_;

        float inPeak     = 0.0f;
        float outPeak    = 0.0f;
        bool  limited    = false;
        bool  adcClipped = false;

        ma_uint32 done = 0;
        while (done < frameCount) {
            const ma_uint32 n = std::min<ma_uint32>(frameCount - done, kScratchFrames);
            size_t lowCount = 0;

            for (ma_uint32 i = 0; i < n; ++i) {
                const ma_uint32 frame = done + i;

                float rawX = 0.0f;
                if (input != nullptr) {
                    const float l = input[frame * cfg::kChannelsIn];
                    const float r = input[frame * cfg::kChannelsIn + 1];
                    rawX = (select == 0) ? l : (select == 1) ? r : 0.5f * (l + r);
                }
                if (!std::isfinite(rawX)) rawX = 0.0f;

                // Measure IN peak BEFORE digital trim so hardware ADC clipping is always visible
                const float absRaw = std::fabs(rawX);
                inPeak = std::max(inPeak, absRaw);
                if (absRaw >= 0.999f) adcClipped = true;

                float x = rawX * trim;
                if (!std::isfinite(x)) x = 0.0f;

                for (const auto& node : dspChain_) x = node->process(x, sr);

                dryScratch_[i] = dcBlocker_->LastOutput();        // Pre-FX tap
                const float y  = SoftKneeLimit(x, limited);
                wetScratch_[i] = y;                               // Post-FX tap (pre-mute)

                // 2-stage anti-aliasing LPF (~1.8 kHz) + 4x decimation for 2.93 Hz low-band FFT
                lowAa1_ += lowAaCoeff_ * (y - lowAa1_);
                lowAa2_ += lowAaCoeff_ * (lowAa1_ - lowAa2_);
                if (!std::isfinite(lowAa1_) || !std::isfinite(lowAa2_)) {
                    lowAa1_ = 0.0f;
                    lowAa2_ = 0.0f;
                }
                if (++lowDecimPhase_ >= cfg::kLowBandDecim) {
                    lowDecimPhase_ = 0;
                    lowWetScratch_[lowCount++] = lowAa2_;
                }

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
            if (lowCount > 0) lowWetBuffer_.Write(lowWetScratch_.data(), lowCount);
            done += n;
        }

        AtomicMax(inputPeak_, inPeak);
        AtomicMax(outputPeak_, outPeak);
        if (limited)    limitEvent_.store(true, std::memory_order_relaxed);
        if (adcClipped) adcClipEvent_.store(true, std::memory_order_relaxed);
    }

    // ---- UI controls (UI thread) ----------------------------------------------------
    void CycleInput() noexcept {
        inputSelect_.store((inputSelect_.load(std::memory_order_relaxed) + 1) % 3,
                           std::memory_order_relaxed);
    }
    void SetInputSelect(InputSelect sel) noexcept {
        inputSelect_.store(std::clamp(static_cast<int>(sel), 0, 2), std::memory_order_relaxed);
    }
    InputSelect Input() const noexcept {
        return static_cast<InputSelect>(inputSelect_.load(std::memory_order_relaxed));
    }
    void  ToggleMute() noexcept { muted_.store(!Muted(), std::memory_order_relaxed); }
    bool  Muted() const noexcept { return muted_.load(std::memory_order_relaxed); }
    void  ToggleCab() noexcept {
        cab_->SetCabEnabled(!cab_->CabEnabled());
        presetModified_ = true;
    }
    void  SetCabEnabled(bool on) noexcept {
        cab_->SetCabEnabled(on);
        presetModified_ = true;
    }
    bool  CabEnabled() const noexcept { return cab_->CabEnabled(); }
    void  ToggleGate() noexcept { gate_->SetEnabled(!gate_->Enabled()); }
    void  SetGateEnabled(bool on) noexcept { gate_->SetEnabled(on); }
    bool  GateEnabled() const noexcept { return gate_->Enabled(); }
    void  SetTrim(float g) noexcept {
        trim_.store(std::clamp(g, cfg::kTrimMin, cfg::kTrimMax), std::memory_order_relaxed);
    }
    float Trim() const noexcept { return trim_.load(std::memory_order_relaxed); }
    bool  GateOpen() const noexcept { return gate_->IsOpen(); }

    // ---- Phase 2 DSP controls & presets ---------------------------------------------
    void  ToggleOverdrive() noexcept {
        overdrive_->SetBypass(!overdrive_->Bypass());
        presetModified_ = true;
    }
    void  SetOverdriveBypass(bool bypass) noexcept {
        overdrive_->SetBypass(bypass);
        presetModified_ = true;
    }
    bool  OverdriveEnabled() const noexcept { return !overdrive_->Bypass(); }
    void  SetDrive(float d) noexcept {
        overdrive_->SetDrive(d);
        presetModified_ = true;
    }
    float Drive() const noexcept { return overdrive_->Drive(); }
    void  SetBlend(float b) noexcept {
        overdrive_->SetBlend(b);
        presetModified_ = true;
    }
    float Blend() const noexcept { return overdrive_->Blend(); }

    void  SetBassDb(float db) noexcept {
        eq_->SetBassDb(db);
        presetModified_ = true;
    }
    float BassDb() const noexcept { return eq_->BassDb(); }
    void  SetMidDb(float db) noexcept {
        eq_->SetMidDb(db);
        presetModified_ = true;
    }
    float MidDb() const noexcept { return eq_->MidDb(); }
    void  SetTrebleDb(float db) noexcept {
        eq_->SetTrebleDb(db);
        presetModified_ = true;
    }
    float TrebleDb() const noexcept { return eq_->TrebleDb(); }

    float CompGainReductionDb() const noexcept { return comp_->GainReductionDb(); }
    float CompThresholdDb() const noexcept { return comp_->ThresholdDb(); }
    float CompMakeupDb() const noexcept { return comp_->MakeupGainDb(); }

    void ApplyPreset(int index) noexcept {
        presetIndex_    = ((index % static_cast<int>(kPresets.size())) +
                           static_cast<int>(kPresets.size())) %
                          static_cast<int>(kPresets.size());
        presetModified_ = false;
        const TonePreset& p = kPresets[presetIndex_];
        comp_->SetThresholdDb(p.compThreshDb);
        comp_->SetMakeupGainDb(p.compMakeupDb);
        overdrive_->SetBypass(p.odBypass);
        overdrive_->SetDrive(p.odDrive);
        overdrive_->SetBlend(p.odBlend);
        eq_->SetBassDb(p.bassDb);
        eq_->SetMidDb(p.midDb);
        eq_->SetTrebleDb(p.trebleDb);
        cab_->SetCabEnabled(p.cabEnabled);
    }
    void        CyclePreset() noexcept { ApplyPreset(presetIndex_ + 1); }
    int         PresetIndex() const noexcept { return presetIndex_; }
    const char* PresetName() const noexcept { return kPresets[presetIndex_].name; }
    bool        PresetModified() const noexcept { return presetModified_; }

    // ---- Settings Persistence -------------------------------------------------------
    void ImportSettings(const AppSettings& s) noexcept {
        ApplyPreset(s.presetIndex);
        SetTrim(s.trim);
        SetInputSelect(static_cast<InputSelect>(s.inputSelect));
        gate_->SetEnabled(s.gateEnabled);
        cab_->SetCabEnabled(s.cabEnabled);
        overdrive_->SetBypass(s.odBypass);
        overdrive_->SetDrive(s.odDrive);
        overdrive_->SetBlend(s.odBlend);
        eq_->SetBassDb(s.bassDb);
        eq_->SetMidDb(s.midDb);
        eq_->SetTrebleDb(s.trebleDb);
        const TonePreset& p = kPresets[presetIndex_];
        presetModified_ = (s.cabEnabled != p.cabEnabled || s.odBypass != p.odBypass ||
                           std::fabs(s.odDrive - p.odDrive) > 1e-3f ||
                           std::fabs(s.odBlend - p.odBlend) > 1e-3f ||
                           std::fabs(s.bassDb - p.bassDb) > 1e-3f ||
                           std::fabs(s.midDb - p.midDb) > 1e-3f ||
                           std::fabs(s.trebleDb - p.trebleDb) > 1e-3f);
    }

    void ExportSettings(AppSettings& s) const noexcept {
        s.trim        = Trim();
        s.inputSelect = static_cast<int>(Input());
        s.presetIndex = PresetIndex();
        s.gateEnabled = GateEnabled();
        s.cabEnabled  = CabEnabled();
        s.odBypass    = !OverdriveEnabled();
        s.odDrive     = Drive();
        s.odBlend     = Blend();
        s.bassDb      = BassDb();
        s.midDb       = MidDb();
        s.trebleDb    = TrebleDb();
        s.exclusive   = Exclusive();
    }

    // ---- Telemetry (UI thread consumes and resets) ----------------------------------
    float ConsumeInputPeak() noexcept   { return inputPeak_.exchange(0.0f, std::memory_order_relaxed); }
    float ConsumeOutputPeak() noexcept  { return outputPeak_.exchange(0.0f, std::memory_order_relaxed); }
    bool  ConsumeLimitEvent() noexcept  { return limitEvent_.exchange(false, std::memory_order_relaxed); }
    bool  ConsumeAdcClipEvent() noexcept { return adcClipEvent_.exchange(false, std::memory_order_relaxed); }

    const History& DryTap() const noexcept    { return dryBuffer_; }
    const History& WetTap() const noexcept    { return wetBuffer_; }
    const History& LowWetTap() const noexcept { return lowWetBuffer_; }

    // ---- Device info & loss / fallback notifications --------------------------------
    bool        Running() const noexcept   { return running_; }
    bool        Exclusive() const noexcept { return exclusive_; }
    bool        DeviceLost() const noexcept { return deviceLost_.load(std::memory_order_relaxed); }
    float       SampleRate() const noexcept { return sampleRate_; }
    float       LowBandSampleRate() const noexcept {
        return sampleRate_ / static_cast<float>(cfg::kLowBandDecim);
    }
    LatencyInfo Latency() const noexcept      { return latency_; }
    const char* CaptureName() const noexcept  { return captureName_; }
    const char* PlaybackName() const noexcept { return playbackName_; }

    // Poll on UI thread to detect rerouted devices or consume status notices
    bool ConsumeNotice(char* outBuf, size_t outSize, bool& outIsWarning) noexcept {
        if (deviceRerouted_.exchange(false, std::memory_order_relaxed) && running_) {
            ma_device_get_name(&device_, ma_device_type_capture, captureName_,
                               sizeof(captureName_), nullptr);
            ma_device_get_name(&device_, ma_device_type_playback, playbackName_,
                               sizeof(playbackName_), nullptr);
            SetNotice("AUDIO ENDPOINT REROUTED BY WINDOWS", false);
        }
        if (!hasNotice_.exchange(false, std::memory_order_acquire)) return false;
        if (outBuf && outSize > 0) {
            std::snprintf(outBuf, outSize, "%s", noticeText_);
        }
        outIsWarning = noticeWarning_;
        return true;
    }

    // Simulate device-loss notification in offline tests
    void SimulateDeviceLossForTest() noexcept {
        deviceLost_.store(true, std::memory_order_relaxed);
        SetNotice("AUDIO DEVICE LOST - PRESS [E] OR [D] TO RECONNECT", true);
    }

private:
    static constexpr ma_uint32 kScratchFrames = 512;

    static void DataCallback(ma_device* device, void* output, const void* input,
                             ma_uint32 frameCount) {
        static_cast<AudioEngine*>(device->pUserData)
            ->ProcessInterleaved(static_cast<float*>(output),
                                 static_cast<const float*>(input), frameCount);
    }

    static void NotificationCallback(const ma_device_notification* pNotification) {
        if (pNotification == nullptr || pNotification->pDevice == nullptr) return;
        auto* self = static_cast<AudioEngine*>(pNotification->pDevice->pUserData);
        if (self == nullptr) return;

        if (pNotification->type == ma_device_notification_type_stopped) {
            if (!self->stopping_.load(std::memory_order_relaxed)) {
                self->deviceLost_.store(true, std::memory_order_relaxed);
                self->SetNotice("AUDIO DEVICE LOST - PRESS [E] OR [D] TO RECONNECT", true);
            }
        } else if (pNotification->type == ma_device_notification_type_rerouted) {
            self->deviceRerouted_.store(true, std::memory_order_relaxed);
        }
    }

    static void AtomicMax(std::atomic<float>& target, float value) noexcept {
        float current = target.load(std::memory_order_relaxed);
        while (value > current &&
               !target.compare_exchange_weak(current, value, std::memory_order_relaxed)) {
        }
    }

    void SetNotice(const char* text, bool isWarning) noexcept {
        std::snprintf(noticeText_, sizeof(noticeText_), "%s", text ? text : "");
        noticeWarning_ = isWarning;
        hasNotice_.store(true, std::memory_order_release);
    }

    void SetSampleRate(float sampleRate) noexcept {
        sampleRate_ = sampleRate;
        muteCoeff_  = OnePoleCoeff(cfg::kMuteRampMs, sampleRate);
        // Anti-aliasing cutoff at ~1800 Hz before 4x decimation (Nyquist of 12 kHz = 6 kHz)
        lowAaCoeff_ = static_cast<float>(
            1.0 - std::exp(-2.0 * cfg::kPi * 1800.0 / static_cast<double>(sampleRate)));
    }

    void EnumerateEndpoints() noexcept {
        captureEndpoints_.clear();
        playbackEndpoints_.clear();
        if (!contextOk_) return;

        ma_device_info* pPlaybackInfos = nullptr;
        ma_uint32       playbackCount  = 0;
        ma_device_info* pCaptureInfos  = nullptr;
        ma_uint32       captureCount   = 0;
        if (ma_context_get_devices(&context_, &pPlaybackInfos, &playbackCount,
                                   &pCaptureInfos, &captureCount) != MA_SUCCESS) {
            return;
        }
        for (ma_uint32 i = 0; i < captureCount; ++i) {
            EndpointEntry e{};
            e.id = pCaptureInfos[i].id;
            std::snprintf(e.name, sizeof(e.name), "%s", pCaptureInfos[i].name);
            e.isDefault = (pCaptureInfos[i].isDefault != 0);
            captureEndpoints_.push_back(e);
        }
        for (ma_uint32 i = 0; i < playbackCount; ++i) {
            EndpointEntry e{};
            e.id = pPlaybackInfos[i].id;
            std::snprintf(e.name, sizeof(e.name), "%s", pPlaybackInfos[i].name);
            e.isDefault = (pPlaybackInfos[i].isDefault != 0);
            playbackEndpoints_.push_back(e);
        }
    }

    bool TryStart(ma_share_mode shareMode) {
        ma_device_config config         = ma_device_config_init(ma_device_type_duplex);
        config.capture.format           = ma_format_f32;
        config.capture.channels         = cfg::kChannelsIn;
        config.capture.shareMode        = shareMode;
        if (selectedCaptureIdx_ >= 0 &&
            selectedCaptureIdx_ < static_cast<int>(captureEndpoints_.size())) {
            config.capture.pDeviceID = &captureEndpoints_[selectedCaptureIdx_].id;
        }
        config.playback.format          = ma_format_f32;
        config.playback.channels        = cfg::kChannelsOut;
        config.playback.shareMode       = shareMode;
        if (selectedPlaybackIdx_ >= 0 &&
            selectedPlaybackIdx_ < static_cast<int>(playbackEndpoints_.size())) {
            config.playback.pDeviceID = &playbackEndpoints_[selectedPlaybackIdx_].id;
        }
        config.sampleRate               = cfg::kSampleRate;
        config.periodSizeInFrames       = cfg::kPeriodFrames;
        config.periods                  = cfg::kPeriods;
        config.performanceProfile       = ma_performance_profile_low_latency;
        config.wasapi.usage             = ma_wasapi_usage_pro_audio;
        config.wasapi.noAutoConvertSRC  = (shareMode == ma_share_mode_exclusive) ? MA_TRUE : MA_FALSE;
        config.dataCallback             = DataCallback;
        config.notificationCallback     = NotificationCallback;
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
    CompressorNode*                        comp_      = nullptr;
    BassOverdriveNode*                     overdrive_ = nullptr;
    ThreeBandEqNode*                       eq_        = nullptr;
    CabSimNode*                            cab_       = nullptr;

    History dryBuffer_;      // Pre-FX tap  -> tuner (48 kHz)
    History wetBuffer_;      // Post-FX tap -> triggered scope + high-band spectrum (48 kHz)
    History lowWetBuffer_;   // 4x-decimated post-FX tap -> 2.93 Hz low-band spectrum (12 kHz)

    // Audio-thread-only state
    std::array<float, kScratchFrames> dryScratch_{};
    std::array<float, kScratchFrames> wetScratch_{};
    std::array<float, kScratchFrames> lowWetScratch_{};
    float  sampleRate_    = static_cast<float>(cfg::kSampleRate);
    float  muteCoeff_     = 1.0f;
    float  muteGain_      = 0.0f;
    float  lowAaCoeff_    = 0.2f;
    float  lowAa1_        = 0.0f;
    float  lowAa2_        = 0.0f;
    size_t lowDecimPhase_ = 0;

    // Cross-thread controls & telemetry
    std::atomic<int>      inputSelect_{static_cast<int>(InputSelect::Sum)};
    std::atomic<float>    trim_{1.0f};
    std::atomic<bool>     muted_{true};   // start muted: protects against mic/speaker feedback
    std::atomic<float>    inputPeak_{0.0f};
    std::atomic<float>    outputPeak_{0.0f};
    std::atomic<bool>     limitEvent_{false};
    std::atomic<bool>     adcClipEvent_{false};
    std::atomic<uint64_t> callbackCount_{0};
    int                   presetIndex_    = 0;
    bool                  presetModified_ = false;

    // Device & endpoint state
    ma_context                 context_{};
    ma_device                  device_{};
    bool                       contextOk_           = false;
    bool                       running_             = false;
    bool                       exclusive_           = false;
    std::atomic<bool>          stopping_{false};
    std::atomic<bool>          deviceLost_{false};
    std::atomic<bool>          deviceRerouted_{false};
    std::vector<EndpointEntry> captureEndpoints_;
    std::vector<EndpointEntry> playbackEndpoints_;
    int                        selectedCaptureIdx_  = -1;
    int                        selectedPlaybackIdx_ = -1;
    LatencyInfo                latency_{};
    char                       captureName_[MA_MAX_DEVICE_NAME_LENGTH + 1]  = "none";
    char                       playbackName_[MA_MAX_DEVICE_NAME_LENGTH + 1] = "none";

    // Status notice banner (mode fallback / device change)
    std::atomic<bool> hasNotice_{false};
    bool              noticeWarning_ = false;
    char              noticeText_[160]{};
};
