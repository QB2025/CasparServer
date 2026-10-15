#include "aja_producer.h"
#include <common/diagnostics/cadence.h>

#include <common/except.h>
#include <common/log.h>
#include <common/param.h>

#include <core/frame/draw_frame.h>
#include <core/frame/frame.h>
#include <core/frame/frame_factory.h>
#include <core/frame/pixel_format.h>
#include <core/monitor/monitor.h>
#include <core/producer/frame_producer.h>
#include <core/video_format.h>

#include <ajantv2/includes/ntv2card.h>
#include <ajantv2/includes/ntv2devicescanner.h>
#include <ajantv2/includes/ntv2enums.h>
#include <ajantv2/includes/ntv2utils.h>

#include <boost/algorithm/string/predicate.hpp>

extern "C" {
#include <libavutil/channel_layout.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <map>
#include <memory>
#include <mutex>

#include <sstream>

#include <string>
#include <tbb/parallel_for.h>
#include <tbb/parallel_invoke.h>
#include <thread>
#include <utility>
#include <vector>

namespace caspar { namespace aja {
namespace {

constexpr NTV2FrameBufferFormat kPixelFormat         = NTV2_FBF_8BIT_YCBCR;
constexpr ULWord                kAutoCirculateFrames = 7;
constexpr ULWord                kCasparAudioChannels = 16;

// CasparCG constructs a replacement producer before destroying the producer
// currently on the layer.  AJA AutoCirculate, however, is a device/channel
// resource rather than an object-local resource.  Track the most recent
// producer that owns each physical capture channel so an obsolete producer
// cannot stop or disable hardware already taken over by its replacement.
using capture_owner_key = std::pair<ULWord, int>;

std::mutex                                 g_capture_owner_mutex;
std::map<capture_owner_key, std::uint64_t> g_capture_owners;
std::atomic<std::uint64_t>                 g_next_capture_owner_token{1};

NTV2VideoFormat get_aja_video_format(core::video_format format)
{
    switch (format) {
        case core::video_format::x1080i5000:
            return NTV2_FORMAT_1080i_5000;
        case core::video_format::x1080i5994:
            return NTV2_FORMAT_1080i_5994;
        case core::video_format::x1080i6000:
            return NTV2_FORMAT_1080i_6000;
        case core::video_format::x1080p2398:
            return NTV2_FORMAT_1080p_2398;
        case core::video_format::x1080p2400:
            return NTV2_FORMAT_1080p_2400;
        case core::video_format::x1080p2500:
            return NTV2_FORMAT_1080p_2500;
        case core::video_format::x1080p2997:
            return NTV2_FORMAT_1080p_2997;
        case core::video_format::x1080p3000:
            return NTV2_FORMAT_1080p_3000;
        case core::video_format::x1080p5000:
            return NTV2_FORMAT_1080p_5000_A;
        case core::video_format::x1080p5994:
            return NTV2_FORMAT_1080p_5994_A;
        case core::video_format::x1080p6000:
            return NTV2_FORMAT_1080p_6000_A;
        default:
            return NTV2_FORMAT_UNKNOWN;
    }
}

bool aja_format_matches_caspar_format(NTV2VideoFormat detected, core::video_format requested)
{
    switch (requested) {
        case core::video_format::x1080p5000:
            return detected == NTV2_FORMAT_1080p_5000_A || detected == NTV2_FORMAT_1080p_5000_B;
        case core::video_format::x1080p5994:
            return detected == NTV2_FORMAT_1080p_5994_A || detected == NTV2_FORMAT_1080p_5994_B;
        case core::video_format::x1080p6000:
            return detected == NTV2_FORMAT_1080p_6000_A || detected == NTV2_FORMAT_1080p_6000_B;
        default:
            return detected == get_aja_video_format(requested);
    }
}

class aja_producer final : public core::frame_producer
{
    const spl::shared_ptr<core::frame_factory> frame_factory_;
    const core::video_format_desc              channel_format_desc_;
    const core::video_format_repository        format_repository_;
    core::video_format_desc                    input_format_desc_;

    ULWord      device_index_ = 0;
    NTV2Channel channel_      = NTV2_CHANNEL1;

    CNTV2Card       device_;
    NTV2InputSource input_source_       = NTV2_INPUTSOURCE_INVALID;
    NTV2VideoFormat input_format_       = NTV2_FORMAT_UNKNOWN;
    NTV2AudioSystem audio_system_       = NTV2_AUDIOSYSTEM_1;
    ULWord          aja_audio_channels_ = 0;

    std::size_t width_  = 0;
    std::size_t height_ = 0;

    std::vector<std::uint8_t> capture_buffer_;
    std::vector<std::uint8_t> capture_audio_buffer_;

    struct captured_frame
    {
        std::vector<std::uint8_t> uyvy;
        std::vector<std::int32_t> audio;
        std::uint64_t             generation = 0;
        std::uint64_t             sequence   = 0;
    };

    struct raw_captured_frame
    {
        std::vector<std::uint8_t> uyvy;
        std::vector<std::int32_t> audio;
        std::uint64_t             generation = 0;
        std::uint64_t             sequence   = 0;
    };

    static constexpr std::size_t kRawQueueCapacity             = 4;
    static constexpr std::size_t kCaptureQueueCapacity         = 4;
    static constexpr std::size_t kCaptureQueueStartupPrebuffer = 3;

    mutable std::mutex             raw_mutex_;
    std::deque<raw_captured_frame> raw_queue_;

    mutable std::mutex         frame_mutex_;
    std::deque<captured_frame> capture_queue_;
    bool                       capture_queue_primed_       = false;
    std::size_t                capture_queue_prime_target_ = kCaptureQueueStartupPrebuffer;

    mutable std::mutex exception_mutex_;
    std::exception_ptr capture_exception_;

    std::atomic<bool>          stop_requested_{false};
    std::atomic<std::uint64_t> capture_generation_{0};
    std::atomic<std::uint64_t> capture_sequence_{0};
    std::thread                capture_thread_;
    std::thread                packaging_thread_;

    bool auto_circulate_started_ = false;

    const std::uint64_t capture_owner_token_ = g_next_capture_owner_token.fetch_add(1, std::memory_order_relaxed);
    std::uint64_t       previous_capture_owner_token_ = 0;
    bool                capture_ownership_claimed_    = false;

    std::chrono::steady_clock::time_point last_no_frame_signal_check_ = std::chrono::steady_clock::now();

    static constexpr unsigned kCleanFramesAfterReacquire = 3;
    bool                      signal_was_good_           = true;
    unsigned                  clean_reacquire_frames_    = kCleanFramesAfterReacquire;

    core::draw_frame latched_frame_a_;

    // Shared progressive/interlaced clock-domain adapter. AJA capture and the Caspar channel
    // run from independent clocks even when both nominally use the same
    // frame rate. Keep video selection and audio delivery independent:
    // video may repeat/skip, while audio is consumed as a continuous 48 kHz
    // stream according to Caspar's requested nb_samples cadence.
    std::vector<std::uint8_t> clock_adapter_video_uyvy_;
    std::vector<std::int32_t> clock_adapter_audio_;
    std::size_t               clock_adapter_audio_offset_ = 0;

    // Shared audio clock-domain servo.  The SDI input and Caspar/output
    // clocks are independent, so maintain an operating region rather than only
    // priming once and then allowing the reservoir to wander toward zero.
    static constexpr std::size_t kAudioTargetFrames   = 3200;
    static constexpr std::size_t kAudioDeadbandFrames = 200;
    static constexpr std::size_t kAudioLowFrames =
        kAudioTargetFrames - kAudioDeadbandFrames;
    static constexpr std::size_t kAudioHighFrames =
        kAudioTargetFrames + kAudioDeadbandFrames;

    bool clock_adapter_audio_started_ = false;

    // Keep the external captured-audio reservoir as the shock absorber.  SWR is only a slow fractional actuator.  A filtered view of
    // the EXTERNAL reservoir estimates long-term clock drift; callback-scale
    // jitter must not steer the resampler.
    SwrContext*                  clock_adapter_audio_swr_                = nullptr;
    static constexpr int         kAudioRate                 = 48000;
    static constexpr int         kAudioCompensationDistance = 48000; // 1 second
    static constexpr int         kAudioMaxCorrectionPpm     = 1000;
    static constexpr double      kAudioFilterAlpha          = 0.03; // ~0.55 s at 60 callbacks/s
    static constexpr std::size_t kAudioSWRPrimeFrames       = 64;
    bool                         clock_adapter_audio_filter_initialized_ = false;
    double                       clock_adapter_audio_filtered_depth_ = static_cast<double>(kAudioTargetFrames);
    bool                         clock_adapter_audio_swr_primed_     = false;
    double                       clock_adapter_audio_input_phase_    = 0.0;
    int                          clock_adapter_audio_correction_ppm_ = 0;
    int                          clock_adapter_audio_compensation_delta_ = 0;

    std::uint64_t    clock_adapter_generation_ = 0;
    std::uint64_t    clock_adapter_sequence_   = 0;
    std::atomic_bool clock_adapter_reset_requested_{false};

    // Diagnostics only. Cross-thread counters use relaxed atomics; none feed
    // queue selection, audio budgets, compensation or recovery decisions.
    std::atomic<std::uint64_t>            diag_captured_{0};
    std::atomic<std::uint64_t>            diag_raw_dropped_{0}, diag_raw_audio_dropped_{0};
    std::atomic<std::uint64_t>            diag_packaged_dropped_{0}, diag_packaged_audio_dropped_{0};
    std::atomic<std::uint64_t>            diag_swr_input_{0}, diag_swr_output_{0};
    std::atomic<std::uint64_t>            diag_underflows_{0}, diag_missing_output_{0};
    std::mutex                            diag_reservoir_mutex_;
    std::size_t                           diag_reservoir_min_ = static_cast<std::size_t>(-1);
    std::size_t                           diag_reservoir_max_ = 0, diag_reservoir_observations_ = 0;
    std::chrono::steady_clock::time_point diag_last_report_ = std::chrono::steady_clock::now();

    struct callback_diagnostic_counts
    {
        std::uint64_t                calls = 0, positive_samples = 0, zero_calls = 0, negative_calls = 0;
        int                          request_min = 0, request_max = 0;
        std::map<int, std::uint64_t> requests;
        std::map<std::wstring, std::pair<std::uint64_t, std::uint64_t>> returns;
        std::uint64_t gaps = 0, gap_sum_us = 0, gap_min_us = 0, gap_max_us = 0;
    };
    std::mutex                            diag_callback_mutex_;
    callback_diagnostic_counts            diag_impl_callbacks_, diag_clock_adapter_callbacks_;
    std::chrono::steady_clock::time_point diag_impl_previous_{}, diag_clock_adapter_previous_{};

    core::monitor::state state_;

  public:
    aja_producer(const spl::shared_ptr<core::frame_factory>& frame_factory,
                 const core::video_format_desc&              channel_format_desc,
                 const core::video_format_repository&        format_repository,
                 ULWord                                      device_index,
                 NTV2Channel                                 channel,
                 const std::wstring&                         format)
        : frame_factory_(frame_factory)
        , channel_format_desc_(channel_format_desc)
        , format_repository_(format_repository)
        , device_index_(device_index)
        , channel_(channel)
    {
        if (format.empty())
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA FORMAT parameter is required"));

        input_format_desc_ = format_repository_.find(format);

        if (input_format_desc_.format == core::video_format::invalid)
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Unknown AJA input FORMAT"));

        initialize();

        // Claim the physical capture channel before the replacement producer
        // becomes visible to CasparCG.  Once claimed, teardown from the old
        // producer is no longer allowed to stop/disable this channel.
        previous_capture_owner_token_ = claim_capture_ownership();

        try {
            packaging_thread_ = std::thread([this] { packaging_loop(); });
            capture_thread_   = std::thread([this] { capture_loop(); });
        } catch (...) {
            stop_requested_ = true;
            if (capture_thread_.joinable())
                capture_thread_.join();
            if (packaging_thread_.joinable())
                packaging_thread_.join();
            rollback_capture_ownership();
            throw;
        }
    }

    ~aja_producer() override
    {
        stop_requested_ = true;

        // Wake/stop our capture only while we still own the physical channel.
        // A replacement producer may already have started AutoCirculate here.
        stop_autocirculate_if_owner();

        if (capture_thread_.joinable())
            capture_thread_.join();
        if (packaging_thread_.joinable())
            packaging_thread_.join();

        // CLEAR/shutdown: we are still owner, so stop and disable normally.
        // Replacement: ownership has moved, so leave the new producer alone.
        release_capture_hardware_if_owner();

        if (clock_adapter_audio_swr_)
            swr_free(&clock_adapter_audio_swr_);

        CASPAR_LOG(info) << print() << L" Uninitialized";
    }

    caspar::diagnostics::cadence_metric aja_receive_{L"aja_receive"};
    caspar::diagnostics::cadence_metric aja_make_frame_{L"aja_make_frame"};
    core::draw_frame                    receive_impl(const core::video_field field, int nb_samples) override
    {
        diagnostic_callback_entry(false, nb_samples);

        rethrow_capture_exception();

        // Both progressive and interlaced inputs cross an asynchronous clock boundary here.
        // Do not require one physical AJA frame for every Caspar callback.
        // Drain whatever capture has arrived, select the newest video frame,
        // and consume exactly nb_samples from a continuous audio reservoir.
        const bool measure = nb_samples > 0 && clock_adapter_audio_started_ && !clock_adapter_reset_requested_.load();
        caspar::diagnostics::cadence_scope aja_receive_scope(aja_receive_, measure, nb_samples, [&] {
            return print() + L" channel_format=" + channel_format_desc_.name + L" nb_samples=" +
                   std::to_wstring(nb_samples) + L" reservoir_sample_frames=" +
                   std::to_wstring((clock_adapter_audio_.size() - clock_adapter_audio_offset_) / kCasparAudioChannels);
        });
        return receive_clock_adapted(nb_samples);
    }

    core::draw_frame first_frame(const core::video_field field) override { return receive_impl(field, 0); }

    core::draw_frame last_frame(const core::video_field field) override
    {
        return latched_frame_a_ ? core::draw_frame::still(latched_frame_a_) : core::draw_frame::empty();
    }

    bool is_ready() override
    {
        rethrow_capture_exception();

        std::lock_guard<std::mutex> lock(frame_mutex_);
        return (capture_queue_primed_ && !capture_queue_.empty()) || static_cast<bool>(latched_frame_a_);
    }

    std::wstring print() const override
    {
        return L"AJA input [" + std::to_wstring(device_index_ + 1) + L"|" +
               std::to_wstring(static_cast<int>(channel_) + 1) + L"|" + input_format_desc_.name + L"]";
    }

    std::wstring         name() const override { return L"aja"; }
    core::monitor::state state() const override { return state_; }

  private:

    void diagnostic_callback_entry(bool adapter, int samples)
    {
        if (!caspar::diagnostics::cadence_enabled())
            return;

        std::lock_guard<std::mutex> lock(diag_callback_mutex_);
        const auto                  now      = std::chrono::steady_clock::now();
        auto&                       counts   = adapter ? diag_clock_adapter_callbacks_ : diag_impl_callbacks_;
        auto&                       previous = adapter ? diag_clock_adapter_previous_ : diag_impl_previous_;
        if (counts.calls == 0)
            counts.request_min = counts.request_max = samples;
        counts.request_min = std::min(counts.request_min, samples);
        counts.request_max = std::max(counts.request_max, samples);
        ++counts.calls;
        ++counts.requests[samples];
        if (samples > 0)
            counts.positive_samples += static_cast<std::uint64_t>(samples);
        else if (samples == 0)
            ++counts.zero_calls;
        else
            ++counts.negative_calls;
        if (previous != std::chrono::steady_clock::time_point{}) {
            const auto gap = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now - previous).count());
            if (counts.gaps == 0)
                counts.gap_min_us = counts.gap_max_us = gap;
            counts.gap_min_us = std::min(counts.gap_min_us, gap);
            counts.gap_max_us = std::max(counts.gap_max_us, gap);
            ++counts.gaps;
            counts.gap_sum_us += gap;
        }
        previous = now;
    }

    void diagnostic_callback_return(bool adapter, const wchar_t* path, int samples)
    {
        if (!caspar::diagnostics::cadence_enabled())
            return;

        std::lock_guard<std::mutex> lock(diag_callback_mutex_);
        auto                        record = [path, samples](callback_diagnostic_counts& counts) {
            auto& value = counts.returns[path];
            ++value.first;
            if (samples > 0)
                value.second += static_cast<std::uint64_t>(samples);
        };
        record(diag_impl_callbacks_);
        // receive_clock_adapted is called exclusively by receive_impl; its return
        // is also the enclosing receive_impl return. Do not count entries twice.
        if (adapter)
            record(diag_clock_adapter_callbacks_);
    }

    void report_callback_diagnostics(std::int64_t elapsed_ms)
    {
        if (!caspar::diagnostics::cadence_enabled())
            return;

        callback_diagnostic_counts impl, adapter;
        {
            std::lock_guard<std::mutex> lock(diag_callback_mutex_);
            std::swap(impl, diag_impl_callbacks_);
            std::swap(adapter, diag_clock_adapter_callbacks_);
        }
        auto report = [this, elapsed_ms](const wchar_t* stage, const callback_diagnostic_counts& counts) {
            std::wostringstream text;
            text << L" callback-diag stage=" << stage << L" interval-ms=" << elapsed_ms << L" calls=" << counts.calls
                 << L" calls-per-sec=" << (1000.0 * static_cast<double>(counts.calls) / static_cast<double>(elapsed_ms))
                 << L" requested-positive-sample-frames=" << counts.positive_samples << L" zero-calls="
                 << counts.zero_calls << L" negative-calls=" << counts.negative_calls << L" request-min="
                 << counts.request_min << L" request-max=" << counts.request_max << L" gaps=" << counts.gaps
                 << L" gap-min-us=" << counts.gap_min_us << L" gap-max-us=" << counts.gap_max_us << L" gap-mean-us="
                 << (counts.gaps ? static_cast<double>(counts.gap_sum_us) / static_cast<double>(counts.gaps) : 0.0)
                 << L" request-hist={";
            for (const auto& request : counts.requests)
                text << request.first << L":" << request.second << L",";
            text << L"} returns={";
            for (const auto& path : counts.returns)
                text << path.first << L":" << path.second.first << L"/" << path.second.second << L",";
            text << L"}";
            CASPAR_LOG(info) << print() << text.str();
        };
        report(L"receive_impl", impl);
        report(L"receive_clock_adapted", adapter);
    }

    capture_owner_key capture_owner_key_value() const { return {device_index_, static_cast<int>(channel_)}; }

    std::uint64_t claim_capture_ownership()
    {
        std::lock_guard<std::mutex> lock(g_capture_owner_mutex);
        const auto                  key      = capture_owner_key_value();
        const auto                  it       = g_capture_owners.find(key);
        const std::uint64_t         previous = it == g_capture_owners.end() ? 0 : it->second;
        g_capture_owners[key]                = capture_owner_token_;
        capture_ownership_claimed_           = true;
        return previous;
    }

    void rollback_capture_ownership() noexcept
    {
        if (!capture_ownership_claimed_)
            return;

        std::lock_guard<std::mutex> lock(g_capture_owner_mutex);
        const auto                  key = capture_owner_key_value();
        const auto                  it  = g_capture_owners.find(key);
        if (it != g_capture_owners.end() && it->second == capture_owner_token_) {
            if (previous_capture_owner_token_ != 0)
                it->second = previous_capture_owner_token_;
            else
                g_capture_owners.erase(it);
        }
        capture_ownership_claimed_ = false;
    }

    void stop_autocirculate_if_owner() noexcept
    {
        if (!capture_ownership_claimed_)
            return;

        std::lock_guard<std::mutex> lock(g_capture_owner_mutex);
        const auto                  it = g_capture_owners.find(capture_owner_key_value());
        if (it == g_capture_owners.end() || it->second != capture_owner_token_)
            return;

        try {
            if (device_.IsOpen())
                device_.AutoCirculateStop(channel_);
        } catch (...) {
        }
    }

    void release_capture_hardware_if_owner() noexcept
    {
        if (!capture_ownership_claimed_)
            return;

        std::lock_guard<std::mutex> lock(g_capture_owner_mutex);
        const auto                  key = capture_owner_key_value();
        const auto                  it  = g_capture_owners.find(key);
        if (it == g_capture_owners.end() || it->second != capture_owner_token_) {
            capture_ownership_claimed_ = false;
            return;
        }

        try {
            if (device_.IsOpen()) {
                device_.AutoCirculateStop(channel_);
                device_.DisableChannel(channel_);
            }
        } catch (...) {
        }

        g_capture_owners.erase(it);
        capture_ownership_claimed_ = false;
    }

    void initialize()
    {
        const NTV2VideoFormat expected_format = get_aja_video_format(input_format_desc_.format);
        if (expected_format == NTV2_FORMAT_UNKNOWN) {
            CASPAR_THROW_EXCEPTION(user_error()
                                   << msg_info("AJA input producer currently supports 1080 HD formats only"));
        }

        CNTV2DeviceScanner scanner(true);

        if (!CNTV2DeviceScanner::GetDeviceAtIndex(device_index_, device_))
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Unable to open selected AJA input device"));

        if (!device_.features().CanDoCapture())
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Selected AJA device does not support capture"));

        if (!device_.features().CanDoChannel(channel_))
            CASPAR_THROW_EXCEPTION(user_error()
                                   << msg_info("Selected AJA device does not support requested input channel"));

        input_source_ = ::NTV2ChannelToInputSource(channel_, NTV2_IOKINDS_SDI);

        if (!device_.features().CanDoInputSource(input_source_))
            CASPAR_THROW_EXCEPTION(user_error()
                                   << msg_info("Selected AJA device does not provide requested SDI input"));

        device_.SetEveryFrameServices(NTV2_OEM_TASKS);

        if (!device_.EnableChannel(channel_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to enable selected AJA input channel"));

        device_.SetMode(channel_, NTV2_MODE_CAPTURE);

        if (device_.features().HasBiDirectionalSDI()) {
            device_.SetSDITransmitEnable(channel_, false);
            for (int i = 0; i < 10; ++i)
                device_.WaitForInputVerticalInterrupt(channel_);
        }

        bool is_3gb = false;
        device_.GetSDIInput3GbPresent(is_3gb, channel_);

        input_format_ = device_.GetInputVideoFormat(input_source_);

        if (input_format_ == NTV2_FORMAT_UNKNOWN)
            CASPAR_THROW_EXCEPTION(user_error()
                                   << msg_info("No signal or unknown video format on selected AJA SDI input"));

        if (!aja_format_matches_caspar_format(input_format_, input_format_desc_.format))
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA SDI input format does not match requested FORMAT"));

        if (!device_.features().CanDoVideoFormat(input_format_))
            CASPAR_THROW_EXCEPTION(user_error()
                                   << msg_info("Selected AJA device cannot capture detected video format"));

        NTV2VideoFormat capture_format = input_format_;
        bool            level_b_to_a   = false;

        switch (input_format_) {
            case NTV2_FORMAT_1080p_5000_B:
                capture_format = NTV2_FORMAT_1080p_5000_A;
                level_b_to_a   = true;
                break;
            case NTV2_FORMAT_1080p_5994_B:
                capture_format = NTV2_FORMAT_1080p_5994_A;
                level_b_to_a   = true;
                break;
            case NTV2_FORMAT_1080p_6000_B:
                capture_format = NTV2_FORMAT_1080p_6000_A;
                level_b_to_a   = true;
                break;
            default:
                break;
        }

        if (!device_.features().CanDoVideoFormat(capture_format))
            CASPAR_THROW_EXCEPTION(user_error()
                                   << msg_info("Selected AJA device cannot capture normalized video format"));

        if (caspar::diagnostics::cadence_enabled()) {
            const bool supports_multiformat = device_.features().CanDoMultiFormat();

            bool       multiformat_enabled = false;
            const bool mode_read_ok        = device_.GetMultiFormatMode(multiformat_enabled);

            NTV2VideoFormat channel1_format = NTV2_FORMAT_UNKNOWN;
            NTV2VideoFormat channel3_format = NTV2_FORMAT_UNKNOWN;

            const bool ch1_ok = device_.GetVideoFormat(channel1_format, NTV2_CHANNEL1);
            const bool ch3_ok = device_.GetVideoFormat(channel3_format, NTV2_CHANNEL3);

            CASPAR_LOG(info) << L"AJA format diagnostic (before-input-format): capable=" << supports_multiformat << L" mode_read_ok="
                             << mode_read_ok << L" enabled=" << multiformat_enabled << L" ch1_read_ok=" << ch1_ok
                             << L" ch1_format=" << static_cast<int>(channel1_format) << L" ch3_read_ok=" << ch3_ok
                             << L" ch3_format=" << static_cast<int>(channel3_format);
        }

        if (!device_.SetVideoFormat(capture_format, false, false, channel_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to set AJA input video format"));

        if (caspar::diagnostics::cadence_enabled()) {
            const bool supports_multiformat = device_.features().CanDoMultiFormat();
            bool multiformat_enabled = false;
            const bool mode_read_ok_after = device_.GetMultiFormatMode(multiformat_enabled);

            NTV2VideoFormat channel1_format = NTV2_FORMAT_UNKNOWN;
            NTV2VideoFormat channel3_format = NTV2_FORMAT_UNKNOWN;

            const bool ch1_ok_after = device_.GetVideoFormat(channel1_format, NTV2_CHANNEL1);
            const bool ch3_ok_after = device_.GetVideoFormat(channel3_format, NTV2_CHANNEL3);

            CASPAR_LOG(info) << L"AJA format diagnostic (after-input-format): capable=" << supports_multiformat
                             << L" mode_read_ok=" << mode_read_ok_after << L" enabled=" << multiformat_enabled
                             << L" ch1_read_ok=" << ch1_ok_after
                             << L" ch1_format=" << static_cast<int>(channel1_format)
                             << L" ch3_read_ok=" << ch3_ok_after
                             << L" ch3_format=" << static_cast<int>(channel3_format);
        }

        device_.SetSDIInLevelBtoLevelAConversion(channel_, level_b_to_a);

        if (caspar::diagnostics::cadence_enabled())
            CASPAR_LOG(info) << print() << L" SDI 3G input detected: " << (is_3gb ? L"yes" : L"no")
                         << L"; Level B to Level A conversion: " << (level_b_to_a ? L"enabled" : L"disabled");

        device_.SetVANCMode(NTV2_VANCMODE_OFF, channel_);
        device_.SetVANCShiftMode(channel_, NTV2_VANCDATA_NORMAL);

        if (!device_.SetFrameBufferFormat(channel_, kPixelFormat))
            CASPAR_THROW_EXCEPTION(caspar_exception()
                                   << msg_info("Unable to set AJA input framebuffer to 8-bit YCbCr"));

        audio_system_ = NTV2_AUDIOSYSTEM_1;
        if (device_.features().GetNumAudioSystems() > 1)
            audio_system_ = ::NTV2ChannelToAudioSystem(channel_);

        if (!device_.SetAudioSystemInputSource(
                audio_system_, NTV2_AUDIO_EMBEDDED, ::NTV2InputSourceToEmbeddedAudioInput(input_source_)))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to select AJA embedded audio input"));

        aja_audio_channels_ = static_cast<ULWord>(device_.features().GetMaxAudioChannels());
        if (aja_audio_channels_ == 0 || aja_audio_channels_ > kCasparAudioChannels)
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unsupported AJA embedded audio channel count"));

        if (!device_.SetNumberAudioChannels(aja_audio_channels_, audio_system_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to configure AJA embedded audio input"));

        ULWord configured_audio_channels = 0;
        if (!device_.GetNumberAudioChannels(configured_audio_channels, audio_system_) ||
            configured_audio_channels != aja_audio_channels_)
            CASPAR_THROW_EXCEPTION(caspar_exception()
                                   << msg_info("AJA embedded audio channel configuration did not take effect"));

        if (!device_.SetAudioRate(NTV2_AUDIO_48K, audio_system_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to configure AJA audio rate"));

        if (!device_.SetAudioBufferSize(NTV2_AUDIO_BUFFER_SIZE_4MB, audio_system_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to configure AJA audio buffer"));

        if (!device_.SetAudioLoopBack(NTV2_AUDIO_LOOPBACK_OFF, audio_system_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to disable AJA audio loopback"));

        AVChannelLayout swr_layout = AV_CHANNEL_LAYOUT_HEXADECAGONAL;
        if (swr_alloc_set_opts2(&clock_adapter_audio_swr_,
                                &swr_layout,
                                AV_SAMPLE_FMT_S32,
                                kAudioRate,
                                &swr_layout,
                                AV_SAMPLE_FMT_S32,
                                kAudioRate,
                                0,
                                nullptr) < 0 ||
            !clock_adapter_audio_swr_ || swr_init(clock_adapter_audio_swr_) < 0) {
            if (clock_adapter_audio_swr_)
                swr_free(&clock_adapter_audio_swr_);
            CASPAR_THROW_EXCEPTION(caspar_exception()
                                   << msg_info("Unable to initialize AJA clock-adapted audio clock resampler"));
        }

        const NTV2OutputXptID input_xpt       = ::GetInputSourceOutputXpt(input_source_);
        const NTV2InputXptID  frame_store_xpt = ::GetFrameStoreInputXptFromChannel(channel_);

        NTV2XptConnections connections;
        connections.insert(NTV2XptConnection(frame_store_xpt, input_xpt));

        if (!device_.ApplySignalRoute(connections, false))
            CASPAR_THROW_EXCEPTION(caspar_exception()
                                   << msg_info("Unable to route selected AJA SDI input to FrameStore"));

        const NTV2FormatDescriptor format_desc(capture_format, kPixelFormat);
        width_  = static_cast<std::size_t>(format_desc.GetRasterWidth());
        height_ = static_cast<std::size_t>(format_desc.GetRasterHeight());

        if (width_ != static_cast<std::size_t>(channel_format_desc_.width) ||
            height_ != static_cast<std::size_t>(channel_format_desc_.height))
            CASPAR_THROW_EXCEPTION(caspar_exception()
                                   << msg_info("AJA input raster size does not match the CasparCG channel"));

        capture_buffer_.resize(static_cast<std::size_t>(format_desc.GetVideoWriteSize()));
        capture_audio_buffer_.resize(256 * 1024);

        state_["device"]                  = static_cast<int64_t>(device_index_ + 1);
        state_["channel"]                 = static_cast<int64_t>(static_cast<int>(channel_) + 1);
        state_["format"]                  = input_format_desc_.name;
        state_["audio/sample-rate"]       = static_cast<int64_t>(48000);
        state_["audio/channels"]          = static_cast<int64_t>(kCasparAudioChannels);
        state_["audio/hardware-channels"] = static_cast<int64_t>(aja_audio_channels_);

        CASPAR_LOG(info) << L"AJA producer initialized: device " << (device_index_ + 1) << L", input channel "
                         << (static_cast<int>(channel_) + 1) << L", detected " << input_format_desc_.name << L", "
                         << aja_audio_channels_ << L"-channel 48 kHz embedded audio" << L" -> " << kCasparAudioChannels
                         << L"-channel Caspar layout";
    }

    void monitor_signal_without_frame()
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_no_frame_signal_check_ < std::chrono::milliseconds(100))
            return;

        last_no_frame_signal_check_ = now;

        // AutoCirculate can legitimately have no transferable frame when SDI
        // disappears. Probe receiver state independently so signal-loss
        // detection does not depend on a successful transfer.
        (void)input_frame_is_stable();
    }

    void observe_diagnostic_reservoir(std::size_t frames)
    {
        if (!caspar::diagnostics::cadence_enabled())
            return;

        std::lock_guard<std::mutex> lock(diag_reservoir_mutex_);
        diag_reservoir_min_ = std::min(diag_reservoir_min_, frames);
        diag_reservoir_max_ = std::max(diag_reservoir_max_, frames);
        ++diag_reservoir_observations_;
    }

    // Called only by the capture thread, including while no signal is present.
    void report_audio_diagnostics()
    {
        if (!caspar::diagnostics::cadence_enabled())
            return;

        const auto now = std::chrono::steady_clock::now();
        if (now - diag_last_report_ < std::chrono::seconds(1))
            return;
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - diag_last_report_).count();
        diag_last_report_     = now;
        std::size_t minimum, maximum, observations;
        {
            std::lock_guard<std::mutex> lock(diag_reservoir_mutex_);
            minimum                      = diag_reservoir_min_;
            maximum                      = diag_reservoir_max_;
            observations                 = diag_reservoir_observations_;
            diag_reservoir_min_          = static_cast<std::size_t>(-1);
            diag_reservoir_max_          = 0;
            diag_reservoir_observations_ = 0;
        }

        report_callback_diagnostics(elapsed_ms);

        CASPAR_LOG(info) << print() << L" audio-diag interval-ms=" << elapsed_ms << L" captured="
                         << diag_captured_.exchange(0, std::memory_order_relaxed) << L" raw-dropped="
                         << diag_raw_dropped_.exchange(0, std::memory_order_relaxed) << L" raw-audio-dropped="
                         << diag_raw_audio_dropped_.exchange(0, std::memory_order_relaxed) << L" packaged-dropped="
                         << diag_packaged_dropped_.exchange(0, std::memory_order_relaxed) << L" packaged-audio-dropped="
                         << diag_packaged_audio_dropped_.exchange(0, std::memory_order_relaxed) << L" swr-input="
                         << diag_swr_input_.exchange(0, std::memory_order_relaxed) << L" swr-output="
                         << diag_swr_output_.exchange(0, std::memory_order_relaxed) << L" reservoir-min="
                         << (minimum == static_cast<std::size_t>(-1) ? 0 : minimum) << L" reservoir-max=" << maximum
                         << L" reservoir-observations=" << observations << L" underflows="
                         << diag_underflows_.exchange(0, std::memory_order_relaxed) << L" missing-output="
                         << diag_missing_output_.exchange(0, std::memory_order_relaxed);
    }

    void capture_loop()
    {
        try {
            device_.AutoCirculateStop(channel_);

            if (!device_.AutoCirculateInitForInput(
                    channel_, kAutoCirculateFrames, audio_system_, AUTOCIRCULATE_WITH_RP188))
                CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to initialize AJA input AutoCirculate"));

            if (!device_.AutoCirculateStart(channel_))
                CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to start AJA input AutoCirculate"));

            auto_circulate_started_ = true;
            AUTOCIRCULATE_TRANSFER transfer;

            while (!stop_requested_) {
                report_audio_diagnostics();
                AUTOCIRCULATE_STATUS status;
                device_.AutoCirculateGetStatus(channel_, status);

                if (status.IsRunning() && status.HasAvailableInputFrame()) {
                    transfer.SetVideoBuffer(reinterpret_cast<ULWord*>(capture_buffer_.data()),
                                            static_cast<ULWord>(capture_buffer_.size()));
                    transfer.SetAudioBuffer(reinterpret_cast<ULWord*>(capture_audio_buffer_.data()),
                                            static_cast<ULWord>(capture_audio_buffer_.size()));

                    if (!device_.AutoCirculateTransfer(channel_, transfer)) {
                        CASPAR_LOG(warning) << print() << L" AutoCirculate input transfer failed";
                        continue;
                    }

                    if (caspar::diagnostics::cadence_enabled()) {
                        const auto audio_bytes = transfer.GetCapturedAudioByteCount();
                        const auto frame_bytes = aja_audio_channels_ * sizeof(std::int32_t);
                        if (frame_bytes != 0 && audio_bytes <= capture_audio_buffer_.size() &&
                            audio_bytes % frame_bytes == 0)
                            diag_captured_.fetch_add(audio_bytes / frame_bytes, std::memory_order_relaxed);
                    }

                    if (!input_frame_is_stable())
                        continue;

                    const ULWord audio_bytes                     = transfer.GetCapturedAudioByteCount();
                    const ULWord hardware_bytes_per_sample_frame = aja_audio_channels_ * sizeof(std::int32_t);

                    if (audio_bytes == 0 || audio_bytes > capture_audio_buffer_.size() ||
                        audio_bytes % hardware_bytes_per_sample_frame != 0) {
                        CASPAR_LOG(warning) << print() << L" invalid captured audio byte count: " << audio_bytes;
                        continue;
                    }

                    raw_captured_frame raw;
                    raw.generation = capture_generation_.load(std::memory_order_acquire);
                    raw.sequence   = capture_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
                    raw.uyvy.resize(capture_buffer_.size());
                    std::memcpy(raw.uyvy.data(), capture_buffer_.data(), capture_buffer_.size());

                    // AJA transfers are interleaved using the hardware audio-system
                    // channel count. Normalize immediately to CasparCG's fixed
                    // 16-channel layout used by the capture queue and shared audio adapter.
                    const std::size_t captured_sample_frames = audio_bytes / hardware_bytes_per_sample_frame;
                    raw.audio.assign(captured_sample_frames * kCasparAudioChannels, 0);

                    const auto* hardware_audio = reinterpret_cast<const std::int32_t*>(capture_audio_buffer_.data());

                    for (std::size_t sample = 0; sample < captured_sample_frames; ++sample) {
                        std::copy_n(hardware_audio + sample * aja_audio_channels_,
                                    aja_audio_channels_,
                                    raw.audio.data() + sample * kCasparAudioChannels);
                    }

                    {
                        std::lock_guard<std::mutex> lock(raw_mutex_);
                        raw_queue_.emplace_back(std::move(raw));
                        if (raw_queue_.size() > kRawQueueCapacity) {
                            if (caspar::diagnostics::cadence_enabled())
                                diag_raw_dropped_.fetch_add(1, std::memory_order_relaxed);
                            if (caspar::diagnostics::cadence_enabled())
                                diag_raw_audio_dropped_.fetch_add(raw_queue_.front().audio.size() / kCasparAudioChannels,
                                                              std::memory_order_relaxed);
                            raw_queue_.pop_front();
                            CASPAR_LOG(info) << print() << L" raw capture queue: overflow; dropped oldest frame";
                        }
                    }

                } else {
                    monitor_signal_without_frame();

                    // Do not wait for the next vertical here: a frame can become available
                    // between the status query and the wait, costing an extra frame period.
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }

            stop_autocirculate_if_owner();
            auto_circulate_started_ = false;
        } catch (...) {
            {
                std::lock_guard<std::mutex> lock(exception_mutex_);
                capture_exception_ = std::current_exception();
            }

            stop_autocirculate_if_owner();
            auto_circulate_started_ = false;
        }
    }

    void packaging_loop()
    {
        try {
            while (!stop_requested_) {
                raw_captured_frame raw;
                bool               have_raw = false;

                {
                    std::lock_guard<std::mutex> lock(raw_mutex_);
                    if (!raw_queue_.empty()) {
                        raw = std::move(raw_queue_.front());
                        raw_queue_.pop_front();
                        have_raw = true;
                    }
                }

                if (!have_raw) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                captured_frame captured;
                captured.generation = raw.generation;
                captured.sequence   = raw.sequence;
                captured.uyvy       = std::move(raw.uyvy);
                captured.audio      = std::move(raw.audio);

                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);

                    // A signal-loss/reset can invalidate a raw frame while this
                    // worker is packaging it. Never allow such an in-flight
                    // frame to cross the generation boundary and repopulate the
                    // packaged FIFO after both queues have been flushed.
                    const auto current_generation = capture_generation_.load(std::memory_order_acquire);
                    if (captured.generation != current_generation) {
                        CASPAR_LOG(info) << print() << L" capture generation: discarded stale packaged frame seq="
                                         << captured.sequence << L", frame-generation=" << captured.generation
                                         << L", current-generation=" << current_generation;
                        continue;
                    }

                    capture_queue_.emplace_back(std::move(captured));

                    if (capture_queue_.size() > kCaptureQueueCapacity) {
                        if (caspar::diagnostics::cadence_enabled())
                            diag_packaged_dropped_.fetch_add(1, std::memory_order_relaxed);
                        if (caspar::diagnostics::cadence_enabled())
                            diag_packaged_audio_dropped_.fetch_add(
                            capture_queue_.front().audio.size() / kCasparAudioChannels, std::memory_order_relaxed);
                        capture_queue_.pop_front();
                    }

                    if (!capture_queue_primed_ && capture_queue_.size() >= capture_queue_prime_target_) {
                        capture_queue_primed_ = true;
                        if (caspar::diagnostics::cadence_enabled())
                            CASPAR_LOG(info) << print() << L" capture queue: primed "
                                             << capture_queue_.size() << L"/" << kCaptureQueueCapacity;
                    }
                }
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(exception_mutex_);
            if (!capture_exception_)
                capture_exception_ = std::current_exception();
            stop_requested_ = true;
        }
    }

    bool input_frame_is_stable()
    {
        const auto detected_format = device_.GetInputVideoFormat(input_source_);

        NTV2SDIInStatistics stats;
        NTV2SDIInputStatus  input_status;
        const bool          have_sdi_status =
            device_.ReadSDIStatistics(stats) && stats.GetSDIInputStatus(input_status, static_cast<UWord>(channel_));

        const bool format_good = detected_format == input_format_;

        const bool receiver_good =
            format_good && (!have_sdi_status || (input_status.mLocked && !input_status.mFrameTRSError));

        if (!receiver_good) {
            if (signal_was_good_) {
                const auto new_generation = capture_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
                CASPAR_LOG(info) << print() << L" SDI signal lost/unstable; holding last good frame"
                                 << L"; capture generation=" << new_generation;

                {
                    std::lock_guard<std::mutex> raw_lock(raw_mutex_);
                    raw_queue_.clear();
                }

                std::lock_guard<std::mutex> lock(frame_mutex_);
                capture_queue_.clear();
                capture_queue_primed_       = false;
                capture_queue_prime_target_ = kCaptureQueueStartupPrebuffer;
                clock_adapter_reset_requested_.store(true);
            }

            signal_was_good_        = false;
            clean_reacquire_frames_ = 0;
            return false;
        }

        if (!signal_was_good_) {
            ++clean_reacquire_frames_;

            if (clean_reacquire_frames_ < kCleanFramesAfterReacquire)
                return false;

            signal_was_good_ = true;
            CASPAR_LOG(info) << print() << L" SDI signal stable after " << kCleanFramesAfterReacquire
                             << L" clean frames; capture resumed";
        }

        return true;
    }

    core::draw_frame
    make_caspar_frame(const std::vector<std::uint8_t>& uyvy, const std::int32_t* audio, std::size_t audio_values)
    {
        caspar::diagnostics::cadence_scope aja_make_frame_scope(aja_make_frame_,
                                                                clock_adapter_audio_started_,
                                                                audio_values / kCasparAudioChannels,
                                                                [&] { return print(); });
        core::pixel_format_desc            pixel_desc(core::pixel_format::uyvy, core::color_space::bt709);

        // Match CasparCG's native AV_PIX_FMT_UYVY422 representation.
        // Both logical planes contain the same packed UYVY bytes, exposed
        // with different texel widths for the OpenGL mixer's luma/chroma
        // sampling path.
        pixel_desc.planes.emplace_back(static_cast<int>(width_), static_cast<int>(height_), 2);
        pixel_desc.planes.emplace_back(static_cast<int>(width_ / 2), static_cast<int>(height_), 4);

        auto frame = frame_factory_->create_frame(this, pixel_desc);

        auto& luma_view   = frame.image_data(0);
        auto& chroma_view = frame.image_data(1);

        const std::size_t active_uyvy_bytes = width_ * height_ * 2u;

        if (uyvy.size() < active_uyvy_bytes || luma_view.size() != active_uyvy_bytes ||
            chroma_view.size() != active_uyvy_bytes)
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unexpected CasparCG UYVY frame buffer size"));

        // Mirror CasparCG's established FFmpeg UYVY copy path: the two
        // logical UYVY planes both source the same packed raster (data_map
        // {0, 0}), and their row copies are scheduled in parallel.  The
        // current frame_factory still owns two physical plane buffers; this
        // avoids serialising both full-raster copies on the receive thread.
        const std::size_t row_bytes = width_ * 2u;

        tbb::parallel_invoke(
            [&]() {
                tbb::parallel_for(std::size_t{0}, height_, [&](std::size_t y) {
                    std::memcpy(luma_view.data() + y * row_bytes, uyvy.data() + y * row_bytes, row_bytes);
                });
            },
            [&]() {
                tbb::parallel_for(std::size_t{0}, height_, [&](std::size_t y) {
                    std::memcpy(chroma_view.data() + y * row_bytes, uyvy.data() + y * row_bytes, row_bytes);
                });
            });

        if (audio && audio_values)
            frame.audio_data() = std::vector<std::int32_t>(audio, audio + audio_values);

        return core::draw_frame(std::move(frame));
    }

    void reset_clock_adapter_audio_resampler()
    {
        if (clock_adapter_audio_swr_) {
            swr_close(clock_adapter_audio_swr_);
            if (swr_init(clock_adapter_audio_swr_) < 0)
                CASPAR_THROW_EXCEPTION(caspar_exception()
                                       << msg_info("Unable to reset AJA clock-adapted audio clock resampler"));
        }
        clock_adapter_audio_filter_initialized_ = false;
        clock_adapter_audio_filtered_depth_     = static_cast<double>(kAudioTargetFrames);
        clock_adapter_audio_swr_primed_         = false;
        clock_adapter_audio_input_phase_        = 0.0;
        clock_adapter_audio_correction_ppm_     = 0;
        clock_adapter_audio_compensation_delta_ = 0;
    }

    core::draw_frame receive_clock_adapted(int requested_samples)
    {
        diagnostic_callback_entry(true, requested_samples);

        if (requested_samples <= 0) {
            diagnostic_callback_return(true,
                                       latched_frame_a_ ? L"clock-adapted.nonpositive.still"
                                                        : L"clock-adapted.nonpositive.empty",
                                       requested_samples);
            return latched_frame_a_ ? core::draw_frame::still(latched_frame_a_) : core::draw_frame::empty();
        }

        if (clock_adapter_reset_requested_.exchange(false)) {
            clock_adapter_audio_.clear();
            clock_adapter_audio_offset_  = 0;
            clock_adapter_audio_started_ = false;
            reset_clock_adapter_audio_resampler();
            clock_adapter_generation_ = capture_generation_.load(std::memory_order_acquire);
            clock_adapter_sequence_   = 0;
        }

        std::vector<captured_frame> available;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);

            // Preserve the startup prebuffer, but once started never force a
            // rebuffer merely because the two clocks cross between frames.
            if (!capture_queue_primed_) {
                diagnostic_callback_return(true,
                                           latched_frame_a_ ? L"clock-adapted.unprimed.still"
                                                            : L"clock-adapted.unprimed.empty",
                                           requested_samples);
                return latched_frame_a_ ? core::draw_frame::still(latched_frame_a_) : core::draw_frame::empty();
            }

            while (!capture_queue_.empty()) {
                available.emplace_back(std::move(capture_queue_.front()));
                capture_queue_.pop_front();
            }
        }

        for (auto& captured : available) {
            if (captured.generation != clock_adapter_generation_) {
                clock_adapter_generation_ = captured.generation;
                clock_adapter_sequence_   = 0;
                clock_adapter_audio_.clear();
                clock_adapter_audio_offset_  = 0;
                clock_adapter_audio_started_ = false;
                reset_clock_adapter_audio_resampler();
            }

            if (clock_adapter_sequence_ != 0 && captured.sequence <= clock_adapter_sequence_) {
                CASPAR_LOG(warning) << print() << L" capture sequence anomaly: previous=" << clock_adapter_sequence_
                                    << L", current=" << captured.sequence << L", generation=" << captured.generation;
            }
            clock_adapter_sequence_ = captured.sequence;

            // Keep only the newest video raster. Intermediate physical frames
            // may be skipped when Caspar runs slower than the input clock.
            clock_adapter_video_uyvy_ = std::move(captured.uyvy);

            // Audio is different: retain every captured sample in order.
            clock_adapter_audio_.insert(clock_adapter_audio_.end(), captured.audio.begin(), captured.audio.end());
        }

        if (clock_adapter_video_uyvy_.empty()) {
            diagnostic_callback_return(true,
                                       latched_frame_a_ ? L"clock-adapted.no-video.still" : L"clock-adapted.no-video.empty",
                                       requested_samples);
            return latched_frame_a_ ? core::draw_frame::still(latched_frame_a_) : core::draw_frame::empty();

        }

        const std::size_t requested_frames = static_cast<std::size_t>(requested_samples);
        const std::size_t requested_values = requested_frames * kCasparAudioChannels;
        const std::size_t available_values = clock_adapter_audio_.size() > clock_adapter_audio_offset_
                                                 ? clock_adapter_audio_.size() - clock_adapter_audio_offset_
                                                 : 0u;

        std::vector<std::int32_t> audio_frame(requested_values, 0);

        // Preserve the startup/recovery shock absorber.  The external FIFO
        // is the authoritative reserve; SWR is not allowed to become the
        // reservoir.
        const std::size_t available_frames = available_values / kCasparAudioChannels;
        observe_diagnostic_reservoir(available_frames);
        if (!clock_adapter_audio_started_ && available_frames >= kAudioTargetFrames) {
            clock_adapter_audio_started_            = true;
            clock_adapter_audio_filter_initialized_ = true;
            clock_adapter_audio_filtered_depth_     = static_cast<double>(available_frames);
            if (caspar::diagnostics::cadence_enabled())
                CASPAR_LOG(info) << print() << L" clock-adapted audio reservoir primed " << available_frames << L"/"
                             << kAudioTargetFrames << L" frames";
        }

        int produced_frames = 0;

        if (clock_adapter_audio_started_ && clock_adapter_audio_swr_) {
            // Slow estimator: sample the external reserve AFTER newly captured
            // audio has arrived but BEFORE this callback consumes its block.
            // Observe depth at the same phase on every callback; the
            // servo reacts only to the long-term average, not one bad callback.
            if (!clock_adapter_audio_filter_initialized_) {
                clock_adapter_audio_filtered_depth_     = static_cast<double>(available_frames);
                clock_adapter_audio_filter_initialized_ = true;
            } else {
                clock_adapter_audio_filtered_depth_ +=
                    kAudioFilterAlpha *
                    (static_cast<double>(available_frames) - clock_adapter_audio_filtered_depth_);
            }

            int correction_ppm = 0;
            if (clock_adapter_audio_filtered_depth_ < static_cast<double>(kAudioLowFrames) ||
                clock_adapter_audio_filtered_depth_ > static_cast<double>(kAudioHighFrames)) {
                const auto filtered_error = static_cast<std::int64_t>(std::llround(
                    clock_adapter_audio_filtered_depth_ - static_cast<double>(kAudioTargetFrames)));
                correction_ppm            = static_cast<int>(std::clamp<std::int64_t>(
                    filtered_error, -kAudioMaxCorrectionPpm, kAudioMaxCorrectionPpm));
            }

            if (correction_ppm != clock_adapter_audio_correction_ppm_) {
                // Telemetry sign: positive ppm means consume input faster.
                // FFmpeg's positive sample_delta stretches output, so invert.
                const int sample_delta = -correction_ppm * kAudioCompensationDistance / 1000000;
                if (swr_set_compensation(clock_adapter_audio_swr_, sample_delta, kAudioCompensationDistance) >=
                    0) {
                    clock_adapter_audio_correction_ppm_     = correction_ppm;
                    clock_adapter_audio_compensation_delta_ = sample_delta;
                }
            }

            // Fractional input-consumption budget.  The slow controller must
            // actually move the EXTERNAL reservoir: positive ppm consumes input
            // slightly faster, negative ppm slightly slower.  Carry the
            // fractional part between callbacks so a 1000 ppm request becomes
            // occasional +/-1-frame changes rather than a sample slip in the
            // output.  SWR smooths those tiny input-rate changes while always
            // producing Caspar's exact requested block.
            const double input_budget = static_cast<double>(requested_frames) *
                                            (1.0 + static_cast<double>(clock_adapter_audio_correction_ppm_) / 1000000.0) +
                                        clock_adapter_audio_input_phase_;
            std::size_t  desired_input_frames = static_cast<std::size_t>(std::max(0.0, std::floor(input_budget)));
            clock_adapter_audio_input_phase_    = input_budget - static_cast<double>(desired_input_frames);

            // One-time small prime covers SWR filter delay. This
            // headroom is never added again, so the 3200-frame external
            // shock absorber cannot migrate wholesale into SWR.
            if (!clock_adapter_audio_swr_primed_)
                desired_input_frames += kAudioSWRPrimeFrames;

            const int input_frames = static_cast<int>(std::min<std::size_t>(available_frames, desired_input_frames));
            const uint8_t* input_ptr =
                input_frames > 0
                    ? reinterpret_cast<const uint8_t*>(clock_adapter_audio_.data() + clock_adapter_audio_offset_)
                    : nullptr;
            const uint8_t* input_planes[1]  = {input_ptr};
            uint8_t*       output_ptr       = reinterpret_cast<uint8_t*>(audio_frame.data());
            uint8_t*       output_planes[1] = {output_ptr};

            produced_frames = swr_convert(clock_adapter_audio_swr_,
                                          output_planes,
                                          requested_samples,
                                          input_frames > 0 ? input_planes : nullptr,
                                          input_frames);
            if (produced_frames < 0)
                CASPAR_THROW_EXCEPTION(caspar_exception()
                                       << msg_info("AJA clock-adapted audio resampler conversion failed"));

            std::size_t consumed_input_frames = static_cast<std::size_t>(input_frames);

            // SWR's filter/phase state can occasionally leave the first conversion
            // one sample short even though the external reservoir still has plenty
            // of audio.  Top up the SAME output block from the external reservoir
            // instead of exposing a zero-filled tail to Caspar.  This preserves the
            // fractional input budget as the normal clock-domain servo; the extra
            // input is consumed only when SWR proves that it needs it to complete
            // the requested fixed-size output block.
            if (produced_frames < requested_samples && consumed_input_frames < available_frames) {
                const int         remaining_output   = requested_samples - produced_frames;
                const std::size_t remaining_external = available_frames - consumed_input_frames;
                const std::size_t topup_input_frames =
                    std::min<std::size_t>(remaining_external, static_cast<std::size_t>(remaining_output));

                const uint8_t* topup_input_ptr =
                    reinterpret_cast<const uint8_t*>(clock_adapter_audio_.data() + clock_adapter_audio_offset_ +
                                                     consumed_input_frames * kCasparAudioChannels);
                const uint8_t* topup_input_planes[1] = {topup_input_ptr};
                uint8_t*       topup_output_ptr      = reinterpret_cast<uint8_t*>(
                    audio_frame.data() + static_cast<std::size_t>(produced_frames) * kCasparAudioChannels);
                uint8_t* topup_output_planes[1] = {topup_output_ptr};

                const int topup_produced = swr_convert(clock_adapter_audio_swr_,
                                                       topup_output_planes,
                                                       remaining_output,
                                                       topup_input_planes,
                                                       static_cast<int>(topup_input_frames));
                if (topup_produced < 0)
                    CASPAR_THROW_EXCEPTION(caspar_exception()
                                           << msg_info("AJA clock-adapted audio resampler top-up conversion failed"));

                consumed_input_frames += topup_input_frames;
                produced_frames += topup_produced;
            }

            if (caspar::diagnostics::cadence_enabled())
                diag_swr_input_.fetch_add(consumed_input_frames, std::memory_order_relaxed);
            if (caspar::diagnostics::cadence_enabled())
                diag_swr_output_.fetch_add(static_cast<std::uint64_t>(produced_frames), std::memory_order_relaxed);
            observe_diagnostic_reservoir(available_frames - consumed_input_frames);
            clock_adapter_audio_offset_ += consumed_input_frames * kCasparAudioChannels;
            if (!clock_adapter_audio_swr_primed_ && consumed_input_frames > 0)
                clock_adapter_audio_swr_primed_ = true;

            if (produced_frames < requested_samples) {
                if (caspar::diagnostics::cadence_enabled())
                    diag_underflows_.fetch_add(1, std::memory_order_relaxed);
                if (caspar::diagnostics::cadence_enabled())
                    diag_missing_output_.fetch_add(requested_samples - produced_frames, std::memory_order_relaxed);
                CASPAR_LOG(warning) << print() << L" clock-adapted audio underflow after SWR top-up: requested="
                                    << requested_samples << L", produced=" << produced_frames << L", external-before="
                                    << available_frames << L", consumed-input=" << consumed_input_frames
                                    << L", correction-ppm=" << clock_adapter_audio_correction_ppm_ << L", swr-delay="
                                    << swr_get_delay(clock_adapter_audio_swr_, kAudioRate);
            }
        }

        // Compact occasionally rather than erasing on every callback.
        if (clock_adapter_audio_offset_ >= 65536 && clock_adapter_audio_offset_ * 2 >= clock_adapter_audio_.size()) {
            clock_adapter_audio_.erase(clock_adapter_audio_.begin(),
                                     clock_adapter_audio_.begin() + clock_adapter_audio_offset_);
            clock_adapter_audio_offset_ = 0;
        }

        // Build a fresh Caspar frame every callback so repeated video can carry
        // the next audio slice. This is intentional clock-domain adaptation,
        // not a capture underflow.
        latched_frame_a_ = make_caspar_frame(clock_adapter_video_uyvy_, audio_frame.data(), audio_frame.size());

        diagnostic_callback_return(true,
                                   available.empty() ? L"clock-adapted.new-frame.cached-video"
                                                     : L"clock-adapted.new-frame.new-video",
                                   requested_samples);
        return latched_frame_a_;
    }

    void rethrow_capture_exception()
    {
        std::exception_ptr exception;

        {
            std::lock_guard<std::mutex> lock(exception_mutex_);
            exception = capture_exception_;
        }

        if (exception)
            std::rethrow_exception(exception);
    }
};

} // namespace

spl::shared_ptr<core::frame_producer> create_producer(const core::frame_producer_dependencies& dependencies,
                                                      const std::vector<std::wstring>&         params)
{
    if (params.empty() || !boost::iequals(params.at(0), L"AJA"))
        return core::frame_producer::empty();

    const int  device_number  = get_param(L"DEVICE", params, 1);
    const int  channel_number = get_param(L"CHANNEL", params, 1);
    const auto format         = get_param(L"FORMAT", params);

    if (format.empty())
        CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA FORMAT parameter is required"));

    if (device_number < 1)
        CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA DEVICE must be 1 or greater"));

    if (channel_number < 1 || channel_number > 8)
        CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA CHANNEL must be between 1 and 8"));

    return spl::make_shared<aja_producer>(dependencies.frame_factory,
                                          dependencies.format_desc,
                                          dependencies.format_repository,
                                          static_cast<ULWord>(device_number - 1),
                                          static_cast<NTV2Channel>(channel_number - 1),
                                          format);
}

}} // namespace caspar::aja
