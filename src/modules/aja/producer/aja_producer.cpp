#include "aja_producer.h"

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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace caspar { namespace aja {
namespace {

constexpr NTV2FrameBufferFormat kPixelFormat         = NTV2_FBF_8BIT_YCBCR;
constexpr ULWord                kAutoCirculateFrames = 7;
constexpr ULWord                kAudioChannels       = 16;

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

inline std::uint8_t clamp_byte(int value) { return static_cast<std::uint8_t>(std::max(0, std::min(255, value))); }

void uyvy_to_bgra(const std::uint8_t* src, std::uint8_t* dst, std::size_t width, std::size_t height)
{
    const std::size_t pixels = width * height;

    for (std::size_t i = 0, o = 0; i < pixels; i += 2, o += 4) {
        const int u  = static_cast<int>(src[o + 0]) - 128;
        const int y0 = static_cast<int>(src[o + 1]) - 16;
        const int v  = static_cast<int>(src[o + 2]) - 128;
        const int y1 = static_cast<int>(src[o + 3]) - 16;

        const auto write_pixel = [&](std::size_t pixel, int y) {
            y           = std::max(0, y);
            const int c = 298 * y;
            const int r = (c + 459 * v + 128) >> 8;
            const int g = (c - 55 * u - 136 * v + 128) >> 8;
            const int b = (c + 541 * u + 128) >> 8;

            dst[pixel * 4 + 0] = clamp_byte(b);
            dst[pixel * 4 + 1] = clamp_byte(g);
            dst[pixel * 4 + 2] = clamp_byte(r);
            dst[pixel * 4 + 3] = 255;
        };

        write_pixel(i + 0, y0);
        write_pixel(i + 1, y1);
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
    NTV2InputSource input_source_ = NTV2_INPUTSOURCE_INVALID;
    NTV2VideoFormat input_format_ = NTV2_FORMAT_UNKNOWN;
    NTV2AudioSystem audio_system_ = NTV2_AUDIOSYSTEM_1;

    std::size_t width_  = 0;
    std::size_t height_ = 0;

    std::vector<std::uint8_t> capture_buffer_;
    std::vector<std::uint8_t> capture_audio_buffer_;

    struct captured_frame
    {
        std::vector<std::uint8_t> bgra;
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

    static constexpr std::size_t kRawQueueCapacity            = 4;
    static constexpr std::size_t kCaptureQueueCapacity        = 4;
    static constexpr std::size_t kCaptureQueueStartupPrebuffer = 3;
    static constexpr std::size_t kCaptureQueueRebuffer         = 2;

    mutable std::mutex             raw_mutex_;
    std::deque<raw_captured_frame> raw_queue_;

    mutable std::mutex         frame_mutex_;
    std::deque<captured_frame> capture_queue_;
    bool                       capture_queue_primed_ = false;
    std::size_t                capture_queue_prime_target_ = kCaptureQueueStartupPrebuffer;

    mutable std::mutex exception_mutex_;
    std::exception_ptr capture_exception_;

    std::atomic<bool>          stop_requested_{false};
    std::atomic<std::uint64_t> capture_generation_{0};
    std::atomic<std::uint64_t> capture_sequence_{0};
    std::thread                capture_thread_;
    std::thread                conversion_thread_;

    bool auto_circulate_started_ = false;

    std::chrono::steady_clock::time_point last_no_frame_signal_check_ =
        std::chrono::steady_clock::now();

    static constexpr unsigned kCleanFramesAfterReacquire = 3;
    bool                      signal_was_good_           = true;
    unsigned                  clean_reacquire_frames_    = kCleanFramesAfterReacquire;

    // Passive SDI receiver diagnostics. These counters are observational only:
    // they never reject a frame or alter capture/recovery behavior.
    ULWord sdi_diag_unlock_count_ = 0;
    ULWord sdi_diag_crc_a_        = 0;
    ULWord sdi_diag_crc_b_        = 0;
    bool   sdi_diag_initialized_  = false;

    core::draw_frame latched_frame_a_;
    core::draw_frame latched_frame_b_;
    bool             deliver_b_audio_  = false;
    bool             interlaced_second_half_ = false;
    std::atomic_bool interlaced_phase_reset_requested_{false};
    std::uint64_t    last_latched_generation_ = 0;
    std::uint64_t    last_latched_sequence_   = 0;

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
        conversion_thread_ = std::thread([this] { conversion_loop(); });
        capture_thread_    = std::thread([this] { capture_loop(); });
    }

    ~aja_producer() override
    {
        stop_requested_ = true;

        try {
            if (device_.IsOpen())
                device_.AutoCirculateStop(channel_);
        } catch (...) {
        }

        if (capture_thread_.joinable())
            capture_thread_.join();
        if (conversion_thread_.joinable())
            conversion_thread_.join();

        try {
            device_.AutoCirculateStop(channel_);
            device_.DisableChannel(channel_);
        } catch (...) {
        }

        CASPAR_LOG(info) << print() << L" Uninitialized";
    }

    core::draw_frame receive_impl(const core::video_field field, int nb_samples) override
    {
        rethrow_capture_exception();

        if (input_format_desc_.field_count == 2) {
            if (interlaced_phase_reset_requested_.exchange(false)) {
                interlaced_second_half_ = false;
                deliver_b_audio_        = false;
            }

            // Caspar currently reaches this producer with video_field == 0 for
            // every 1080i callback, so field cannot be used to select A/B.
            // Drive the two-half delivery explicitly:
            //   first callback  -> pop one complete 25 Hz capture, return A
            //   second callback -> reuse that capture, return B
            //
            // Startup/probe calls use nb_samples == 0. They must not consume
            // FIFO entries or advance the A/B phase.
            if (nb_samples <= 0)
                return latched_frame_a_ ? core::draw_frame::still(latched_frame_a_)
                                        : core::draw_frame::empty();

            if (!interlaced_second_half_ || !latched_frame_a_) {
                const bool new_pair = latch_latest_pair(nb_samples);
                deliver_b_audio_    = new_pair;

                if (!new_pair) {
                    interlaced_second_half_ = false;
                    return latched_frame_a_ ? core::draw_frame::still(latched_frame_a_)
                                            : core::draw_frame::empty();
                }

                interlaced_second_half_ = true;
                return latched_frame_a_;
            }

            const bool with_audio = deliver_b_audio_;
            deliver_b_audio_      = false;
            interlaced_second_half_ = false;

            return latched_frame_b_ ? (with_audio ? latched_frame_b_ : core::draw_frame::still(latched_frame_b_))
                                    : core::draw_frame::empty();
        }

        const bool new_frame = latch_latest_pair(nb_samples);

        return latched_frame_a_ ? (new_frame ? latched_frame_a_ : core::draw_frame::still(latched_frame_a_))
                                : core::draw_frame::empty();
    }

    core::draw_frame first_frame(const core::video_field field) override { return receive_impl(field, 0); }

    core::draw_frame last_frame(const core::video_field field) override
    {
        if (!latched_frame_a_)
            latch_latest_pair(0);

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

        if (!device_.SetVideoFormat(capture_format, false, false, channel_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to set AJA input video format"));

        device_.SetSDIInLevelBtoLevelAConversion(channel_, level_b_to_a);

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

        if (!device_.SetNumberAudioChannels(kAudioChannels, audio_system_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to configure 16-channel AJA audio input"));

        if (!device_.SetAudioRate(NTV2_AUDIO_48K, audio_system_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to configure AJA audio rate"));

        if (!device_.SetAudioBufferSize(NTV2_AUDIO_BUFFER_SIZE_4MB, audio_system_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to configure AJA audio buffer"));

        if (!device_.SetAudioLoopBack(NTV2_AUDIO_LOOPBACK_OFF, audio_system_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to disable AJA audio loopback"));

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

        sdi_diag_unlock_count_ = device_.GetSDIUnlockCount(channel_);
        sdi_diag_crc_a_        = device_.GetCRCErrorCountA(channel_);
        sdi_diag_crc_b_        = device_.GetCRCErrorCountB(channel_);
        sdi_diag_initialized_  = true;

        CASPAR_LOG(info) << print()
                         << L" SDI diagnostics baseline: unlocks=" << sdi_diag_unlock_count_
                         << L", CRC-A=" << sdi_diag_crc_a_
                         << L", CRC-B=" << sdi_diag_crc_b_;

        state_["device"]            = static_cast<int64_t>(device_index_ + 1);
        state_["channel"]           = static_cast<int64_t>(static_cast<int>(channel_) + 1);
        state_["format"]            = input_format_desc_.name;
        state_["audio/sample-rate"] = static_cast<int64_t>(48000);
        state_["audio/channels"]    = static_cast<int64_t>(kAudioChannels);

        CASPAR_LOG(info) << L"AJA producer initialized: device " << (device_index_ + 1) << L", input channel "
                         << (static_cast<int>(channel_) + 1) << L", detected " << input_format_desc_.name
                         << L", 16-channel 48 kHz embedded audio";
    }

    void monitor_sdi_diagnostics()
    {
        const ULWord unlock_count = device_.GetSDIUnlockCount(channel_);
        const ULWord crc_a        = device_.GetCRCErrorCountA(channel_);
        const ULWord crc_b        = device_.GetCRCErrorCountB(channel_);

        if (!sdi_diag_initialized_) {
            sdi_diag_unlock_count_ = unlock_count;
            sdi_diag_crc_a_        = crc_a;
            sdi_diag_crc_b_        = crc_b;
            sdi_diag_initialized_  = true;
            return;
        }

        if (unlock_count == sdi_diag_unlock_count_ &&
            crc_a == sdi_diag_crc_a_ &&
            crc_b == sdi_diag_crc_b_)
            return;

        CASPAR_LOG(warning) << print()
                            << L" SDI diagnostics changed: unlocks="
                            << sdi_diag_unlock_count_ << L"->" << unlock_count
                            << L", CRC-A=" << sdi_diag_crc_a_ << L"->" << crc_a
                            << L", CRC-B=" << sdi_diag_crc_b_ << L"->" << crc_b;

        sdi_diag_unlock_count_ = unlock_count;
        sdi_diag_crc_a_        = crc_a;
        sdi_diag_crc_b_        = crc_b;
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
        monitor_sdi_diagnostics();
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

                    if (!input_frame_is_stable())
                        continue;

                    monitor_sdi_diagnostics();

                    const ULWord audio_bytes            = transfer.GetCapturedAudioByteCount();
                    const ULWord bytes_per_sample_frame = kAudioChannels * sizeof(std::int32_t);

                    if (audio_bytes == 0 || audio_bytes > capture_audio_buffer_.size() ||
                        audio_bytes % bytes_per_sample_frame != 0) {
                        CASPAR_LOG(warning) << print() << L" invalid captured audio byte count: " << audio_bytes;
                        continue;
                    }

                    raw_captured_frame raw;
                    raw.generation = capture_generation_.load(std::memory_order_acquire);
                    raw.sequence   = capture_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
                    raw.uyvy.resize(capture_buffer_.size());
                    std::memcpy(raw.uyvy.data(), capture_buffer_.data(), capture_buffer_.size());
                    raw.audio.resize(audio_bytes / sizeof(std::int32_t));
                    std::memcpy(raw.audio.data(), capture_audio_buffer_.data(), audio_bytes);


                    {
                        std::lock_guard<std::mutex> lock(raw_mutex_);
                        raw_queue_.emplace_back(std::move(raw));
                        if (raw_queue_.size() > kRawQueueCapacity) {
                            raw_queue_.pop_front();
                            CASPAR_LOG(info) << print()
                                             << L" raw capture queue: overflow; dropped oldest frame";
                        }
                    }

                } else {
                    monitor_signal_without_frame();

                    // Do not wait for the next vertical here: a frame can become available
                    // between the status query and the wait, costing an extra frame period.
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }

            device_.AutoCirculateStop(channel_);
            auto_circulate_started_ = false;
        } catch (...) {
            {
                std::lock_guard<std::mutex> lock(exception_mutex_);
                capture_exception_ = std::current_exception();
            }

            try {
                device_.AutoCirculateStop(channel_);
            } catch (...) {
            }

            auto_circulate_started_ = false;
        }
    }

    void conversion_loop()
    {
        try {
            while (!stop_requested_) {
                raw_captured_frame raw;
                bool have_raw = false;

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
                captured.bgra.resize(width_ * height_ * 4u);
                uyvy_to_bgra(raw.uyvy.data(), captured.bgra.data(), width_, height_);
                captured.audio = std::move(raw.audio);


                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);

                    // A signal-loss/reset can invalidate a raw frame while this
                    // worker is converting it. Never allow such an in-flight
                    // frame to cross the generation boundary and repopulate the
                    // converted FIFO after both queues have been flushed.
                    const auto current_generation = capture_generation_.load(std::memory_order_acquire);
                    if (captured.generation != current_generation) {
                        CASPAR_LOG(info) << print()
                                         << L" capture generation: discarded stale converted frame seq="
                                         << captured.sequence << L", frame-generation=" << captured.generation
                                         << L", current-generation=" << current_generation;
                        continue;
                    }

                    capture_queue_.emplace_back(std::move(captured));

                    if (capture_queue_.size() > kCaptureQueueCapacity) {
                        capture_queue_.pop_front();
                        CASPAR_LOG(info) << print()
                                         << L" capture queue: overflow; dropped oldest frame";
                    }

                    if (!capture_queue_primed_ &&
                        capture_queue_.size() >= capture_queue_prime_target_) {
                        capture_queue_primed_ = true;
                        CASPAR_LOG(info) << print()
                                         << (capture_queue_prime_target_ == kCaptureQueueStartupPrebuffer
                                                 ? L" capture queue: primed "
                                                 : L" capture queue: rebuffered ")
                                         << capture_queue_.size() << L"/"
                                         << kCaptureQueueCapacity;
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

        const bool receiver_good =
            detected_format == input_format_ && have_sdi_status && input_status.mLocked && !input_status.mFrameTRSError;

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
                interlaced_phase_reset_requested_.store(true);
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
    make_caspar_frame(const std::vector<std::uint8_t>& bgra, const std::int32_t* audio, std::size_t audio_values)
    {
        core::pixel_format_desc pixel_desc(core::pixel_format::bgra, core::color_space::bt709);
        pixel_desc.planes.emplace_back(static_cast<int>(width_), static_cast<int>(height_), 4);

        auto  frame = frame_factory_->create_frame(this, pixel_desc);
        auto& image = frame.image_data(0);

        if (image.size() != bgra.size())
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unexpected CasparCG BGRA frame buffer size"));

        std::memcpy(image.data(), bgra.data(), bgra.size());

        if (audio && audio_values)
            frame.audio_data() = std::vector<std::int32_t>(audio, audio + audio_values);

        return core::draw_frame(std::move(frame));
    }

    bool latch_latest_pair(int requested_samples_per_field)
    {
        captured_frame captured;

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);

            if (!capture_queue_primed_)
                return false;

            if (capture_queue_.empty()) {
                if (signal_was_good_ && latched_frame_a_) {
                    CASPAR_LOG(info) << print()
                                     << L" capture queue: unexpected underflow; rebuffering";
                }
                capture_queue_primed_       = false;
                capture_queue_prime_target_ = kCaptureQueueRebuffer;
                return false;
            }

            captured = std::move(capture_queue_.front());
            capture_queue_.pop_front();
        }

        if (captured.generation == last_latched_generation_) {
            if (last_latched_sequence_ != 0 && captured.sequence <= last_latched_sequence_) {
                CASPAR_LOG(warning) << print()
                                    << L" capture sequence anomaly: previous=" << last_latched_sequence_
                                    << L", current=" << captured.sequence
                                    << L", generation=" << captured.generation;
            }
        } else {
            last_latched_generation_ = captured.generation;
        }
        last_latched_sequence_ = captured.sequence;

        auto& bgra  = captured.bgra;
        auto& audio = captured.audio;

        const std::size_t captured_sample_frames = audio.size() / kAudioChannels;

        const std::size_t requested =
            requested_samples_per_field > 0 ? static_cast<std::size_t>(requested_samples_per_field) : 0u;

        if (input_format_desc_.field_count == 2) {
            std::vector<std::int32_t> audio_a(requested * kAudioChannels, 0);
            std::vector<std::int32_t> audio_b(requested * kAudioChannels, 0);

            const std::size_t copy_a_frames = std::min(requested, captured_sample_frames);
            if (copy_a_frames) {
                std::memcpy(audio_a.data(), audio.data(), copy_a_frames * kAudioChannels * sizeof(std::int32_t));
            }

            const std::size_t remaining_frames =
                captured_sample_frames > copy_a_frames ? captured_sample_frames - copy_a_frames : 0u;
            const std::size_t copy_b_frames = std::min(requested, remaining_frames);
            if (copy_b_frames) {
                std::memcpy(audio_b.data(),
                            audio.data() + copy_a_frames * kAudioChannels,
                            copy_b_frames * kAudioChannels * sizeof(std::int32_t));
            }

            latched_frame_a_ = make_caspar_frame(bgra, audio_a.data(), audio_a.size());
            latched_frame_b_ = make_caspar_frame(bgra, audio_b.data(), audio_b.size());
        } else {
            std::vector<std::int32_t> audio_frame(requested * kAudioChannels, 0);
            const std::size_t         copy_frames = std::min(requested, captured_sample_frames);

            if (copy_frames) {
                std::memcpy(audio_frame.data(), audio.data(), copy_frames * kAudioChannels * sizeof(std::int32_t));
            }

            latched_frame_a_ = make_caspar_frame(bgra, audio_frame.data(), audio_frame.size());
            latched_frame_b_ = core::draw_frame::empty();
        }
        return true;
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
