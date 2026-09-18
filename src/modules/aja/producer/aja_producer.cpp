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
#include <cstdint>
#include <cstring>
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
            y = std::max(0, y);
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

    mutable std::mutex        frame_mutex_;
    std::vector<std::uint8_t> latest_bgra_;
    std::vector<std::int32_t> latest_audio_;
    std::uint64_t             latest_sequence_ = 0;
    bool                      have_frame_      = false;

    mutable std::mutex exception_mutex_;
    std::exception_ptr capture_exception_;

    std::atomic<bool> stop_requested_{false};
    std::thread       capture_thread_;

    bool auto_circulate_started_ = false;

    static constexpr unsigned kCleanFramesAfterReacquire = 3;
    bool                      signal_was_good_           = true;
    unsigned                  clean_reacquire_frames_    = kCleanFramesAfterReacquire;

    core::draw_frame latched_frame_a_;
    core::draw_frame latched_frame_b_;
    std::uint64_t    latched_sequence_ = 0;
    bool             deliver_b_audio_  = false;

    core::monitor::state state_;

  public:
    aja_producer(const spl::shared_ptr<core::frame_factory>& frame_factory,
                 const core::video_format_desc&              channel_format_desc,
                 ULWord                                      device_index,
                 NTV2Channel                                 channel)
        : frame_factory_(frame_factory)
        , channel_format_desc_(channel_format_desc)
        , device_index_(device_index)
        , channel_(channel)
    {
        initialize();
        capture_thread_ = std::thread([this] { capture_loop(); });
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

        if (channel_format_desc_.field_count == 2) {
            if (field == core::video_field::a || !latched_frame_a_) {
                const bool new_pair = latch_latest_pair(nb_samples);
                deliver_b_audio_    = new_pair;

                return latched_frame_a_
                           ? (new_pair ? latched_frame_a_ : core::draw_frame::still(latched_frame_a_))
                           : core::draw_frame::empty();
            }

            const bool with_audio = deliver_b_audio_;
            deliver_b_audio_      = false;

            return latched_frame_b_
                       ? (with_audio ? latched_frame_b_ : core::draw_frame::still(latched_frame_b_))
                       : core::draw_frame::empty();
        }

        const bool new_frame = latch_latest_pair(nb_samples);
        return latched_frame_a_
                   ? (new_frame ? latched_frame_a_ : core::draw_frame::still(latched_frame_a_))
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
        return have_frame_;
    }

    std::wstring print() const override
    {
        return L"AJA input [" + std::to_wstring(device_index_ + 1) + L"|" +
               std::to_wstring(static_cast<int>(channel_) + 1) + L"|1080i5000]";
    }

    std::wstring name() const override { return L"aja"; }
    core::monitor::state state() const override { return state_; }

  private:
    void initialize()
    {
        if (channel_format_desc_.format != core::video_format::x1080i5000) {
            CASPAR_THROW_EXCEPTION(user_error()
                                   << msg_info("Initial AJA input producer requires a 1080i5000 CasparCG channel"));
        }

        CNTV2DeviceScanner scanner(true);

        if (!CNTV2DeviceScanner::GetDeviceAtIndex(device_index_, device_))
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Unable to open selected AJA input device"));

        if (!device_.features().CanDoCapture())
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Selected AJA device does not support capture"));

        if (!device_.features().CanDoChannel(channel_))
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Selected AJA device does not support requested input channel"));

        input_source_ = ::NTV2ChannelToInputSource(channel_, NTV2_IOKINDS_SDI);

        if (!device_.features().CanDoInputSource(input_source_))
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Selected AJA device does not provide requested SDI input"));

        device_.SetEveryFrameServices(NTV2_OEM_TASKS);

        if (!device_.EnableChannel(channel_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to enable selected AJA input channel"));

        device_.SetMode(channel_, NTV2_MODE_CAPTURE);

        if (device_.features().HasBiDirectionalSDI()) {
            device_.SetSDITransmitEnable(channel_, false);
            for (int i = 0; i < 10; ++i)
                device_.WaitForInputVerticalInterrupt(channel_);
        }

        input_format_ = device_.GetInputVideoFormat(input_source_);

        if (input_format_ == NTV2_FORMAT_UNKNOWN)
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("No signal or unknown video format on selected AJA SDI input"));

        if (input_format_ != NTV2_FORMAT_1080i_5000)
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Initial AJA input producer supports 1080i5000 input only"));

        if (!device_.features().CanDoVideoFormat(input_format_))
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Selected AJA device cannot capture detected video format"));

        if (!device_.SetVideoFormat(input_format_, false, false, channel_))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to set AJA input video format"));

        device_.SetVANCMode(NTV2_VANCMODE_OFF, channel_);
        device_.SetVANCShiftMode(channel_, NTV2_VANCDATA_NORMAL);

        if (!device_.SetFrameBufferFormat(channel_, kPixelFormat))
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to set AJA input framebuffer to 8-bit YCbCr"));

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
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to route selected AJA SDI input to FrameStore"));

        const NTV2FormatDescriptor format_desc(input_format_, kPixelFormat);
        width_  = static_cast<std::size_t>(format_desc.GetRasterWidth());
        height_ = static_cast<std::size_t>(format_desc.GetRasterHeight());

        if (width_ != 1920 || height_ != 1080)
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unexpected raster size for AJA 1080i5000 input"));

        capture_buffer_.resize(static_cast<std::size_t>(format_desc.GetVideoWriteSize()));
        capture_audio_buffer_.resize(256 * 1024);
        latest_bgra_.resize(width_ * height_ * 4u);

        state_["device"]                = static_cast<int64_t>(device_index_ + 1);
        state_["channel"]               = static_cast<int64_t>(static_cast<int>(channel_) + 1);
        state_["format"]                = std::wstring(L"1080i5000");
        state_["audio/sample-rate"]     = static_cast<int64_t>(48000);
        state_["audio/channels"]        = static_cast<int64_t>(kAudioChannels);

        CASPAR_LOG(info) << L"AJA producer initialized: device " << (device_index_ + 1) << L", input channel "
                         << (static_cast<int>(channel_) + 1)
                         << L", detected 1080i5000, 16-channel 48 kHz embedded audio";
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

                    const ULWord audio_bytes           = transfer.GetCapturedAudioByteCount();
                    const ULWord bytes_per_sample_frame = kAudioChannels * sizeof(std::int32_t);

                    if (audio_bytes == 0 || audio_bytes > capture_audio_buffer_.size() ||
                        audio_bytes % bytes_per_sample_frame != 0) {
                        CASPAR_LOG(warning) << print() << L" invalid captured audio byte count: " << audio_bytes;
                        continue;
                    }

                    std::vector<std::uint8_t> converted(width_ * height_ * 4u);
                    uyvy_to_bgra(capture_buffer_.data(), converted.data(), width_, height_);

                    {
                        std::lock_guard<std::mutex> lock(frame_mutex_);
                        latest_bgra_.swap(converted);
                        latest_audio_.resize(audio_bytes / sizeof(std::int32_t));
                        std::memcpy(latest_audio_.data(), capture_audio_buffer_.data(), audio_bytes);
                        ++latest_sequence_;
                        have_frame_ = true;
                    }
                } else {
                    device_.WaitForInputVerticalInterrupt(channel_);
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

    bool input_frame_is_stable()
    {
        const auto detected_format = device_.GetInputVideoFormat(input_source_);

        NTV2SDIInStatistics stats;
        NTV2SDIInputStatus  input_status;
        const bool          have_sdi_status =
            device_.ReadSDIStatistics(stats) && stats.GetSDIInputStatus(input_status, static_cast<UWord>(channel_));

        const bool receiver_good = detected_format == NTV2_FORMAT_1080i_5000 && have_sdi_status &&
                                   input_status.mLocked && !input_status.mFrameTRSError;

        if (!receiver_good) {
            if (signal_was_good_)
                CASPAR_LOG(info) << print() << L" SDI signal lost/unstable; holding last good frame";

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

    core::draw_frame make_caspar_frame(const std::vector<std::uint8_t>& bgra,
                                       const std::int32_t*              audio,
                                       std::size_t                      audio_values)
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
        std::vector<std::uint8_t> bgra;
        std::vector<std::int32_t> audio;
        std::uint64_t             sequence = 0;

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);

            if (!have_frame_)
                return false;

            if (latched_frame_a_ && latest_sequence_ == latched_sequence_)
                return false;

            bgra     = latest_bgra_;
            audio    = latest_audio_;
            sequence = latest_sequence_;
        }

        const std::size_t captured_sample_frames = audio.size() / kAudioChannels;
        const std::size_t requested =
            requested_samples_per_field > 0 ? static_cast<std::size_t>(requested_samples_per_field) : 0u;

        std::vector<std::int32_t> audio_a(requested * kAudioChannels, 0);
        std::vector<std::int32_t> audio_b(requested * kAudioChannels, 0);

        const std::size_t copy_a_frames = std::min(requested, captured_sample_frames);
        if (copy_a_frames) {
            std::memcpy(audio_a.data(),
                        audio.data(),
                        copy_a_frames * kAudioChannels * sizeof(std::int32_t));
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
        latched_sequence_ = sequence;
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

    const int device_number  = get_param(L"DEVICE", params, 1);
    const int channel_number = get_param(L"CHANNEL", params, 1);

    if (device_number < 1)
        CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA DEVICE must be 1 or greater"));

    if (channel_number < 1 || channel_number > 8)
        CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA CHANNEL must be between 1 and 8"));

    return spl::make_shared<aja_producer>(dependencies.frame_factory,
                                          dependencies.format_desc,
                                          static_cast<ULWord>(device_number - 1),
                                          static_cast<NTV2Channel>(channel_number - 1));
}

}} // namespace caspar::aja
