#include "aja_consumer.h"

#include <common/except.h>
#include <common/future.h>
#include <common/log.h>

#include <core/consumer/channel_info.h>
#include <core/consumer/frame_consumer.h>
#include <core/frame/frame.h>
#include <core/frame/pixel_format.h>
#include <core/video_format.h>

#include <ajantv2/includes/ntv2card.h>
#include <ajantv2/includes/ntv2devicescanner.h>
#include <ajantv2/includes/ntv2enums.h>
#include <ajantv2/includes/ntv2utils.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <vector>

namespace caspar { namespace aja {
namespace {

constexpr NTV2FrameBufferFormat kPixelFormat = NTV2_FBF_8BIT_YCBCR;

inline std::uint8_t clamp_byte(int value) { return static_cast<std::uint8_t>(std::max(0, std::min(255, value))); }

void bgra_to_uyvy(const std::uint8_t* src, std::uint8_t* dst, std::size_t width, std::size_t height)
{
    const std::size_t pixels = width * height;

    for (std::size_t i = 0, o = 0; i < pixels; i += 2, o += 4) {
        const int b0 = src[(i + 0) * 4 + 0];
        const int g0 = src[(i + 0) * 4 + 1];
        const int r0 = src[(i + 0) * 4 + 2];

        const int b1 = src[(i + 1) * 4 + 0];
        const int g1 = src[(i + 1) * 4 + 1];
        const int r1 = src[(i + 1) * 4 + 2];

        // BT.709 limited-range integer approximation.
        const int y0 = ((47 * r0 + 157 * g0 + 16 * b0 + 128) >> 8) + 16;
        const int y1 = ((47 * r1 + 157 * g1 + 16 * b1 + 128) >> 8) + 16;

        const int u0 = ((-26 * r0 - 87 * g0 + 112 * b0 + 128) >> 8) + 128;
        const int v0 = ((112 * r0 - 102 * g0 - 10 * b0 + 128) >> 8) + 128;

        const int u1 = ((-26 * r1 - 87 * g1 + 112 * b1 + 128) >> 8) + 128;
        const int v1 = ((112 * r1 - 102 * g1 - 10 * b1 + 128) >> 8) + 128;

        dst[o + 0] = clamp_byte((u0 + u1) / 2);
        dst[o + 1] = clamp_byte(y0);
        dst[o + 2] = clamp_byte((v0 + v1) / 2);
        dst[o + 3] = clamp_byte(y1);
    }
}

void bgra_field_to_interlaced_uyvy(const uint8_t* src, uint8_t* dst, int width, int height, int first_line)
{
    const int src_stride = width * 4;
    const int dst_stride = width * 2;

    for (int y = first_line; y < height; y += 2) {
        bgra_to_uyvy(src + y * src_stride, dst + y * dst_stride, width, 1);
    }
}

void bgra_to_key_uyvy(const std::uint8_t* src, std::uint8_t* dst, std::size_t width, std::size_t height)
{
    const std::size_t pixels = width * height;

    for (std::size_t i = 0, o = 0; i < pixels; i += 2, o += 4) {
        const int a0 = src[(i + 0) * 4 + 3];
        const int a1 = src[(i + 1) * 4 + 3];

        // Convert full-range 8-bit alpha to legal-range luma.
        const std::uint8_t y0 = static_cast<std::uint8_t>(16 + ((a0 * 219 + 127) / 255));
        const std::uint8_t y1 = static_cast<std::uint8_t>(16 + ((a1 * 219 + 127) / 255));

        dst[o + 0] = 128;
        dst[o + 1] = y0;
        dst[o + 2] = 128;
        dst[o + 3] = y1;
    }
}

void bgra_field_to_interlaced_key_uyvy(const std::uint8_t* src,
                                       std::uint8_t*       dst,
                                       int                 width,
                                       int                 height,
                                       int                 first_line)
{
    const int src_stride = width * 4;
    const int dst_stride = width * 2;

    for (int y = first_line; y < height; y += 2) {
        bgra_to_key_uyvy(src + y * src_stride, dst + y * dst_stride, width, 1);
    }
}

NTV2VideoFormat get_aja_video_format(core::video_format format)
{
    switch (format) {
        case core::video_format::pal:
            return NTV2_FORMAT_625_5000;

        case core::video_format::ntsc:
            return NTV2_FORMAT_525_5994;

        case core::video_format::x720p2398:
            return NTV2_FORMAT_720p_2398;

        case core::video_format::x720p2500:
            return NTV2_FORMAT_720p_2500;

        case core::video_format::x720p5000:
            return NTV2_FORMAT_720p_5000;

        case core::video_format::x720p5994:
            return NTV2_FORMAT_720p_5994;

        case core::video_format::x720p6000:
            return NTV2_FORMAT_720p_6000;

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

        case core::video_format::x2160p2398:
            return NTV2_FORMAT_4x1920x1080p_2398;

        case core::video_format::x2160p2400:
            return NTV2_FORMAT_4x1920x1080p_2400;

        case core::video_format::x2160p2500:
            return NTV2_FORMAT_4x1920x1080p_2500;

        case core::video_format::x2160p2997:
            return NTV2_FORMAT_4x1920x1080p_2997;

        case core::video_format::x2160p3000:
            return NTV2_FORMAT_4x1920x1080p_3000;

        case core::video_format::x2160p5000:
            return NTV2_FORMAT_4x1920x1080p_5000;

        case core::video_format::x2160p5994:
            return NTV2_FORMAT_4x1920x1080p_5994;

        case core::video_format::x2160p6000:
            return NTV2_FORMAT_4x1920x1080p_6000;

        default:
            return NTV2_FORMAT_UNKNOWN;
    }
}

class aja_consumer final : public core::frame_consumer
{
    CNTV2Card               device_;
    core::video_format_desc format_desc_;
    int                     channel_index_ = 0;

    std::vector<uint8_t>      video_buffer_;
    std::vector<uint8_t>      key_buffer_;
    std::vector<std::int32_t> audio_buffer_;

    NTV2AudioSystem audio_system_ = NTV2_AUDIOSYSTEM_1;

    ULWord          device_index_ = 0;
    NTV2Channel     channel_      = NTV2_CHANNEL1;
    NTV2Channel     key_channel_  = NTV2_CHANNEL_INVALID;
    NTV2VideoFormat video_format_ = NTV2_FORMAT_UNKNOWN;

    bool key_enabled_ = false;

    bool   initialized_                = false;
    bool   auto_circulate_started_     = false;
    ULWord successful_frame_transfers_ = 0;

    LWord fill_start_frame_ = -1;
    LWord fill_end_frame_   = -1;
    LWord next_fill_frame_  = -1;

  public:
    aja_consumer(ULWord device_index, NTV2Channel channel, NTV2Channel key_channel = NTV2_CHANNEL_INVALID)
        : device_index_(device_index)
        , channel_(channel)
        , key_channel_(key_channel)
        , key_enabled_(key_channel != NTV2_CHANNEL_INVALID)
    {
    }

    ~aja_consumer() override
    {
        try {
            if (auto_circulate_started_) {
                device_.AutoCirculateStop(channel_);
                auto_circulate_started_ = false;
            }

            device_.DisableChannel(channel_);

            if (key_enabled_)
                device_.DisableChannel(key_channel_);
        } catch (...) {
        }
    }

    void initialize(const core::video_format_desc& format_desc,
                    const core::channel_info&      channel_info,
                    int                            port_index) override
    {
        format_desc_   = format_desc;
        channel_index_ = channel_info.index;

        video_format_ = get_aja_video_format(format_desc.format);

        const bool is_uhd     = format_desc.width == 3840 && format_desc.height == 2160;
        const bool is_uhd_hfr = is_uhd && format_desc.fps > 30.0;

        if (video_format_ == NTV2_FORMAT_UNKNOWN) {
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Unsupported CasparCG video format for AJA output"));
        }

        const std::size_t frame_buffer_size =
            static_cast<std::size_t>(format_desc.width) * static_cast<std::size_t>(format_desc.height) * 2u;

        video_buffer_.resize(frame_buffer_size);

        if (key_enabled_)
            key_buffer_.resize(frame_buffer_size);

        CASPAR_LOG(info) << L"AJA consumer initializing for Caspar channel " << channel_index_ << L", AJA device "
                         << (device_index_ + 1) << L", output channel " << (static_cast<int>(channel_) + 1);

        CNTV2DeviceScanner scanner(true);

        if (!CNTV2DeviceScanner::GetDeviceAtIndex(device_index_, device_)) {
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Unable to open selected AJA device"));
        }

        if (!device_.features().CanDoChannel(channel_)) {
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Selected AJA device does not support requested channel"));
        }

        if (key_enabled_) {
            if (is_uhd) {
                CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA fill/key output does not currently support UHD"));
            }

            if (static_cast<int>(key_channel_) != static_cast<int>(channel_) + 1) {
                CASPAR_THROW_EXCEPTION(user_error()
                                       << msg_info("AJA key channel must immediately follow fill channel"));
            }

            if (!device_.features().CanDoChannel(key_channel_)) {
                CASPAR_THROW_EXCEPTION(user_error()
                                       << msg_info("Selected AJA device does not support requested key channel"));
            }
        }

        if (is_uhd && channel_ != NTV2_CHANNEL1) {
            CASPAR_THROW_EXCEPTION(user_error()
                                   << msg_info("Initial AJA UHD TSI implementation supports output channel 1 only"));
        }

        if (is_uhd && !device_.features().CanDoChannel(NTV2_CHANNEL2)) {
            CASPAR_THROW_EXCEPTION(
                user_error() << msg_info(
                    "Selected AJA device does not provide the second channel required for UHD TSI output"));
        }

        if (!device_.features().CanDoVideoFormat(video_format_)) {
            CASPAR_THROW_EXCEPTION(user_error()
                                   << msg_info("Selected AJA device does not support requested video format"));
        }

        device_.SetEveryFrameServices(NTV2_OEM_TASKS);

        if (is_uhd) {
            //
            // UHD TSI on the original Corvid44 uses two FrameStores.
            // Initial implementation is intentionally restricted to channel 1.
            //
            NTV2ChannelSet frame_stores = ::NTV2MakeChannelSet(NTV2_CHANNEL1, 2);

            if (!device_.EnableChannels(frame_stores)) {
                CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to enable AJA UHD FrameStores"));
            }

            device_.SetMode(frame_stores, NTV2_MODE_DISPLAY);

            device_.SetVANCMode(frame_stores, NTV2_VANCMODE_OFF);

            device_.SetVANCShiftMode(frame_stores, NTV2_VANCDATA_NORMAL);

            device_.SetReference(NTV2_REFERENCE_FREERUN);

            if (!device_.SetVideoFormat(frame_stores, video_format_, false)) {
                CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to set AJA UHD video format"));
            }

            if (!device_.SetFrameBufferFormat(frame_stores, kPixelFormat)) {
                CASPAR_THROW_EXCEPTION(caspar_exception()
                                       << msg_info("Unable to set AJA UHD framebuffer format to 8-bit YCbCr"));
            }
        } else {
            if (!device_.EnableChannel(channel_)) {
                CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to enable selected AJA channel"));
            }

            device_.SetMode(channel_, NTV2_MODE_DISPLAY);

            device_.SetVANCMode(NTV2_VANCMODE_OFF, channel_);

            device_.SetVANCShiftMode(channel_, NTV2_VANCDATA_NORMAL);

            device_.SetReference(NTV2_REFERENCE_FREERUN);

            if (!device_.SetVideoFormat(video_format_, false, false, channel_)) {
                CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to set selected AJA video format"));
            }

            if (!device_.SetFrameBufferFormat(channel_, kPixelFormat)) {
                CASPAR_THROW_EXCEPTION(caspar_exception()
                                       << msg_info("Unable to set AJA framebuffer format to 8-bit YCbCr"));
            }

            if (key_enabled_) {
                if (!device_.EnableChannel(key_channel_)) {
                    CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to enable AJA key channel"));
                }

                device_.SetMode(key_channel_, NTV2_MODE_DISPLAY);

                device_.SetVANCMode(NTV2_VANCMODE_OFF, key_channel_);

                device_.SetVANCShiftMode(key_channel_, NTV2_VANCDATA_NORMAL);

                if (!device_.SetVideoFormat(video_format_, false, false, key_channel_)) {
                    CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to set AJA key video format"));
                }

                if (!device_.SetFrameBufferFormat(key_channel_, kPixelFormat)) {
                    CASPAR_THROW_EXCEPTION(caspar_exception()
                                           << msg_info("Unable to set AJA key framebuffer format to 8-bit YCbCr"));
                }
            }
        }

        //
        // AJA SDI output setup and routing.
        // Mirrors the known-good NTV2Player output path.
        //

        const NTV2Standard video_std = GetNTV2StandardFromVideoFormat(video_format_);

        device_.SetSDIOutputStandard(channel_, video_std);

        device_.SetSDIOutLevelAtoLevelBConversion(channel_, false);

        device_.SetSDIOutRGBLevelAConversion(channel_, false);

        device_.SetSDITransmitEnable(channel_, true);

        if (key_enabled_) {
            device_.SetSDIOutputStandard(key_channel_, video_std);

            device_.SetSDIOutLevelAtoLevelBConversion(key_channel_, false);

            device_.SetSDIOutRGBLevelAConversion(key_channel_, false);

            device_.SetSDITransmitEnable(key_channel_, true);
        }

        NTV2XptConnections connections;

        if (is_uhd) {
            //
            // UHD TSI routing for original Corvid44, channel 1.
            //
            // Two UHD FrameStores feed four 4:2:5 mux inputs.  The mux
            // outputs are carried as DS1/DS2 on SDI outputs 1 and 2.
            //
            connections.insert(NTV2XptConnection(NTV2_Xpt425Mux1AInput, NTV2_XptFrameBuffer1YUV));

            connections.insert(NTV2XptConnection(NTV2_Xpt425Mux1BInput, NTV2_XptFrameBuffer1_DS2YUV));

            connections.insert(NTV2XptConnection(NTV2_Xpt425Mux2AInput, NTV2_XptFrameBuffer2YUV));

            connections.insert(NTV2XptConnection(NTV2_Xpt425Mux2BInput, NTV2_XptFrameBuffer2_DS2YUV));

            if (is_uhd_hfr) {
                //
                // High-frame-rate UHD TSI:
                // four mux components are carried on four SDI outputs.
                //
                connections.insert(NTV2XptConnection(NTV2_XptSDIOut1Input, NTV2_Xpt425Mux1AYUV));

                connections.insert(NTV2XptConnection(NTV2_XptSDIOut2Input, NTV2_Xpt425Mux1BYUV));

                connections.insert(NTV2XptConnection(NTV2_XptSDIOut3Input, NTV2_Xpt425Mux2AYUV));

                connections.insert(NTV2XptConnection(NTV2_XptSDIOut4Input, NTV2_Xpt425Mux2BYUV));
            } else {
                //
                // Low-frame-rate UHD TSI:
                // two SDI outputs, each carrying DS1 + DS2.
                //
                connections.insert(NTV2XptConnection(NTV2_XptSDIOut1Input, NTV2_Xpt425Mux1AYUV));

                connections.insert(NTV2XptConnection(NTV2_XptSDIOut1InputDS2, NTV2_Xpt425Mux1BYUV));

                connections.insert(NTV2XptConnection(NTV2_XptSDIOut2Input, NTV2_Xpt425Mux2AYUV));

                connections.insert(NTV2XptConnection(NTV2_XptSDIOut2InputDS2, NTV2_Xpt425Mux2BYUV));
            }
        } else {
            const NTV2OutputXptID source_xpt = GetFrameStoreOutputXptFromChannel(channel_, false);

            connections.insert(NTV2XptConnection(GetSDIOutputInputXpt(channel_), source_xpt));

            if (key_enabled_) {
                const NTV2OutputXptID key_source_xpt = GetFrameStoreOutputXptFromChannel(key_channel_, false);

                connections.insert(NTV2XptConnection(GetSDIOutputInputXpt(key_channel_), key_source_xpt));
            }
        }

        if (!device_.ApplySignalRoute(connections, false)) {
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to apply AJA output routing"));
        }

        if (is_uhd) {
            device_.SetTsiFrameEnable(true, NTV2_CHANNEL1);

            device_.SetSDITransmitEnable(NTV2_CHANNEL1, true);
            device_.SetSDITransmitEnable(NTV2_CHANNEL2, true);

            if (is_uhd_hfr) {
                device_.SetSDITransmitEnable(NTV2_CHANNEL3, true);
                device_.SetSDITransmitEnable(NTV2_CHANNEL4, true);
            }
        }

        device_.AutoCirculateStop(channel_);
        device_.WaitForOutputVerticalInterrupt(channel_, 4);

        audio_system_ = NTV2_AUDIOSYSTEM_1;

        if (device_.features().GetNumAudioSystems() > 1)
            audio_system_ = NTV2ChannelToAudioSystem(channel_);

        if (!device_.features().CanDoFrameStore1Display())
            audio_system_ = NTV2_AUDIOSYSTEM_1;

        ULWord num_audio_channels = device_.features().GetMaxAudioChannels();

        if (num_audio_channels > 8 && !device_.features().CanDo2110() && NTV2_IS_2K_1080_VIDEO_FORMAT(video_format_)) {
            num_audio_channels = 8;
        }

        device_.SetNumberAudioChannels(num_audio_channels, audio_system_);

        device_.SetAudioRate(NTV2_AUDIO_48K, audio_system_);

        device_.SetAudioBufferSize(NTV2_AUDIO_BUFFER_BIG, audio_system_);

        if (is_uhd) {
            const NTV2ChannelSet audio_spigots = ::NTV2MakeChannelSet(NTV2_CHANNEL1, 4);

            device_.SetSDIOutputAudioSystem(audio_spigots, audio_system_);
        } else {
            device_.SetSDIOutputAudioSystem(channel_, audio_system_);
            device_.SetSDIOutputAudioEnabled(channel_, true);

            if (key_enabled_)
                device_.SetSDIOutputAudioEnabled(key_channel_, false);
        }

        device_.SetSDIOutputDS2AudioSystem(channel_, audio_system_);

        device_.SetAudioLoopBack(NTV2_AUDIO_LOOPBACK_OFF, audio_system_);

        const UByte auto_circulate_channels = key_enabled_ ? 2 : 1;

        if (!device_.AutoCirculateInitForOutput(
                channel_, 7, audio_system_, AUTOCIRCULATE_WITH_RP188, auto_circulate_channels)) {
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to initialize AJA AutoCirculate output"));
        }

        AUTOCIRCULATE_STATUS ac_status;

        if (!device_.AutoCirculateGetStatus(channel_, ac_status)) {
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to query AJA AutoCirculate status"));
        }

        fill_start_frame_ = ac_status.GetStartFrame();
        fill_end_frame_   = ac_status.GetEndFrame();
        next_fill_frame_  = fill_start_frame_;

        CASPAR_LOG(info) << L"AJA AutoCirculate ring: " << fill_start_frame_ << L"-" << fill_end_frame_;

        auto_circulate_started_ = false;
        initialized_            = true;

        CASPAR_LOG(info) << L"AJA consumer initialized: device " << (device_index_ + 1) << L", channel "
                         << (static_cast<int>(channel_) + 1) << L", " << format_desc_.name
                         << L", 8-bit YCbCr, AJA audio system configured";
    }

    std::future<bool> send(core::video_field field, core::const_frame frame) override
    {
        if (!initialized_ || !frame)
            return caspar::make_ready_future(false);

        try {
            const auto& desc = frame.pixel_format_desc();

            if (desc.format != core::pixel_format::bgra) {
                CASPAR_LOG(error) << L"AJA consumer received unsupported pixel format " << static_cast<int>(desc.format)
                                  << L"; initial implementation expects BGRA";

                return caspar::make_ready_future(false);
            }

            const auto& image = frame.image_data(0);

            const std::size_t expected =
                static_cast<std::size_t>(format_desc_.width) * static_cast<std::size_t>(format_desc_.height) * 4u;

            if (image.size() < expected) {
                CASPAR_LOG(error) << L"AJA consumer received undersized BGRA frame: " << image.size()
                                  << L" bytes, expected at least " << expected;

                return caspar::make_ready_future(false);
            }

            const bool interlaced = format_desc_.field_count == 2;

            const auto& audio = frame.audio_data();

            if (interlaced) {
                const int first_line = field == core::video_field::a ? 0 : 1;

                bgra_field_to_interlaced_uyvy(
                    image.data(), video_buffer_.data(), format_desc_.width, format_desc_.height, first_line);

                if (key_enabled_) {
                    bgra_field_to_interlaced_key_uyvy(
                        image.data(), key_buffer_.data(), format_desc_.width, format_desc_.height, first_line);
                }

                if (field == core::video_field::a)
                    audio_buffer_.clear();

                audio_buffer_.insert(audio_buffer_.end(), audio.begin(), audio.end());

                if (field == core::video_field::a)
                    return caspar::make_ready_future(true);
            } else {
                bgra_to_uyvy(image.data(), video_buffer_.data(), format_desc_.width, format_desc_.height);

                if (key_enabled_) {
                    bgra_to_key_uyvy(image.data(), key_buffer_.data(), format_desc_.width, format_desc_.height);
                }

                audio_buffer_.assign(audio.begin(), audio.end());
            }

            AUTOCIRCULATE_STATUS status;

            device_.AutoCirculateGetStatus(channel_, status);

            while (!status.CanAcceptMoreOutputFrames()) {
                device_.WaitForOutputVerticalInterrupt(channel_);
                device_.AutoCirculateGetStatus(channel_, status);
            }

            AUTOCIRCULATE_TRANSFER transfer;

            transfer.SetVideoBuffer(reinterpret_cast<ULWord*>(video_buffer_.data()),
                                    static_cast<ULWord>(video_buffer_.size()));

            if (!audio_buffer_.empty()) {
                transfer.SetAudioBuffer(reinterpret_cast<ULWord*>(audio_buffer_.data()),
                                        static_cast<ULWord>(audio_buffer_.size() * sizeof(std::int32_t)));
            }

            if (key_enabled_) {
                const LWord stride = fill_end_frame_ - fill_start_frame_ + 1;

                const LWord key_device_frame = next_fill_frame_ + stride;

                if (!device_.DMAWriteFrame(static_cast<ULWord>(key_device_frame),
                                           reinterpret_cast<const ULWord*>(key_buffer_.data()),
                                           static_cast<ULWord>(key_buffer_.size()))) {
                    CASPAR_LOG(error) << L"AJA key DMAWriteFrame failed for device frame " << key_device_frame;

                    return caspar::make_ready_future(false);
                }

                transfer.acDesiredFrame = next_fill_frame_;
            }

            if (!device_.AutoCirculateTransfer(channel_, transfer)) {
                CASPAR_LOG(error) << L"AJA AutoCirculateTransfer failed";
                return caspar::make_ready_future(false);
            }

            if (key_enabled_) {
                if (next_fill_frame_ >= fill_end_frame_)
                    next_fill_frame_ = fill_start_frame_;
                else
                    ++next_fill_frame_;
            }

            ++successful_frame_transfers_;

            if (!auto_circulate_started_ && successful_frame_transfers_ >= 5) {
                if (!device_.AutoCirculateStart(channel_)) {
                    CASPAR_LOG(error) << L"Unable to start AJA AutoCirculate frame output";

                    return caspar::make_ready_future(false);
                }

                auto_circulate_started_ = true;

                CASPAR_LOG(info) << L"AJA AutoCirculate frame output started after " << successful_frame_transfers_
                                 << L" buffered frames";
            }

            return caspar::make_ready_future(true);
        } catch (...) {
            CASPAR_LOG_CURRENT_EXCEPTION();
            return caspar::make_ready_future(false);
        }
    }

    core::monitor::state state() const override { return {}; }

    std::wstring print() const override
    {
        std::wstringstream ss;
        ss << L"AJA [" << (device_index_ + 1) << L"|" << (static_cast<int>(channel_) + 1) << L"|" << format_desc_.name
           << L"]";
        return ss.str();
    }

    std::wstring name() const override { return L"aja"; }

    int index() const override { return 400; }

    bool has_synchronization_clock() const override { return true; }
};

} // namespace

spl::shared_ptr<core::frame_consumer> create_consumer(const std::vector<std::wstring>&     params,
                                                      const core::video_format_repository& format_repository,
                                                      const std::vector<spl::shared_ptr<core::video_channel>>& channels,
                                                      const core::channel_info& channel_info)
{
    if (params.empty() || !boost::iequals(params.at(0), L"AJA"))
        return core::frame_consumer::empty();

    return spl::make_shared<aja_consumer>(0, NTV2_CHANNEL1);
}

spl::shared_ptr<core::frame_consumer>
create_preconfigured_consumer(const boost::property_tree::wptree&                      ptree,
                              const core::video_format_repository&                     format_repository,
                              const std::vector<spl::shared_ptr<core::video_channel>>& channels,
                              const core::channel_info&                                channel_info)
{
    const int device      = ptree.get<int>(L"device", 1);
    const int channel     = ptree.get<int>(L"channel", 1);
    const int key_channel = ptree.get<int>(L"key-channel", 0);

    if (device < 1) {
        CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA device must be >= 1"));
    }

    if (channel < 1) {
        CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA channel must be >= 1"));
    }

    if (key_channel < 0) {
        CASPAR_THROW_EXCEPTION(user_error() << msg_info("AJA key channel must be >= 1 when specified"));
    }

    const NTV2Channel aja_key_channel =
        key_channel > 0 ? static_cast<NTV2Channel>(key_channel - 1) : NTV2_CHANNEL_INVALID;

    return spl::make_shared<aja_consumer>(
        static_cast<ULWord>(device - 1), static_cast<NTV2Channel>(channel - 1), aja_key_channel);
}

}} // namespace caspar::aja
