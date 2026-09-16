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

constexpr ULWord                kDeviceIndex                = 0;
constexpr NTV2Channel           kChannel                    = NTV2_CHANNEL1;
constexpr NTV2VideoFormat       kVideoFormat                = NTV2_FORMAT_1080i_5000;
constexpr NTV2FrameBufferFormat kPixelFormat                = NTV2_FBF_8BIT_YCBCR;
ULWord                          successful_frame_transfers_ = 0;

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

class aja_consumer final : public core::frame_consumer
{
    CNTV2Card               device_;
    core::video_format_desc format_desc_;
    int                     channel_index_ = 0;

    std::vector<uint8_t> video_buffer_;

    bool initialized_            = false;
    bool auto_circulate_started_ = false;

  public:
    aja_consumer() = default;

    ~aja_consumer() override
    {
        try {
            if (auto_circulate_started_) {
                device_.AutoCirculateStop(kChannel);
                auto_circulate_started_ = false;
            }

            device_.DisableChannel(kChannel);
        } catch (...) {
        }
    }

    void initialize(const core::video_format_desc& format_desc,
                    const core::channel_info&      channel_info,
                    int                            port_index) override
    {
        format_desc_   = format_desc;
        channel_index_ = channel_info.index;

        const std::size_t frame_buffer_size = 1920u * 1080u * 2u;

        video_buffer_.resize(frame_buffer_size);

        CASPAR_LOG(info) << L"AJA consumer initializing for Caspar channel " << channel_index_;

        if (format_desc.width != 1920 || format_desc.height != 1080 ||
            format_desc.format != core::video_format::x1080i5000) {
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Initial AJA consumer supports only 1080i5000"));
        }

        CNTV2DeviceScanner scanner(true);

        if (!CNTV2DeviceScanner::GetDeviceAtIndex(kDeviceIndex, device_)) {
            CASPAR_THROW_EXCEPTION(user_error() << msg_info("Unable to open AJA device 0"));
        }

        device_.SetEveryFrameServices(NTV2_OEM_TASKS);

        if (!device_.EnableChannel(kChannel)) {
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to enable AJA channel 1"));
        }

        device_.SetMode(kChannel, NTV2_MODE_DISPLAY);

        device_.SetVANCMode(NTV2_VANCMODE_OFF, kChannel);

        device_.SetVANCShiftMode(kChannel, NTV2_VANCDATA_NORMAL);

        device_.SetReference(NTV2_REFERENCE_FREERUN);

        if (!device_.SetVideoFormat(kVideoFormat, false, false, kChannel)) {
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to set AJA video format to 1080i50"));
        }

        if (!device_.SetFrameBufferFormat(kChannel, kPixelFormat)) {
            CASPAR_THROW_EXCEPTION(caspar_exception()
                                   << msg_info("Unable to set AJA framebuffer format to 8-bit YCbCr"));
        }

        //
        // Corvid 44 SDI output setup and routing.
        // Mirrors the known-good NTV2Player path for
        // channel 1, YCbCr, 1080i50.
        //

        const NTV2Standard video_std = GetNTV2StandardFromVideoFormat(kVideoFormat);

        device_.SetSDIOutputStandard(kChannel, video_std);

        device_.SetSDIOutLevelAtoLevelBConversion(kChannel, false);

        device_.SetSDIOutRGBLevelAConversion(kChannel, false);

        device_.SetSDITransmitEnable(kChannel, true);

        NTV2XptConnections connections;

        const NTV2OutputXptID source_xpt = GetFrameStoreOutputXptFromChannel(kChannel,
                                                                             false); // YCbCr

        connections.insert(NTV2XptConnection(GetSDIOutputInputXpt(kChannel), source_xpt));

        if (!device_.ApplySignalRoute(connections, true)) {
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to apply AJA SDI routing"));
        }

        device_.AutoCirculateStop(kChannel);
        device_.WaitForOutputVerticalInterrupt(kChannel, 4);

        if (!device_.AutoCirculateInitForOutput(kChannel, 7, NTV2_AUDIOSYSTEM_INVALID, AUTOCIRCULATE_WITH_RP188)) {
            CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("Unable to initialize AJA AutoCirculate output"));
        }

        auto_circulate_started_ = false;
        initialized_            = true;

        CASPAR_LOG(info) << L"AJA consumer initialized: device 0, channel 1, " << L"1080i50, 8-bit YCbCr, video only";
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

            const std::size_t expected = static_cast<std::size_t>(1920) * static_cast<std::size_t>(1080) * 4u;

            if (image.size() < expected) {
                CASPAR_LOG(error) << L"AJA consumer received undersized BGRA frame: " << image.size()
                                  << L" bytes, expected at least " << expected;

                return caspar::make_ready_future(false);
            }

            const int first_line = field == core::video_field::a ? 0 : 1;

            bgra_field_to_interlaced_uyvy(image.data(), video_buffer_.data(), 1920, 1080, first_line);

            if (field == core::video_field::a)
                return caspar::make_ready_future(true);

            AUTOCIRCULATE_STATUS status;
            device_.AutoCirculateGetStatus(kChannel, status);

            while (!status.CanAcceptMoreOutputFrames()) {
                device_.WaitForOutputVerticalInterrupt(kChannel);
                device_.AutoCirculateGetStatus(kChannel, status);
            }

            AUTOCIRCULATE_TRANSFER transfer;

            transfer.SetVideoBuffer(reinterpret_cast<ULWord*>(video_buffer_.data()),
                                    static_cast<ULWord>(video_buffer_.size()));

            if (!device_.AutoCirculateTransfer(kChannel, transfer)) {
                CASPAR_LOG(error) << L"AJA AutoCirculateTransfer failed";
                return caspar::make_ready_future(false);
            }

            ++successful_frame_transfers_;

            if (!auto_circulate_started_ && successful_frame_transfers_ >= 3) {
                if (!device_.AutoCirculateStart(kChannel)) {
                    CASPAR_LOG(error) << L"Unable to start AJA AutoCirculate frame output";

                    return caspar::make_ready_future(true);
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

    std::wstring print() const override { return L"AJA [0|1|1080i5000]"; }

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

    return spl::make_shared<aja_consumer>();
}

spl::shared_ptr<core::frame_consumer>
create_preconfigured_consumer(const boost::property_tree::wptree&                      ptree,
                              const core::video_format_repository&                     format_repository,
                              const std::vector<spl::shared_ptr<core::video_channel>>& channels,
                              const core::channel_info&                                channel_info)
{
    return spl::make_shared<aja_consumer>();
}

}} // namespace caspar::aja
