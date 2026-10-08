#include <core/frame/frame.h>
#include <core/frame/geometry.h>
#include <core/frame/pixel_format.h>
#include <core/producer/route/fix_stream_tag.h>
#include <common/except.h>

#include <iostream>
#include <stdexcept>

using namespace caspar;
using namespace caspar::core;

namespace {
void require(bool value, const char* message)
{
    if (!value)
        throw std::runtime_error(message);
}

struct gpu_storage {
    std::shared_ptr<int> resource;
    std::shared_ptr<bool> throw_on_copy;
    gpu_storage(std::shared_ptr<int> resource, std::shared_ptr<bool> throw_on_copy)
        : resource(std::move(resource)), throw_on_copy(std::move(throw_on_copy)) {}
    gpu_storage(const gpu_storage& other)
        : resource(other.resource), throw_on_copy(other.throw_on_copy)
    {
        if (*throw_on_copy)
            throw std::runtime_error("injected opaque storage copy failure");
    }
};

struct texture_resource : texture {
    void bind(int) override {}
    void unbind() override {}
};

struct collect_frames : frame_visitor {
    std::vector<const_frame> frames;
    std::vector<frame_transform> transforms;
    std::vector<frame_transform> stack;
    void push(const frame_transform& transform) override { stack.push_back(transform); }
    void visit(const const_frame& frame) override
    {
        frames.push_back(frame);
        transforms.push_back(stack.back());
    }
    void pop() override { stack.pop_back(); }
};

template <typename Action>
void expect_invalid(Action action)
{
    try {
        action();
    } catch (const caspar::invalid_argument&) {
        return;
    }
    throw std::runtime_error("invalid storage was accepted");
}
} // namespace

int main()
{
    try {
        int source_tag = 0, route_tag = 0, next_tag = 0;
        pixel_format_desc desc(pixel_format::bgra, color_space::bt2020);
        desc.is_straight_alpha = true;
        desc.planes.emplace_back(2, 2, 4);

        // Match the CEF D3D import: one plane, no CPU buffers, GPU ownership in opaque_.
        auto resource = std::make_shared<int>(42);
        auto fail_copy = std::make_shared<bool>(false);
        int commits = 0;
        mutable_frame gpu_mutable(&source_tag, {}, array<int32_t>(std::vector<int32_t>{12, -34}), desc,
            [&](std::vector<array<const uint8_t>> buffers) -> std::any {
                require(buffers.empty(), "GPU frame unexpectedly has CPU buffers");
                ++commits;
                return gpu_storage(resource, fail_copy);
            });
        gpu_mutable.geometry() = frame_geometry::get_default_vflip(frame_geometry::scale_mode::fit);
        const_frame gpu(std::move(gpu_mutable));
        auto routed = gpu.with_tag(&route_tag);
        auto next = routed.with_tag(&next_tag);
        require(gpu.stream_tag() == &source_tag && routed.stream_tag() == &route_tag &&
                    next.stream_tag() == &next_tag, "retagging mutated another frame");
        require(commits == 1, "retagging recommitted GPU storage");
        require(std::any_cast<const gpu_storage&>(next.opaque()).resource == resource,
                "GPU ownership was not preserved");
        require(next.pixel_format_desc().planes.size() == 1 && next.width() == 2 && next.height() == 2 &&
                    next.pixel_format_desc().color_space == color_space::bt2020 &&
                    next.pixel_format_desc().is_straight_alpha, "pixel metadata changed");
        require(next.audio_data().data() == gpu.audio_data().data() && next.audio_data().data()[1] == -34,
                "audio storage changed");
        require(next.geometry().mode() == frame_geometry::scale_mode::fit &&
                    next.geometry().data() == gpu.geometry().data(), "geometry changed");
        try {
            next.image_data(0);
            throw std::runtime_error("GPU-only frame acquired a CPU buffer");
        } catch (const std::out_of_range&) {}
        std::cout << "PASS GPU-only retagging, chained tags, metadata, audio and ownership\n";

        // CPU frames still share image storage and preserve the explicit texture handle.
        auto tex = std::make_shared<texture_resource>();
        std::vector<array<const uint8_t>> cpu_buffers;
        cpu_buffers.emplace_back(std::vector<uint8_t>(16, 7));
        const_frame cpu(&source_tag, std::move(cpu_buffers), {}, desc, tex);
        auto cpu_route = cpu.with_tag(&route_tag);
        require(cpu_route.image_data(0).data() == cpu.image_data(0).data() &&
                    cpu_route.image_data(0).data()[0] == 7 && cpu_route.texture() == tex &&
                    !cpu_route.opaque().has_value(), "CPU or explicit texture storage changed");
        require(!const_frame{}.with_tag(&route_tag), "empty frame became valid");
        const_frame audio(mutable_frame(&source_tag, {}, array<int32_t>(std::vector<int32_t>{56}),
                                        pixel_format_desc{}));
        require(audio.with_tag(&route_tag).audio_data().data()[0] == 56, "audio-only retagging failed");
        std::cout << "PASS CPU, explicit texture, empty and audio-only frames\n";

        expect_invalid([&] { const_frame invalid(&source_tag, {}, {}, desc); });
        expect_invalid([&] { const_frame invalid(mutable_frame(&source_tag, {}, {}, desc)); });
        std::cout << "PASS constructor validation remains enforced\n";

        draw_frame video(gpu);
        video.transform().image_transform.opacity = 0.25;
        draw_frame audio_draw(audio);
        audio_draw.transform().audio_transform.volume = 0.75;
        auto combined = draw_frame::push(draw_frame::over(video, audio_draw));
        fix_stream_tag retag(&route_tag);
        auto result = retag(combined);
        collect_frames collected;
        result.accept(collected);
        require(collected.frames.size() == 2, "nested route lost video or audio");
        require(collected.frames[0].stream_tag() == collected.frames[1].stream_tag() &&
                    collected.frames[0].stream_tag() != &source_tag, "route tags were not consistent");
        require(collected.transforms[0].image_transform.opacity == 0.25 &&
                    collected.transforms[1].audio_transform.volume == 0.75, "route transforms changed");
        require(std::any_cast<const gpu_storage&>(collected.frames[0].opaque()).resource == resource,
                "nested route lost GPU ownership");
        collect_frames repeated;
        retag(combined).accept(repeated);
        require(repeated.frames[0].stream_tag() == collected.frames[0].stream_tag(), "route tag was unstable");
        std::cout << "PASS nested video/audio route, transforms and stable tags\n";

        // Throw after a preceding child has already been processed, then reuse the visitor.
        *fail_copy = true;
        bool caught = false;
        try {
            retag(draw_frame::over(audio_draw, video));
        } catch (const std::runtime_error&) {
            caught = true;
        }
        require(caught, "fault injection did not throw");
        *fail_copy = false;
        collect_frames recovered;
        retag(combined).accept(recovered);
        require(recovered.frames.size() == 2 &&
                    recovered.frames[0].stream_tag() == collected.frames[0].stream_tag(),
                "visitor retained incomplete traversal state");
        require(gpu.stream_tag() == &source_tag && commits == 1, "failure changed the source frame");
        std::cout << "PASS visitor recovery after injected storage-copy failure\n";
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
