/*
 * Dedicated final output pass for converting CasparCG's premultiplied
 * composition to straight-alpha BGRA output.
 */
#include "straight_alpha_kernel.h"

#include "../util/device.h"
#include "../util/shader.h"
#include "../util/texture.h"

#include <common/gl/gl_check.h>
#include <core/frame/geometry.h>

#include <GL/glew.h>
#include <string>

namespace caspar::accelerator::ogl {

namespace {

const std::string vertex_shader = R"GLSL(
#version 450
in vec4 TexCoordIn;
in vec2 Position;
out vec4 TexCoord;

void main()
{
    TexCoord = TexCoordIn;
    vec4 pos = vec4(Position, 0.0, 1.0);
    pos.x = pos.x * 2.0 - 1.0;
    pos.y = pos.y * 2.0 - 1.0;
    gl_Position = pos;
}
)GLSL";

const std::string fragment_shader = R"GLSL(
#version 450
uniform sampler2D source;
in vec4 TexCoord;
out vec4 fragColor;

void main()
{
    vec4 color = texture(source, TexCoord.st / TexCoord.q);

    if (color.a > 0.0000001)
        color.rgb /= color.a;
    else
        color.rgb = vec3(0.0);

    fragColor = color;
}
)GLSL";

std::shared_ptr<shader> create_straight_alpha_shader(const spl::shared_ptr<device>& ogl)
{
    // OpenGL resources must be destroyed on the device thread.
    std::weak_ptr<device> weak_ogl = ogl;

    auto deleter = [weak_ogl](shader* p) {
        auto ogl = weak_ogl.lock();

        if (ogl) {
            ogl->dispatch_async([=] { delete p; });
        }
    };

    return std::shared_ptr<shader>(new shader(vertex_shader, fragment_shader), deleter);
}

} // namespace

struct straight_alpha_kernel::impl
{
    spl::shared_ptr<device> ogl_;
    spl::shared_ptr<shader> shader_;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;

    explicit impl(const spl::shared_ptr<device>& ogl)
        : ogl_(ogl)
        , shader_(ogl_->dispatch_sync([&] { return create_straight_alpha_shader(ogl); }))
    {
        ogl_->dispatch_sync([this] {
            GL(glGenVertexArrays(1, &vao_));
            GL(glGenBuffers(1, &vbo_));
        });
    }

    ~impl()
    {
        ogl_->dispatch_sync([this] {
            GL(glDeleteVertexArrays(1, &vao_));
            GL(glDeleteBuffers(1, &vbo_));
        });
    }

    void draw(const std::shared_ptr<texture>& target, const std::shared_ptr<texture>& source)
    {
        if (!target || !source)
            return;

        const auto coords = core::frame_geometry::get_default().data();
        if (coords.empty())
            return;

        source->bind(0);
        shader_->use();
        shader_->set("source", 0);

        GL(glViewport(0, 0, target->width(), target->height()));
        GL(glDisable(GL_DEPTH_TEST));
        target->attach();

        GL(glBindVertexArray(vao_));
        GL(glBindBuffer(GL_ARRAY_BUFFER, vbo_));
        GL(glBufferData(GL_ARRAY_BUFFER,
                        static_cast<GLsizeiptr>(sizeof(core::frame_geometry::coord)) * coords.size(),
                        coords.data(),
                        GL_STATIC_DRAW));

        const auto stride = static_cast<GLsizei>(sizeof(core::frame_geometry::coord));
        const auto vtx_loc = shader_->get_attrib_location("Position");
        const auto tex_loc = shader_->get_attrib_location("TexCoordIn");

        GL(glEnableVertexAttribArray(vtx_loc));
        GL(glEnableVertexAttribArray(tex_loc));
        GL(glVertexAttribPointer(vtx_loc, 2, GL_DOUBLE, GL_FALSE, stride, nullptr));
        GL(glVertexAttribPointer(tex_loc, 4, GL_DOUBLE, GL_FALSE, stride, (GLvoid*)(2 * sizeof(GLdouble))));

        GL(glDrawArrays(GL_TRIANGLE_FAN, 0, static_cast<GLsizei>(coords.size())));
        GL(glTextureBarrier());

        GL(glDisableVertexAttribArray(vtx_loc));
        GL(glDisableVertexAttribArray(tex_loc));
        GL(glBindVertexArray(0));
        GL(glBindBuffer(GL_ARRAY_BUFFER, 0));
        GL(glDisable(GL_SCISSOR_TEST));
        GL(glDisable(GL_BLEND));
    }
};

straight_alpha_kernel::straight_alpha_kernel(const spl::shared_ptr<device>& ogl)
    : impl_(new impl(ogl))
{
}
straight_alpha_kernel::~straight_alpha_kernel() {}

void straight_alpha_kernel::draw(const std::shared_ptr<texture>& target,
                                 const std::shared_ptr<texture>& source)
{
    impl_->draw(target, source);
}

} // namespace caspar::accelerator::ogl
