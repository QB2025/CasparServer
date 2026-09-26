/*
 * Dedicated final output pass for converting CasparCG's premultiplied
 * composition to straight-alpha BGRA output.
 */
#pragma once

#include <common/memory.h>
#include <memory>

namespace caspar { namespace accelerator { namespace ogl {

class device;
class texture;

class straight_alpha_kernel final
{
    straight_alpha_kernel(const straight_alpha_kernel&) = delete;
    straight_alpha_kernel& operator=(const straight_alpha_kernel&) = delete;

  public:
    explicit straight_alpha_kernel(const spl::shared_ptr<device>& ogl);
    ~straight_alpha_kernel();

    void draw(const std::shared_ptr<texture>& target, const std::shared_ptr<texture>& source);

  private:
    struct impl;
    spl::unique_ptr<impl> impl_;
};

}}} // namespace caspar::accelerator::ogl
