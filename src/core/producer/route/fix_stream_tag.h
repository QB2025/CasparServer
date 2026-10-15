/*
 * Copyright (c) 2011 Sveriges Television AB <info@casparcg.com>
 *
 * This file is part of CasparCG (www.casparcg.com).
 *
 * CasparCG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CasparCG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CasparCG. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Robert Nagy, ronag89@gmail.com
 */

#pragma once

#include <core/frame/draw_frame.h>
#include <core/frame/frame.h>
#include <core/frame/frame_transform.h>
#include <core/frame/frame_visitor.h>

#include <cstdint>
#include <optional>
#include <stack>
#include <utility>
#include <vector>

namespace caspar { namespace core {

class fix_stream_tag : public frame_visitor
{
    const void*                                                     route_producer_ptr_;
    std::stack<std::pair<frame_transform, std::vector<draw_frame>>> frames_stack_;
    std::optional<const_frame>                                      upd_frame_;

    fix_stream_tag(const fix_stream_tag&);
    fix_stream_tag& operator=(const fix_stream_tag&);

  public:
    fix_stream_tag(void* stream_tag)
        : route_producer_ptr_(stream_tag)
    {
        frames_stack_ = std::stack<std::pair<frame_transform, std::vector<draw_frame>>>();
        frames_stack_.emplace(frame_transform{}, std::vector<draw_frame>());
    }

    void push(const frame_transform& transform) { frames_stack_.emplace(transform, std::vector<core::draw_frame>()); }

    void visit(const const_frame& frame)
    {
        // Get original tag from the frame
        const void* source_tag = frame.stream_tag();

        // Calculate a unique but stable tag for this source
        // This calculation will always produce the same result for the same inputs
        intptr_t base_addr   = reinterpret_cast<intptr_t>(route_producer_ptr_);
        intptr_t source_addr = reinterpret_cast<intptr_t>(source_tag);
        // Use XOR to create a unique value that combines route producer and source identities
        intptr_t    unique_value = base_addr ^ source_addr ^ 0xDEADBEEF; // Constant helps avoid collisions
        const void* unique_tag   = reinterpret_cast<const void*>(unique_value);

        // Apply the tag to the frame
        upd_frame_ = frame.with_tag(unique_tag);
    }

    void pop()
    {
        auto popped = frames_stack_.top();
        frames_stack_.pop();

        if (upd_frame_ != std::nullopt) {
            auto new_frame        = draw_frame(std::move(*upd_frame_));
            upd_frame_            = std::nullopt;
            new_frame.transform() = popped.first;
            frames_stack_.top().second.push_back(std::move(new_frame));
        } else {
            auto new_frame        = draw_frame(std::move(popped.second));
            new_frame.transform() = popped.first;
            frames_stack_.top().second.push_back(new_frame);
        }
    }

    draw_frame operator()(draw_frame frame)
    {
        // A previous traversal may have thrown while copying frame storage.
        frames_stack_ = std::stack<std::pair<frame_transform, std::vector<draw_frame>>>();
        frames_stack_.emplace(frame_transform{}, std::vector<draw_frame>());
        upd_frame_ = std::nullopt;

        frame.accept(*this);

        auto popped = frames_stack_.top();
        frames_stack_.pop();
        draw_frame result = draw_frame(std::move(popped.second));

        return result;
    }
};

}} // namespace caspar::core
