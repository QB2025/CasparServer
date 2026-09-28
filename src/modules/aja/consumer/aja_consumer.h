#pragma once

#include <common/memory.h>
#include <core/fwd.h>

#include <boost/property_tree/ptree_fwd.hpp>

#include <string>
#include <vector>

namespace caspar { namespace aja {

spl::shared_ptr<core::frame_consumer>
create_consumer(const std::vector<std::wstring>& params,
                const core::video_format_repository& format_repository,
                const std::vector<spl::shared_ptr<core::video_channel>>& channels,
                const core::channel_info& channel_info);

spl::shared_ptr<core::frame_consumer>
create_preconfigured_consumer(const boost::property_tree::wptree& ptree,
                              const core::video_format_repository& format_repository,
                              const std::vector<spl::shared_ptr<core::video_channel>>& channels,
                              const core::channel_info& channel_info);

}} // namespace caspar::aja
