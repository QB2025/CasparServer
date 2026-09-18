#pragma once

#include <core/module_dependencies.h>

namespace caspar { namespace aja {

std::wstring get_version();
void init(const core::module_dependencies& dependencies);

}} // namespace caspar::aja
