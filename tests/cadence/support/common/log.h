#pragma once
#include <sstream>
extern std::wostringstream diagnostic_log;
#define CASPAR_LOG(level) diagnostic_log
