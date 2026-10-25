#include <common/diagnostics/cadence.h>
#include <cassert>
#include <thread>
#include <string>
std::wostringstream diagnostic_log;
int main(int argc, char**)
{
    using namespace caspar::diagnostics;
    int contexts = 0;
    cadence_metric metric(L"test");
    auto context = [&] { ++contexts; return L"context"; };
    { cadence_scope scope(metric, false, 800, context); }
    assert(contexts == 0 && diagnostic_log.str().empty());
    { cadence_scope scope(metric, true, 800, context); }
    assert(contexts == 0 && diagnostic_log.str().empty());
    if (argc == 1) {
        assert(!cadence_enabled());
        return 0;
    }
    assert(cadence_enabled());
    std::this_thread::sleep_for(std::chrono::milliseconds(4100));
    { cadence_scope scope(metric, true, 800, context); }
    assert(contexts == 1);
    auto log = diagnostic_log.str();
    assert(log.find(L"count=1") != std::wstring::npos);
    assert(log.find(L"units_s=") != std::wstring::npos);
    for (int i=0; i<100; ++i) { cadence_scope scope(metric, true, 800, context); }
    assert(contexts == 1); // bounded emission
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    { cadence_scope scope(metric, true, 800, []() -> const wchar_t* { throw 1; }); }
    // Context/logging failure must neither escape nor retry on every frame.
    { cadence_scope scope(metric, true, 800, context); }
    assert(contexts == 1);
}
