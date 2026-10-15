#pragma once

#include <common/log.h>
#include <chrono>
#include <cstdint>
#include <cstdlib>

namespace caspar { namespace diagnostics {

// Opt-in, single-writer counters. Each metric belongs to one execution path;
// never share a metric between channel, preparation and output worker threads.
inline bool cadence_enabled()
{
    static const bool enabled = [] {
        const char* value = std::getenv("CASPAR_AJA_CADENCE_DIAGNOSTICS");
        return value && value[0] == '1' && value[1] == '\0';
    }();
    return enabled;
}

class cadence_metric
{
    using clock = std::chrono::steady_clock;
    const wchar_t* name_;
    clock::time_point epoch_{};
    clock::time_point warmup_{};
    bool started_ = false;
    std::uint64_t count_ = 0;
    std::uint64_t units_ = 0;
    double total_ms_ = 0;
    double max_ms_ = 0;

  public:
    explicit cadence_metric(const wchar_t* name) : name_(name) {}

    cadence_metric(const cadence_metric&) = delete;
    cadence_metric& operator=(const cadence_metric&) = delete;

    template<typename Context>
    void record(std::chrono::steady_clock::time_point begin, std::uint64_t units, Context context) noexcept
    {
        try { record_impl(begin, units, context); } catch (...) {
            // Drop this window on logging/context failure; keep logging bounded.
            epoch_ = clock::now();
            count_ = units_ = 0;
            total_ms_ = max_ms_ = 0;
        }
    }

  private:
    template<typename Context>
    void record_impl(clock::time_point begin, std::uint64_t units, Context context)
    {
        const auto now = clock::now();
        if (!started_) {
            started_ = true;
            warmup_ = now + std::chrono::seconds(3);
            epoch_ = warmup_;
        }
        if (begin < warmup_)
            return;
        const double ms = std::chrono::duration<double, std::milli>(now - begin).count();
        ++count_;
        units_ += units;
        total_ms_ += ms;
        if (ms > max_ms_)
            max_ms_ = ms;
        const double seconds = std::chrono::duration<double>(now - epoch_).count();
        if (seconds < 1.0)
            return;
        CASPAR_LOG(info) << L"[aja-cadence] " << name_ << L" " << context()
                         << L" window_s=" << seconds << L" count=" << count_
                         << L" rate_hz=" << count_ / seconds << L" units_s=" << units_ / seconds
                         << L" mean_ms=" << total_ms_ / count_ << L" max_ms=" << max_ms_;
        epoch_ = now;
        count_ = units_ = 0;
        total_ms_ = max_ms_ = 0;
    }
};

// The disabled path performs no clock reads, string construction or allocation.
// Destruction cannot turn a diagnostic logging failure into a playback failure.
template<typename Context>
class cadence_scope
{
    cadence_metric* metric_;
    std::chrono::steady_clock::time_point begin_{};
    std::uint64_t units_;
    Context context_;
  public:
    cadence_scope(cadence_metric& metric, bool active, std::uint64_t units, Context context)
        : metric_(cadence_enabled() && active ? &metric : nullptr), units_(units), context_(context)
    {
        if (metric_)
            begin_ = std::chrono::steady_clock::now();
    }
    cadence_scope(const cadence_scope&) = delete;
    cadence_scope& operator=(const cadence_scope&) = delete;
    ~cadence_scope() noexcept { finish(); }
    void finish() noexcept
    {
        if (metric_) {
            metric_->record(begin_, units_, context_);
            metric_ = nullptr;
        }
    }
};

}} // namespace caspar::diagnostics
