/*
 * Copyright 2013 Sveriges Television AB http://casparcg.com/
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

#include "html.h"
#include "util.h"

#include "producer/html_cg_proxy.h"
#include "producer/html_producer.h"

#include <common/cef_process_entry.h>
#include <common/env.h>
#include <common/executor.h>
#include <common/future.h>

#include <core/producer/cg_proxy.h>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/range/algorithm/remove_if.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include <include/cef_app.h>
#include <include/cef_version.h>

#ifdef WIN32
#include <accelerator/d3d/d3d_device.h>
#endif

namespace caspar::html {

std::unique_ptr<executor> g_cef_executor;

void caspar_log(const CefRefPtr<CefBrowser>&        browser,
                boost::log::trivial::severity_level level,
                const std::string&                  message)
{
    if (browser != nullptr) {
        auto msg = CefProcessMessage::Create(LOG_MESSAGE_NAME);
        msg->GetArgumentList()->SetInt(0, level);
        msg->GetArgumentList()->SetString(1, message);

        CefRefPtr<CefFrame> mainFrame = browser->GetMainFrame();
        if (mainFrame) {
            mainFrame->SendProcessMessage(PID_BROWSER, msg);
        }
    }
}

class remove_handler : public CefV8Handler
{
    CefRefPtr<CefBrowser> browser_;

  public:
    explicit remove_handler(const CefRefPtr<CefBrowser>& browser)
        : browser_(browser)
    {
    }

    bool Execute(const CefString&       name,
                 CefRefPtr<CefV8Value>  object,
                 const CefV8ValueList&  arguments,
                 CefRefPtr<CefV8Value>& retval,
                 CefString&             exception) override
    {
        if (!CefCurrentlyOn(TID_RENDERER)) {
            return false;
        }

        CefRefPtr<CefFrame> mainFrame = browser_->GetMainFrame();
        if (mainFrame) {
            mainFrame->SendProcessMessage(PID_BROWSER, CefProcessMessage::Create(REMOVE_MESSAGE_NAME));
        }

        return true;
    }

    IMPLEMENT_REFCOUNTING(remove_handler);
};

namespace {

std::wstring trim_html_switch_text(const std::wstring& text)
{
    const auto first = text.find_first_not_of(L" \t\r\n");
    if (first == std::wstring::npos)
        return {};
    return text.substr(first, text.find_last_not_of(L" \t\r\n") - first + 1);
}

struct html_switch
{
    std::wstring name;
    std::wstring value;
    bool         has_value;
};

html_switch parse_html_switch(const std::wstring& text)
{
    const auto arg = trim_html_switch_text(text);
    if (arg.size() < 3 || arg.compare(0, 2, L"--") != 0)
        throw std::invalid_argument("html.args.arg must be --name or --name=value");

    const auto separator = arg.find(L'=');
    const auto name = arg.substr(2, separator == std::wstring::npos ? separator : separator - 2);
    if (name.empty() || name.front() == L'-' ||
        name.find_first_not_of(L"abcdefghijklmnopqrstuvwxyz0123456789-") != std::wstring::npos)
        throw std::invalid_argument("html.args.arg switch names must be lowercase ASCII letters, digits or hyphens");

    const auto value = separator == std::wstring::npos ? std::wstring() : arg.substr(separator + 1);
    if (value.find_first_of(L"\r\n") != std::wstring::npos || value.find(L'\0') != std::wstring::npos)
        throw std::invalid_argument("html.args.arg values must not contain line breaks or NUL characters");
    if ((name == L"enable-features" || name == L"disable-features") && trim_html_switch_text(value).empty())
        throw std::invalid_argument("html.args feature switches require a non-empty comma-separated value");

    return {name, value, separator != std::wstring::npos};
}

std::wstring merge_html_features(const std::wstring& existing, const std::wstring& configured)
{
    std::vector<std::wstring> features;
    for (const auto& list : {existing, configured}) {
        std::size_t start = 0;
        while (start < list.size()) {
            const auto end = list.find(L',', start);
            const auto feature = trim_html_switch_text(list.substr(start, end == std::wstring::npos ? end : end - start));
            if (!feature.empty() && std::find(features.begin(), features.end(), feature) == features.end())
                features.push_back(feature);
            if (end == std::wstring::npos)
                break;
            start = end + 1;
        }
    }
    std::wstring result;
    for (const auto& feature : features) {
        if (!result.empty())
            result += L",";
        result += feature;
    }
    return result;
}

std::vector<html_switch> read_configured_html_switches()
{
    const auto args = env::properties().get_child_optional(L"configuration.html.args");
    if (!args)
        return {};

    // Parse before CefInitialize so invalid configuration never throws across CEF callbacks.
    std::vector<html_switch> switches;
    for (const auto& entry : *args) {
        if (entry.first == L"arg")
            switches.push_back(parse_html_switch(entry.second.get_value<std::wstring>()));
    }

    return switches;
}

void apply_configured_html_switches(CefRefPtr<CefCommandLine> command_line,
                                    const std::vector<html_switch>& switches)
{
    for (const auto& arg : switches) {
        const bool feature_list = arg.name == L"enable-features" || arg.name == L"disable-features";
        const auto value = feature_list
                               ? merge_html_features(command_line->GetSwitchValue(arg.name).ToWString(), arg.value)
                               : arg.value;
        command_line->RemoveSwitch(arg.name);
        if (arg.has_value)
            command_line->AppendSwitchWithValue(arg.name, value);
        else
            command_line->AppendSwitch(arg.name);
        // Avoid logging arbitrary values, which can contain credentials or URLs.
        CASPAR_LOG(info) << L"[html] Applied configured CEF switch --" << arg.name;
    }

    if (!switches.empty()) {
        CASPAR_LOG(info) << L"[html] CEF disable-features: "
                         << command_line->GetSwitchValue("disable-features").ToWString();
    }
}

} // namespace

class renderer_application
    : public CefApp
    , CefRenderProcessHandler
{
    std::vector<CefRefPtr<CefV8Context>> contexts_;
    const bool                           enable_gpu_;
    const bool                           shared_texture_;
    const std::vector<html_switch>       configured_switches_;

  public:
    explicit renderer_application(const bool enable_gpu, const bool shared_texture,
                                  std::vector<html_switch> configured_switches = {})
        : enable_gpu_(enable_gpu)
        , shared_texture_(shared_texture)
        , configured_switches_(std::move(configured_switches))
    {
    }

    CefRefPtr<CefRenderProcessHandler> GetRenderProcessHandler() override { return this; }

    void
    OnContextCreated(CefRefPtr<CefBrowser> browser, CefRefPtr<CefFrame> frame, CefRefPtr<CefV8Context> context) override
    {
        if (!frame->IsMain())
            return;

        caspar_log(
            browser, boost::log::trivial::trace, "context for frame " + frame->GetIdentifier().ToString() + " created");
        contexts_.push_back(context);

        auto window = context->GetGlobal();

        window->SetValue(
            "remove", CefV8Value::CreateFunction("remove", new remove_handler(browser)), V8_PROPERTY_ATTRIBUTE_NONE);

        CefRefPtr<CefV8Value>     ret;
        CefRefPtr<CefV8Exception> exception;
        bool                      injected = context->Eval(R"(
            window.caspar = window.casparcg = {};
		)",
                                      CefString(),
                                      1,
                                      ret,
                                      exception);

        if (!injected) {
            caspar_log(browser, boost::log::trivial::error, "Could not inject javascript animation code.");
        }
    }

    void OnContextReleased(CefRefPtr<CefBrowser>   browser,
                           CefRefPtr<CefFrame>     frame,
                           CefRefPtr<CefV8Context> context) override
    {
        if (!frame->IsMain())
            return;

        auto removed =
            boost::remove_if(contexts_, [&](const CefRefPtr<CefV8Context>& c) { return c->IsSame(context); });

        if (removed != contexts_.end()) {
            caspar_log(browser,
                       boost::log::trivial::trace,
                       "context for frame " + frame->GetIdentifier().ToString() + " released");
        } else {
            caspar_log(browser,
                       boost::log::trivial::warning,
                       "context for frame " + frame->GetIdentifier().ToString() + " released, but not found");
        }
    }

    void OnBrowserDestroyed(CefRefPtr<CefBrowser> browser) override { contexts_.clear(); }

    void OnBeforeCommandLineProcessing(const CefString& process_type, CefRefPtr<CefCommandLine> command_line) override
    {
        if (enable_gpu_) {
            command_line->AppendSwitch("enable-webgl");

            auto default_backend = L""; // Let CEF choose what is best
#if __unix__
            // If there is no X server, Chromium requires us to force it to the angle backend
            if (getenv("DISPLAY") == nullptr)
                default_backend = L"vulkan";
#endif

            // This gives better performance on the gpu->cpu readback, but can perform worse with intense templates
            auto backend = env::properties().get(L"configuration.html.angle-backend", default_backend);
            if (backend.size() > 0) {
                command_line->AppendSwitchWithValue("use-angle", backend);
            }
        }

#if __unix__
        if (getenv("DISPLAY") == nullptr) {
            command_line->AppendSwitchWithValue("ozone-platform", "headless");
        }
#endif

        command_line->AppendSwitch("disable-web-security");
        command_line->AppendSwitch("enable-begin-frame-scheduling");
        command_line->AppendSwitch("enable-media-stream");
        command_line->AppendSwitch("use-fake-ui-for-media-stream");
        command_line->AppendSwitchWithValue("autoplay-policy", "no-user-gesture-required");
        command_line->AppendSwitchWithValue("remote-allow-origins", "*");

        if (process_type.empty() && !enable_gpu_) {
            // This gives more performance, but disabled gpu effects. Without it a single 1080p producer cannot be run
            // smoothly

            command_line->AppendSwitch("disable-gpu");
            command_line->AppendSwitch("disable-gpu-compositing");
            command_line->AppendSwitchWithValue("disable-gpu-vsync", "gpu");
        }

        // Only the browser receives parsed configuration. Chromium forwards
        // applicable switches to children, whose early entry has no config.
        if (process_type.empty())
            apply_configured_html_switches(command_line, configured_switches_);
    }

    IMPLEMENT_REFCOUNTING(renderer_application);
};

CASPAR_CEF_PROCESS_ENTRY bool intercept_command_line(int argc, char** argv)
{
#ifdef _WIN32
    CefMainArgs main_args;
#else
    CefMainArgs main_args(argc, argv);
#endif

    return CefExecuteProcess(main_args, CefRefPtr<CefApp>(new renderer_application(false, false)), nullptr) >= 0;
}

void init(const core::module_dependencies& dependencies)
{
    dependencies.producer_registry->register_producer_factory(L"HTML Producer", html::create_producer);

    CefMainArgs main_args;
    g_cef_executor = std::make_unique<executor>(L"cef");
    bool result    = g_cef_executor->invoke([&] {
#ifdef WIN32
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif
        const auto gpu = is_gpu_shared_texture_enabled();
        const auto configured_switches = read_configured_html_switches();

        CefSettings settings;
        settings.command_line_args_disabled   = false;
        settings.no_sandbox                   = true;
        settings.remote_debugging_port        = env::properties().get(L"configuration.html.remote-debugging-port", 0);
        settings.windowless_rendering_enabled = true;

        auto cache_path = env::properties().get(L"configuration.html.cache-path", L"cef-cache");
        if (!cache_path.empty()) {
            if (!boost::filesystem::path(cache_path).is_absolute()) {
                cache_path = caspar::env::initial_folder() + L"/" + cache_path;
            }
            CASPAR_LOG(info) << L"[html] Using CEF cache path: " << cache_path;
            CefString(&settings.cache_path).FromWString(cache_path);
        }

        return CefInitialize(
            main_args, settings, CefRefPtr<CefApp>(new renderer_application(gpu.first, gpu.second, configured_switches)), nullptr);
    });

    if (!result) {
        CASPAR_LOG(error) << "[html] Failed to initialize CEF";
        return;
    }

    g_cef_executor->begin_invoke([&] { CefRunMessageLoop(); });
    dependencies.cg_registry->register_cg_producer(
        L"html",
        {L".html"},
        [](const spl::shared_ptr<core::frame_producer>& producer) { return spl::make_shared<html_cg_proxy>(producer); },
        [](const core::frame_producer_dependencies& dependencies, const std::wstring& filename) {
            return html::create_cg_producer(dependencies, {filename});
        },
        false);
}

void uninit()
{
    if (!g_cef_executor)
        return;

    invoke([] { CefQuitMessageLoop(); });
    g_cef_executor->begin_invoke([&] { CefShutdown(); });
    g_cef_executor.reset();
}

class cef_task : public CefTask
{
  private:
    std::promise<void>    promise_;
    std::function<void()> function_;

  public:
    explicit cef_task(std::function<void()> function)
        : function_(std::move(function))
    {
    }

    void Execute() override
    {
        CASPAR_LOG(trace) << "[cef_task] executing task";

        try {
            function_();
            promise_.set_value();
            CASPAR_LOG(trace) << "[cef_task] task succeeded";
        } catch (...) {
            promise_.set_exception(std::current_exception());
            CASPAR_LOG(warning) << "[cef_task] task failed";
        }
    }

    std::future<void> future() { return promise_.get_future(); }

    IMPLEMENT_REFCOUNTING(cef_task);
};

void invoke(const std::function<void()>& func) { begin_invoke(func).get(); }

std::future<void> begin_invoke(const std::function<void()>& func)
{
    CefRefPtr<cef_task> task = new cef_task(func);

    if (CefCurrentlyOn(TID_UI)) {
        // Avoid deadlock.
        task->Execute();
        return task->future();
    }

    if (CefPostTask(TID_UI, task.get())) {
        return task->future();
    }
    CASPAR_THROW_EXCEPTION(caspar_exception() << msg_info("[cef_executor] Could not post task"));
}

std::pair<bool, bool> is_gpu_shared_texture_enabled()
{
    const bool enable_gpu            = env::properties().get(L"configuration.html.enable-gpu", false);
    bool       shared_texture_enable = false;

#ifdef WIN32
    if (enable_gpu) {
        auto dev = accelerator::d3d::d3d_device::get_device();
        if (!dev) {
            CASPAR_LOG(warning) << L"Failed to create directX device for cef gpu acceleration";
        } else {
            shared_texture_enable = true;
        }
    }
#else
    // It would be nice to support this on linux, but it needs some investigation and work
    // Test results (March 2026) suggest that linux without shared-texture is more performant than windows with or
    // without
#endif

    return std::make_pair(enable_gpu, shared_texture_enable);
}

} // namespace caspar::html
