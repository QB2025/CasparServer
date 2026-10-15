#pragma once

// Chromium resets the stack canary in forked Linux subprocesses. Every caller
// that returns after CefExecuteProcess must therefore avoid checking the old
// canary. Keep these entry frames out of protected callers even with LTO.
// See base/stack_canary_linux.h and CEF's tests/cefsimple/cefsimple_linux.cc.
#if defined(__linux__) && (defined(__GNUC__) || defined(__clang__))
#define CASPAR_CEF_PROCESS_ENTRY __attribute__((no_stack_protector, noinline))
#else
#define CASPAR_CEF_PROCESS_ENTRY
#endif
