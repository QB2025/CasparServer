# ROUTE frame regression

This standalone target compiles the production frame and ROUTE tag visitor code.
It needs a C++17 compiler and Boost headers, but no CEF, graphics context, AJA
hardware or full server dependencies.

From the repository root:

```sh
cmake -S tests -B build-route-tests -DBOOST_ROOT=/path/to/boost
cmake --build build-route-tests --config Debug
ctest --test-dir build-route-tests -C Debug --output-on-failure
```

It checks GPU-only frames matching the CEF D3D import representation, CPU and
audio-only frames, resource ownership, chained retagging, source immutability,
metadata and geometry, nested video/audio draw frames and stable route tags.
It also injects an opaque-storage copy failure and checks that the production
tag visitor recovers without carrying a partial traversal into the next frame.
Invalid new frames must still fail constructor validation.

The target does not exercise the full ROUTE subscriber, CEF/D3D interop, channel
stage or output consumers. Those need an integration test with a server build.
