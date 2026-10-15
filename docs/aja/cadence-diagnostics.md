# AJA baseline and optional diagnostics

The producer uses a shared audio clock adapter for progressive and interlaced
input. It retains captured audio in order, emits each callback's requested
sample count, and uses a bounded slow SWR correction for independent hardware
clocks. Video selects the newest captured raster and may repeat independently
of audio. This does not add a deinterlacer.

The consumer enables multi-format mode on capable devices. Existing legacy
fill/key routing and capture ownership/replacement handling are retained.

## Enable diagnostic reports

Diagnostics are disabled by default. Set `CASPAR_AJA_CADENCE_DIAGNOSTICS=1`
before starting the server; restart after changing it. Only the exact value
`1` enables reports. In PowerShell:

```powershell
$env:CASPAR_AJA_CADENCE_DIAGNOSTICS = '1'
.\casparcg.exe
```

To disable them, remove the variable or set it to `0`, then restart.
Reports use INFO logging, so the server log level must allow INFO.

The switch controls cadence timing in the channel, stage, producer proxy,
AJA receive/frame construction, output queues, preparation, transfer, and
vertical waits. It also controls producer callback histograms, audio counters,
reservoir observations, startup format probes, and reservoir/queue priming
reports. Disabled callback and reservoir instrumentation returns before locks,
clock reads, histogram allocations, or counter increments. Essential startup,
signal loss/reacquisition, warnings, and errors remain visible.

`callback-diag` reports `receive_impl` and `receive_clock_adapted` requests and
return paths. `audio-diag` counts sample frames per channel, not interleaved
values. Raw/packaged drop counters describe whole-frame queue overflow;
`underflows` and `missing-output` describe short resampler output after top-up.
Cadence reports have a three-second warm-up; callback/audio reports use their
own approximately one-second windows. Windows are not synchronized.

Output mode, reference, and AutoCirculate counters are read-only observations.
Hardware totals reset on AutoCirculate restart. Queue depths are instantaneous
samples; a silent metric cannot establish zero activity or diagnose a blocked
hardware call. Listen to output as well as inspecting counters.

## Validation

The uploaded baseline was compiled and tested on Windows by its operator;
1080i50 input to 1080p60 output no longer produced chopped audio. This is
validation of the baseline before the diagnostic cleanup.

Local cleanup verification covers enabled/disabled cadence helper behavior and
the actual producer diagnostic methods in an extracted C++ harness. Full AJA
server compilation and hardware verification of the cleanup remain Windows
acceptance checks.

Standalone helper tests:

```sh
cmake -S tests/cadence -B build/cadence
cmake --build build/cadence
ctest --test-dir build/cadence --output-on-failure
```

Repeat 1080p50 -> 1080p60 and 1080i50 -> 1080p60 with diagnostics off and on.
After warm-up, expect approximately 60 callbacks and 48,000 requested sample
frames per second, a bounded reservoir, and no sustained missing output.
Also check matching cadences, fractional rates, replacement/CLEAR, and cable
pull/reacquisition. The same audio processing runs with either switch value.
