# Host test harness

Builds the portable firmware sources for the host and exercises them without a Nucleo on the desk.

```
make            # build and run everything
make build      # build only
make run ARGS=--list                 # list test names
make run ARGS=--filter=correlator    # run a subset
make SAN=1      # address + undefined behaviour sanitisers
make valgrind   # run under valgrind, if installed
make clean
```

## What is real and what is not

The firmware sources are compiled **unmodified**. Only three files are left out, because they are nothing but ST peripheral library calls:

| Excluded | Replaced by |
| --- | --- |
| `IOSTM.cpp` | `shim/Stubs.cpp` — `CIO::interrupt()`, the LED/PTT/COS pins |
| `SerialSTM.cpp` | `shim/Stubs.cpp` — `CSerialPort::readInt()` / `writeInt()` |
| `STMUART.cpp` | not needed; the stubs capture bytes directly |

Everything else — `Mode2RX.cpp`, `Mode2TX.cpp`, `IL2PRX.cpp`, `IL2PTX.cpp`, `IL2PRS.cpp`, `IO.cpp`, `SerialPort.cpp`, the AX.25 chain — is the real thing. `MMDVM.cpp` is compiled too (with its `main()` renamed out of the way) so the harness picks up any change to `loop()` or to the firmware globals.

`shim/arm_math.h` and `shim/arm_math.cpp` stand in for the slice of CMSIS-DSP the firmware uses: `arm_fir_fast_q15`, `arm_fir_f32`, `arm_fir_interpolate_q15`, `__SSAT`. The struct layouts and the time-reversed coefficient ordering match the real thing, and the shim counts violations of the documented `numTaps` precondition so a test can assert on them.

## The virtual radio

`Radio.h` wires the firmware's own transmitter to its own receiver through a channel model:

```
mode2TX  ->  CIO::interrupt()  ->  Channel  ->  CIO::interrupt()  ->  CIO::process()  ->  mode2RX
             (DAC, 24 kHz)         phase          (ADC, 24 kHz)      (RX_BLOCK_SIZE)
                                   polarity
                                   level
                                   DC offset
```

Samples move through the real ring buffers at the real block size, so the receiver sees exactly the sample stream it sees on target. The `Channel` knobs are the four things a real link varies that the receiver is supposed to be indifferent to:

- **phase** — a whole-sample delay, 0 to 4. There are five samples per symbol, so sweeping it walks the receiver's sampling instant across a complete symbol period. A receiver that only decodes at some phases will decode intermittently on the air, because the transmitter's and receiver's sample clocks are independent and drift.
- **invert** — the deviation sense. The README at the top of this repo says the receiver detects and decodes either sense, so both must work.
- **gain** and **dcOffset** — receive level and residual DC on the ADC input.

`radio::modulate()` drives the real `CMode2TX`. `radio::modulateSymbols()` is a deliberate second implementation of the wire format, so a test can transmit something the firmware never would — a sync vector with a known number of bit errors, for instance. `baseline_reference_modulator_produces_a_matching_burst` keeps the two honest with each other.

## Fork per test

Every test runs in its own forked process. The firmware is built around file-scope singletons (`io`, `serial`, `mode2RX`, `mode2TX`) with no way to reconstruct them, and several of the faults under test are about state surviving from one packet to the next — so a test that ran after another would not be testing what it claims. Forking also means a test that segfaults or reads wild memory is reported as one failure instead of taking the run down.

## Adding a test

Drop a `tests_*.cpp` into this directory; the Makefile globs them.

```c++
#include "framework.h"
#include "Radio.h"

TF_TEST(my_receiver_does_the_thing)
{
  const std::vector<uint8_t> payload = radio::rawPayload(48U);

  radio::Channel ch;
  ch.phase = 3U;

  CHECK(radio::decodedExactly(radio::loopback(payload, ch), payload));
}
```

`CHECK*` records a failure and carries on, so one run reports everything wrong. `REQUIRE*` abandons the test, for when nothing after it could be meaningful.
