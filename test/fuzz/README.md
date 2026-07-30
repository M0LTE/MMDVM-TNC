# Fuzzing harness

Coverage-guided fuzzing of the firmware's parsers and decoders, using clang's
libFuzzer with ASan and UBSan. The firmware sources are compiled unmodified,
with the same hardware shim the test harness uses.

```
make              # build every fuzzer and generate the seed corpora
make run          # run every fuzzer for TIME seconds each (default 60)
make run-kiss     # run one fuzzer
make run-mode2_rx TIME=3600
make clean
```

Crashing or property-violating inputs land in `artifacts/<target>/`; replay
one with `./build/fuzz_<target> artifacts/<target>/<file>`.

## Targets

| Target | Attack surface | Input |
| --- | --- | --- |
| `kiss` | Everything a host can send: the KISS parser, escaping, command dispatch, the transmitters behind it | raw serial bytes |
| `il2p_rx` | Everything the air can deliver: descrambling, RS decode, header parse, payload block arithmetic | an IL2P frame's bytes |
| `il2p_tx` | The encoder, held to a property: every accepted payload must decode back byte for byte | a payload |
| `mode2_rx` | The mode 2 receiver: RRC filter, sync correlator, timing search, slicers | 16 bit LE q15 samples |
| `ax25_rx` | The mode 1 receiver: bandpass, three AFSK demodulators, HDLC deframer | 16 bit LE q15 samples |

The `il2p_tx` property is not just tidiness: the IL2P receiver computes the
frame CRC over the bytes it *rebuilds* from the translated header, so any
frame `isIL2PType1()` accepts but cannot represent exactly is silently thrown
away by the far end. The fuzzer aborts on any lossy round trip.

## Seeds

`seedgen` builds the corpora out of the firmware itself — real KISS frames,
real IL2P encodings, real modulated bursts — so the fuzzer starts from valid
inputs instead of having to discover the formats from nothing. `make` runs it
automatically; the corpora grow as the fuzzer finds new coverage, and are
disposable.

## What it has found

The first session caught a heap read of uninitialised memory in the AX.25
demodulator's delay line, the matching 11-byte-per-instance leak, and — via
the round-trip property — the type 1 header translations that were lossy:
S frame N(R) corruption, dropped P/F bits, wrong command/response senses,
and untranslatable frames not falling back to type 0. All fixed, with the
regression tests in `../tests_il2p_headers.cpp`.
