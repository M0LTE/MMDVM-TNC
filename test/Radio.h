/*
 *   Copyright (C) 2026 by Tom Fanning M0LTE
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

/*
 * A virtual radio: firmware transmitter -> channel -> firmware receiver.
 *
 * modulate() drives the real CMode2TX through the real CIO to produce the
 * on-air waveform, sampled at 24 kHz in the ADC's own domain (nominally
 * centred on 2048, 12 bit). demodulate() pushes samples back in through
 * CIO::interrupt() and CIO::process(), so the receiver under test sees exactly
 * the sample stream, block size and ring buffer behaviour it sees on target.
 *
 * Everything in between is the Channel: sample phase, polarity, level and DC
 * offset. Those are the four things a real link varies that the receiver is
 * supposed to be indifferent to.
 */

#if !defined(RADIO_H)
#define  RADIO_H

#include <cstdint>
#include <vector>

namespace radio {

  /* Mid rail of the 12 bit ADC/DAC, i.e. no signal. */
  const uint16_t MID = 2048U;

  struct Channel {
    /* Whole sample delay inserted ahead of the burst, 0 .. 4. There are five
       samples per symbol, so this sweeps the receiver's sampling phase across
       a complete symbol period. */
    unsigned phase;

    /* Flip the deviation sense. The README claims the receiver decodes either
       sense, so both must work. */
    bool invert;

    /* Receive level, 1.0 being the transmitter's nominal output. */
    double gain;

    /* Residual DC on the ADC input, in LSB. */
    int dcOffset;

    /* Peak uniform noise added at the ADC, in LSB. Deterministic: the same
       seed always produces the same noise. */
    unsigned noise;
    uint32_t seed;

    Channel() : phase(0U), invert(false), gain(1.0), dcOffset(0), noise(0U), seed(1U) { }
  };

  /* An ordinary AX.25 UI frame, the sort of thing a TNC actually carries.
     CIL2PTX will give this a type 1 (translated) header. */
  std::vector<uint8_t> sampleUIFrame(const char* info = "TEST");

  /* Bytes that are deliberately not AX.25 shaped, so they travel under a
     type 0 (raw) header and come back byte for byte. Better for testing the
     modem itself, because nothing in between reinterprets the content. */
  std::vector<uint8_t> rawPayload(size_t length);

  /* ---- reference modulator -------------------------------------------- *
   *
   * modulate() above drives the firmware's own transmitter, which is the
   * faithful thing to do for end to end tests. These two build a waveform
   * from the outside instead, so a test can put deliberately wrong symbols
   * on the air -- a corrupted sync vector, for instance -- which the
   * firmware transmitter will never produce.
   *
   * Symbol values are the usual +3 / +1 / -1 / -3.
   */

  /* Expand bytes to symbols using the mode 2 dibit mapping, MSB pair first. */
  std::vector<int8_t> bytesToSymbols(const std::vector<uint8_t>& bytes);

  /* Shape symbols into 24 kHz ADC domain samples, five samples per symbol. */
  std::vector<uint16_t> modulateSymbols(const std::vector<int8_t>& symbols);

  /* The full on-air byte stream for a payload: preamble, sync vector, IL2P
     frame, trailing spacer. Handy when a test wants to alter one field. */
  std::vector<uint8_t> onAirBytes(const std::vector<uint8_t>& payload, unsigned preambleBytes = 36U);

  /* Where the sync vector's symbols sit within bytesToSymbols(onAirBytes()). */
  unsigned syncSymbolOffset(unsigned preambleBytes = 36U);

  /* Build the on-air waveform for one packet. txDelay is in units of 10 ms of
     preamble, matching the KISS TX Delay parameter; the default keeps the
     tests short while leaving the receive filter plenty of time to settle.
     mode selects the transmitter: 1 gives 1200 baud AFSK at 24 kHz, 2 gives
     24 kHz C4FSK samples, 3 gives 48 kHz. */
  std::vector<uint16_t> modulate(const uint8_t* payload, uint16_t length, uint8_t txDelay = 3U, uint8_t mode = 2U);

  /* Run the given mode's transmitter until the DAC goes quiet and return the
     waveform it produced. For when the transmission was queued by something
     other than this file -- a KISS frame fed through serial.process(), say --
     so modulate() cannot be used. Returns an empty vector if nothing was
     transmitted. */
  std::vector<uint16_t> runTX(uint8_t mode = 2U);

  std::vector<uint16_t> applyChannel(const std::vector<uint16_t>& in, const Channel& ch);

  /* Resample the waveform as a receiver whose sample clock is off by `ppm`
     would see it, by linear interpolation. Transmit and receive oscillators
     are never the same frequency on real hardware; a positive value makes
     the receiver's clock fast, so the burst appears stretched. */
  std::vector<uint16_t> applyClockError(const std::vector<uint16_t>& in, int ppm);

  /* Concatenate bursts with `gapSamples` of silence between them. */
  std::vector<uint16_t> silence(size_t samples);
  void                  append(std::vector<uint16_t>& dst, const std::vector<uint16_t>& src);

  /* Returns the KISS frames the firmware emitted, each still carrying its
     leading type/address byte.
     resetReceiver false leaves mode2RX exactly as the constructor left it,
     which is what a freshly powered modem sees for its first ever packet. */
  std::vector<std::vector<uint8_t> > demodulate(const std::vector<uint16_t>& adc,
                                                bool resetReceiver = true, uint8_t mode = 2U);

  /* modulate -> channel -> demodulate, for the common case. */
  std::vector<std::vector<uint8_t> > loopback(const std::vector<uint8_t>& payload, const Channel& ch);

  /* True if `frames` contains exactly one frame whose payload (after the KISS
     type byte) equals `payload`. */
  bool decodedExactly(const std::vector<std::vector<uint8_t> >& frames, const std::vector<uint8_t>& payload);

  /* ---- recorded captures ---------------------------------------------- *
   *
   * Feeding a recording of a different implementation is the only way to
   * check the receiver against anything other than this firmware's own
   * transmitter. modulate() and demodulate() share every assumption in this
   * repo; a capture does not.
   */

  /* Mono 16 bit PCM only, which is what a sound card gives you. */
  std::vector<int16_t> loadWav(const char* path, unsigned& sampleRate);

  /* Integer decimation with a lowpass first, for getting a 48 kHz capture
     down to the 24 kHz the mode 2 receiver runs at. */
  std::vector<int16_t> decimate(const std::vector<int16_t>& in, unsigned factor);

  /* Scale into the 12 bit ADC domain, putting the waveform's peak
     `peakCounts` either side of mid rail. The firmware's own transmitter
     produces about 695. */
  std::vector<uint16_t> toAdc(const std::vector<int16_t>& in, int peakCounts);

}

#endif
