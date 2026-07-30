/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2023,2024 by Jonathan Naylor G4KLX
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
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#if !defined(SERIALPORT_H)
#define  SERIALPORT_H

#include "Config.h"
#include "Globals.h"

#if !defined(SERIAL_SPEED)
#define SERIAL_SPEED 115200
#endif


class CSerialPort {
public:
  CSerialPort();

  void start();

  void process();

  void writeKISSData(uint8_t type, const uint8_t* data, uint16_t length);
  void writeKISSAck(uint16_t token);

  // int32_t rather than int16_t: several callers pass a q31_t, which a 16 bit
  // parameter silently wraps.
  void writeDebug(const char* text);
  void writeDebug(const char* text, int32_t n1);
  void writeDebug(const char* text, int32_t n1, int32_t n2);
  void writeDebug(const char* text, int32_t n1, int32_t n2, int32_t n3);
  void writeDebug(const char* text, int32_t n1, int32_t n2, int32_t n3, int32_t n4);
  void writeDebug(const char* text, int32_t n1, int32_t n2, int32_t n3, int32_t n4, int32_t n5);

private:
  uint8_t  m_buffer[2000U];
  uint16_t m_ptr;
  bool     m_inFrame;
  bool     m_isEscaped;

  void processMessage();

#if defined(SERIAL_DEBUGGING)
  void writeDebugInt(const char* text);
  void writeDebugInt(int32_t num);
  void reverse(char* buffer, uint8_t length) const;
#endif

  // Hardware versions
  void    beginInt(uint8_t n, int speed);
  int     availableForReadInt(uint8_t n);
  int     availableForWriteInt(uint8_t n);
  uint8_t readInt(uint8_t n);
  void    writeInt(uint8_t n, const uint8_t* data, uint16_t length, bool flush = false);
};

#endif
