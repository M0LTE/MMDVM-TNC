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
 * A deliberately small test framework.
 *
 * The firmware is built around file scope singletons -- io, serial, mode2RX,
 * mode2TX -- with no way to reconstruct them, and several of the bugs under
 * test are about state that survives from one packet to the next. So every
 * test runs in its own forked process and gets genuinely pristine globals. A
 * child that segfaults or trips a sanitiser is reported as a failure rather
 * than taking the whole run down with it, which matters when the thing you are
 * testing is a read of uninitialised memory.
 */

#if !defined(FRAMEWORK_H)
#define  FRAMEWORK_H

#include <sstream>
#include <string>
#include <vector>

namespace tf {

  typedef void (*TestFn)();

  void registerTest(const char* name, TestFn fn);

  int runAll(int argc, char** argv);

  /* Record a failure in the current test and keep going. */
  void reportFailure(const char* file, int line, const std::string& message);

  /* Record a failure and abandon the current test immediately. */
  void reportFatal(const char* file, int line, const std::string& message);

  struct Registrar {
    Registrar(const char* name, TestFn fn) { registerTest(name, fn); }
  };

  /* Lets the CHECK macros build a message with << without needing a named
     stream at every call site. */
  struct Str {
    std::ostringstream os;

    template <typename T>
    Str& operator<<(const T& v) { os << v; return *this; }

    std::string str() const { return os.str(); }
  };

}

#define TF_TEST(name)                                                     \
  static void name();                                                     \
  static const tf::Registrar tf_reg_##name(#name, name);                  \
  static void name()

#define TF_STR(expr) ((tf::Str() << expr).str())

/* Non fatal: note the failure, carry on so one run reports everything. */
#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond))                                                          \
      tf::reportFailure(__FILE__, __LINE__, "CHECK(" #cond ")");          \
  } while (0)

#define CHECK_MSG(cond, msg)                                              \
  do {                                                                    \
    if (!(cond))                                                          \
      tf::reportFailure(__FILE__, __LINE__, TF_STR("CHECK(" #cond ") -- " << msg)); \
  } while (0)

#define CHECK_EQ(a, b)                                                    \
  do {                                                                    \
    if (!((a) == (b)))                                                    \
      tf::reportFailure(__FILE__, __LINE__,                               \
        TF_STR("CHECK_EQ(" #a ", " #b ") -- got " << (a) << ", expected " << (b))); \
  } while (0)

/* Fatal: the rest of the test cannot mean anything if this fails. */
#define REQUIRE(cond)                                                     \
  do {                                                                    \
    if (!(cond))                                                          \
      tf::reportFatal(__FILE__, __LINE__, "REQUIRE(" #cond ")");          \
  } while (0)

#define REQUIRE_MSG(cond, msg)                                            \
  do {                                                                    \
    if (!(cond))                                                          \
      tf::reportFatal(__FILE__, __LINE__, TF_STR("REQUIRE(" #cond ") -- " << msg)); \
  } while (0)

#define REQUIRE_EQ(a, b)                                                  \
  do {                                                                    \
    if (!((a) == (b)))                                                    \
      tf::reportFatal(__FILE__, __LINE__,                                 \
        TF_STR("REQUIRE_EQ(" #a ", " #b ") -- got " << (a) << ", expected " << (b))); \
  } while (0)

#endif
