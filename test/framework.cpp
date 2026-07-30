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

#include "framework.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <sys/wait.h>
#include <unistd.h>

namespace {

  struct Test {
    std::string name;
    tf::TestFn  fn;
  };

  /* Function local static so registration order across translation units is
     not a problem. */
  std::vector<Test>& registry()
  {
    static std::vector<Test> tests;
    return tests;
  }

  bool g_failed = false;

  const char* RED   = "\033[31m";
  const char* GREEN = "\033[32m";
  const char* DIM   = "\033[2m";
  const char* RESET = "\033[0m";

  bool colourise()
  {
    static const bool tty = ::isatty(1) != 0;
    return tty;
  }

  const char* c(const char* code)
  {
    return colourise() ? code : "";
  }

}

namespace tf {

  void registerTest(const char* name, TestFn fn)
  {
    Test t;
    t.name = name;
    t.fn   = fn;
    registry().push_back(t);
  }

  void reportFailure(const char* file, int line, const std::string& message)
  {
    g_failed = true;
    std::printf("      %s%s:%d: %s%s\n", c(RED), file, line, message.c_str(), c(RESET));
    std::fflush(stdout);
  }

  void reportFatal(const char* file, int line, const std::string& message)
  {
    reportFailure(file, line, message);
    std::printf("      %saborting test%s\n", c(DIM), c(RESET));
    std::fflush(stdout);
    ::_exit(1);
  }

  int runAll(int argc, char** argv)
  {
    const char* filter = NULL;
    bool        list   = false;

    for (int i = 1; i < argc; i++) {
      if (::strcmp(argv[i], "--list") == 0)
        list = true;
      else if (::strncmp(argv[i], "--filter=", 9) == 0)
        filter = argv[i] + 9;
      else
        filter = argv[i];
    }

    std::vector<Test>& tests = registry();

    if (list) {
      for (size_t i = 0U; i < tests.size(); i++)
        std::printf("%s\n", tests[i].name.c_str());
      return 0;
    }

    unsigned passed  = 0U;
    unsigned failed  = 0U;
    unsigned skipped = 0U;

    std::vector<std::string> failures;

    for (size_t i = 0U; i < tests.size(); i++) {
      const Test& t = tests[i];

      if (filter != NULL && t.name.find(filter) == std::string::npos) {
        skipped++;
        continue;
      }

      std::printf("  %s ... ", t.name.c_str());
      std::fflush(stdout);

      const pid_t pid = ::fork();
      if (pid < 0) {
        std::printf("%sfork failed%s\n", c(RED), c(RESET));
        failed++;
        continue;
      }

      if (pid == 0) {
        /* Child. Its stdout is the parent's, so any failure detail printed by
           reportFailure lands under the test name. */
        std::printf("\n");
        std::fflush(stdout);
        g_failed = false;
        t.fn();
        std::fflush(stdout);
        ::_exit(g_failed ? 1 : 0);
      }

      int status = 0;
      ::waitpid(pid, &status, 0);

      if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        std::printf("%sok%s\n", c(GREEN), c(RESET));
        passed++;
      } else if (WIFSIGNALED(status)) {
        std::printf("      %skilled by signal %d (%s)%s\n",
                    c(RED), WTERMSIG(status), ::strsignal(WTERMSIG(status)), c(RESET));
        std::printf("  %sFAILED%s\n", c(RED), c(RESET));
        failures.push_back(t.name);
        failed++;
      } else {
        std::printf("  %sFAILED%s\n", c(RED), c(RESET));
        failures.push_back(t.name);
        failed++;
      }

      std::fflush(stdout);
    }

    std::printf("\n%u passed, %u failed", passed, failed);
    if (skipped > 0U)
      std::printf(", %u filtered out", skipped);
    std::printf("\n");

    if (!failures.empty()) {
      std::printf("\nfailing tests:\n");
      for (size_t i = 0U; i < failures.size(); i++)
        std::printf("  %s%s%s\n", c(RED), failures[i].c_str(), c(RESET));
    }

    return failed == 0U ? 0 : 1;
  }

}

int main(int argc, char** argv)
{
  return tf::runAll(argc, argv);
}
