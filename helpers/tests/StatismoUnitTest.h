/*
 * This file is part of the statismo library.
 *
 * Copyright (c) 2019 Laboratory of Medical Information Processing
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __STATIMO_UNIT_TEST_H_
#define __STATIMO_UNIT_TEST_H_

#include <string>
#include <sstream>
#include <iostream>
#include <memory>
#include <algorithm>
#include <functional>
#include <map>
#include <cmath>

/**
 * This file provides minimal utilities for c++ unit testing.
 *
 * As lots of tests are written in python, we want to avoid
 * importing a unit test framework with lots of superfluous
 * features for such minimal needs
 */

#define STATISMO_ASSERT_EX(expr) _STATISMO_ASSERT_EX(expr, __FILE__, __LINE__, true);
#define STATISMO_ASSERT_NEX(expr) _STATISMO_ASSERT_EX(expr, __FILE__, __LINE__, false);
#define STATISMO_ASSERT_TRUE(expr) _STATISMO_ASSERT_EQ(static_cast<bool>(expr), true, __FILE__, __LINE__);
#define STATISMO_ASSERT_FALSE(expr) _STATISMO_ASSERT_EQ(static_cast<bool>(expr), false, __FILE__, __LINE__);
#define STATISMO_ASSERT_NEQ(a, b) _STATISMO_ASSERT_NEQ(a, b, __FILE__, __LINE__);
#define STATISMO_ASSERT_EQ(a, b) _STATISMO_ASSERT_EQ(a, b, __FILE__, __LINE__);
#define STATISMO_ASSERT_GT(a, b) _STATISMO_ASSERT_GT(a, b, __FILE__, __LINE__);
#define STATISMO_ASSERT_GTE(a, b) _STATISMO_ASSERT_GTE(a, b, __FILE__, __LINE__);
#define STATISMO_ASSERT_LT(a, b) _STATISMO_ASSERT_LT(a, b, __FILE__, __LINE__);
#define STATISMO_ASSERT_LTE(a, b) _STATISMO_ASSERT_LTE(a, b, __FILE__, __LINE__);
#define STATISMO_ASSERT_DOUBLE_EQ(a, b) _STATISMO_ASSERT_DOUBLE_EQ(a, b, __FILE__, __LINE__);

inline std::ostream &
operator<<(std::ostream & os, std::nullptr_t)
{
  os << "NULL";
  return os;
}

namespace statismo::test
{
struct TestFixture
{
  virtual ~TestFixture() = default;
  TestFixture() = default;

  /**
   * \brief Setup resources for the test
   */
  virtual void
  setUp() = 0;

  /**
   * \brief Release resources
   */
  virtual void
  tearDown() = 0;

private:
  TestFixture &
  operator=(const TestFixture &) = delete;
  TestFixture(const TestFixture &) = delete;
};

template <typename T>
struct ScopedTest
{

  template <typename... Args>
  ScopedTest(Args &&... args)
    : fixture(new T(std::forward<Args>(args)...))
  {
    fixture->setUp();
  }

  ~ScopedTest() { fixture->tearDown(); }

  const T * operator->() const { return fixture.get(); }

  std::unique_ptr<T> fixture;
};

template <typename Scalar>
inline bool
AbsoluteToleranceEQ(Scalar x, Scalar y, Scalar tol = std::numeric_limits<Scalar>::epsilon())
{
  return std::fabs(x - y) <= tol;
}

template <typename Scalar>
inline bool
RelativeToleranceEQ(Scalar x, Scalar y, Scalar tol = std::numeric_limits<Scalar>::epsilon())
{
  Scalar maxXY = std::max(std::fabs(x), std::fabs(y));
  return std::fabs(x - y) <= tol * maxXY;
}

template <typename Scalar>
inline bool
CombinedToleranceEQ(Scalar x, Scalar y, Scalar tol = std::numeric_limits<Scalar>::epsilon())
{
  Scalar maxXYOne = std::max({ Scalar{ 1.0 }, std::fabs(x), std::fabs(y) });
  return std::fabs(x - y) <= tol * maxXYOne;
}

inline int
RunAllTests(const std::string & module, const std::map<std::string, std::function<int()>> testsMap)
{
  int res{ EXIT_SUCCESS };
  for (const auto & kv : testsMap)
  {
    if (EXIT_FAILURE == kv.second())
    {
      std::cerr << "[FAILED]"
                << "[" << module << "] " << kv.first << " failed!" << std::endl;
      res = EXIT_FAILURE;
    }
  }
  return res;
}

template <typename T1, typename T2>
inline void
PrintError(const char * compToken,
           const char * file,
           int          line,
           const char * lVarName,
           T1           lVarVal,
           const char * rVarName,
           T2           rVarVal)
{
  std::cerr << "[FAILED] (" << file << ":" << std::to_string(line) << ")\n";
  std::cerr << "Expecting " << lVarName << " = ";
  std::cerr << lVarVal;
  std::cerr << "\n";
  std::cerr << compToken << std::endl;
  std::cerr << rVarName << " = ";
  std::cerr << rVarVal;
  std::cerr << "\n";
}

inline void
PrintErrorEx(const char * file, int line, bool hasThrown)
{
  std::cerr << "[FAILED] (" << file << ":" << std::to_string(line) << ")\n";

  if (hasThrown)
  {
    std::cerr << "Expression has triggered an exception that was not expected\n";
  }
  else
  {
    std::cerr << "Expression should have triggered an exception\n";
  }
}

} // namespace statismo::test

#define _STATISMO_ASSERT_NEQ(a, b, file, line) _STATISMO_ASSERT(a, b, (a != b), "NEQ", file, line)
#define _STATISMO_ASSERT_EQ(a, b, file, line) _STATISMO_ASSERT(a, b, (a == b), "EQ", file, line)
#define _STATISMO_ASSERT_GT(a, b, file, line) _STATISMO_ASSERT(a, b, (a > b), "GT", file, line)
#define _STATISMO_ASSERT_GTE(a, b, file, line) _STATISMO_ASSERT(a, b, (a >= b), "GTE", file, line)
#define _STATISMO_ASSERT_LT(a, b, file, line) _STATISMO_ASSERT(a, b, (a < b), "LT", file, line)
#define _STATISMO_ASSERT_LTE(a, b, file, line) _STATISMO_ASSERT(a, b, (a <= b), "LTE", file, line)
#define _STATISMO_ASSERT_DOUBLE_EQ(a, b, file, line)                                                                   \
  _STATISMO_ASSERT(a, b, statismo::test::AbsoluteToleranceEQ((a), (b)), "EQ", file, line)

#define _STATISMO_ASSERT_EX(expr, file, line, flag)                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    bool hasTrown{ false };                                                                                            \
    try                                                                                                                \
    {                                                                                                                  \
      expr;                                                                                                            \
    }                                                                                                                  \
    catch (...)                                                                                                        \
    {                                                                                                                  \
      hasTrown = true;                                                                                                 \
    }                                                                                                                  \
    if (hasTrown != flag)                                                                                              \
    {                                                                                                                  \
      statismo::test::PrintErrorEx(file, line, hasTrown);                                                              \
      return EXIT_FAILURE;                                                                                             \
    }                                                                                                                  \
  } while (0)

#define _STATISMO_ASSERT(a, b, expr, token, file, line)                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!(expr))                                                                                                       \
    {                                                                                                                  \
      statismo::test::PrintError(token, file, line, #a, a, #b, b);                                                     \
      return EXIT_FAILURE;                                                                                             \
    }                                                                                                                  \
  } while (0);
#endif