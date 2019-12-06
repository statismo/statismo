/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *
 * Copyright (c) 2011 University of Basel
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


#ifndef __STATISMO_EXCEPTIONS_
#define __STATISMO_EXCEPTIONS_

#include <exception>
#include <string>
#include <tuple>
#include <optional>

// status and exceptions definitions
namespace statismo
{

enum class Status
{
  SUCCESS = 0,
  BAD_VERSION_ERROR,
  BAD_INPUT_ERROR,
  IO_ERROR,
  INVALID_DATA_ERROR,
  NOT_IMPLEMENTED_ERROR,
  INTERNAL_ERROR,
  UNKNOWN_ERROR
};

/**
 * \brief Base exception
 */
class IException : public virtual std::exception
{
public:
  virtual ~IException() = default;
  virtual void
  rethrow() const = 0;
};

/**
 * \brief Rethrowable exception
 */
template <typename Base, typename Derived>
class Rethrowable : public Base
{
public:
  using Base::Base;

  virtual void
  rethrow() const
  {
    throw static_cast<const Derived &>(*this);
  }
};

/**
 * \brief Used to indicate that a method has not yet been implemented
 */
class NotImplementedException : public Rethrowable<IException, NotImplementedException>
{
public:
  NotImplementedException(const char * classname, const char * methodname)
    : m_classname(classname)
    , m_methodname(methodname)
  {}
  virtual ~NotImplementedException() = default;

  const char *
  what() const noexcept
  {
    return (m_classname + "::" + m_methodname).c_str();
  }

private:
  std::string m_classname;
  std::string m_methodname;
};

/**
 * \brief Generic Exception class for the statismo Library.
 */
class StatisticalModelException : public Rethrowable<IException, StatisticalModelException>
{
public:
  explicit StatisticalModelException(const char * message, Status status = Status::INTERNAL_ERROR)
    : m_message(message)
    , m_status(status)
  {}

  const char *
  what() const noexcept
  {
    return m_message.c_str();
  }

  Status
  status() const noexcept
  {
    return m_status;
  }

private:
  std::string m_message;
  Status      m_status;
};

} // namespace statismo

// wrappers to implement exception-safe interfaces
namespace statismo
{
namespace details
{
template <typename Callable, typename R = std::invoke_result_t<Callable>>
struct TranslateImpl
{
  std::tuple<Status, std::optional<R>>
  operator()(Callable && f)
  {
    try
    {
      return std::make_tuple(Status::SUCCESS, std::forward<Callable>(f)());
    }
    catch (const StatisticalModelException & ex)
    {
      return std::make_tuple(ex.status(), std::nullopt);
    }
    catch (...)
    {
      return std::make_tuple(Status::INTERNAL_ERROR, std::nullopt);
    }
  }
};

template <typename Callable>
struct TranslateImpl<Callable, void>
{
  auto
  operator()(Callable && f)
  {
    try
    {
      std::forward<Callable>(f)();
      return Status::SUCCESS;
    }
    catch (const StatisticalModelException & ex)
    {
      return ex.status();
    }
    catch (...)
    {
      return Status::INTERNAL_ERROR;
    }
  }
};
} // namespace details

/**
 * \brief Exception conversion utilities to build exception safe interface over
 *        Statismo
 */
template <typename Callable>
auto
Translate(Callable && f)
{
  return details::TranslateImpl<Callable>()(std::forward<Callable>(f));
}

/**
 * \brief Exception conversion result check
 */
template <typename R>
bool
CheckResult(const std::tuple<Status, std::optional<R>> & res)
{
  return std::get<0>(res) == Status::SUCCESS && std::get<1>(res);
}

/**
 * \brief Exception conversion result check and assert
 */
template <typename R>
bool
CheckResultAndAssert(const std::tuple<Status, std::optional<R>> & res, R && expected)
{
  return std::get<0>(res) == Status::SUCCESS && std::get<1>(res) &&
         std::get<1>(res).value() == std::forward<R>(expected);
}
} // namespace statismo

#endif
