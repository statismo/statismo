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

#ifndef __STATIMO_CORE_IMPL_WRAPPER_H_
#define __STATIMO_CORE_IMPL_WRAPPER_H_

#include "statismo/core/NonCopyable.h"
#include "statismo/core/CommonTypes.h"
#include "statismo/core/Exceptions.h"

#include <memory>
#include <type_traits>

namespace statismo
{

struct NullInitializer
{};
struct SafeInitializer
{};

/*
 * Base class for class that wraps a statismo implementation
 */
template <typename I, typename T = NullInitializer>
class ImplWrapper : public NonCopyable
{
public:
  using ImplType = I;

  virtual ~ImplWrapper() = default;

  ImplWrapper()
  {
    if constexpr (std::is_same_v<T, SafeInitializer>)
    {
      m_impl = ImplType::SafeCreate();
    }
  }

  /**
   * Get internal implementation
   */
  const I *
  GetStatismoImplObj() const
  {
    return m_impl.get();
  }

  /**
   * Set internal implementation
   */
  void
  SetStatismoImplObj(UniquePtrType<I> impl)
  {
    m_impl = std::move(impl);
  }

  /**
   * Create internal implementation by forwarding ctor args
   */
  template <typename U, typename = std::enable_if_t<!std::is_same_v<UniquePtrType<I>, std::decay_t<U>>>>
  void
  SetStatismoImplObj(U && arg)
  {
    this->SetStatismoImplObj(ImplType::SafeCreate(std::forward<U>(arg)));
  }

  /**
   * Create internal implementation by forwarding ctor args
   */
  template <typename... Args>
  void
  SetStatismoImplObj(Args &&... args)
  {
    this->SetStatismoImplObj(ImplType::SafeCreate(std::forward<Args>(args)...));
  }

  /**
   * Forward call to implementation
   */
  template <typename Callable, typename... Args>
  decltype(auto)
  CallForwardImpl(Callable && op, Args &&... args)
  {
    return static_cast<const ImplWrapper *>(this)->CallForwardImpl(std::forward<Callable>(op),
                                                                   std::forward<Args>(args)...);
  }

  /**
   * Forward call to implementation
   */
  template <typename Callable, typename... Args>
  decltype(auto)
  CallForwardImpl(Callable && op, Args &&... args) const
  {
    return CallForward(std::forward<Callable>(op), m_impl.get(), std::forward<Args>(args)...);
  }

  /**
   * Forward call to implementation with exception handling
   *
   * \warning Note that \a h must be handler that throws
   */
  template <typename Handler, typename Callable, typename... Args>
  decltype(auto)
  CallForwardImplTrans(Handler && h, Callable && op, Args &&... args)
  {
    return static_cast<const ImplWrapper *>(this)->CallForwardImplTrans(
      std::forward<Handler>(h), std::forward<Callable>(op), std::forward<Args>(args)...);
  }

  /**
   * Forward call to implementation with exception handling
   *
   * \warning Note that \a h must be handler that throws
   */
  template <typename Handler, typename Callable, typename... Args>
  decltype(auto)
  CallForwardImplTrans(Handler && h, Callable && op, Args &&... args) const
  {
    if (!m_impl)
    {
      std::invoke(std::forward<Handler>(h), "invalid null implementation");
    }

    try
    {
      return CallForwardImpl(std::forward<Callable>(op), std::forward<Args>(args)...);
    }
    catch (const StatisticalModelException & ex)
    {
      std::invoke(std::forward<Handler>(h), ex.what());
    }

    // Should never reach here the handler must be a thrower
    throw;
  }

  template <typename Callable, typename... Args>
  decltype(auto)
  CallForward(Callable && op, Args &&... args)
  {
    return std::invoke(std::forward<Callable>(op), std::forward<Args>(args)...);
  }

  template <typename Callable, typename... Args>
  decltype(auto)
  CallForward(Callable && op, Args &&... args) const
  {
    return std::invoke(std::forward<Callable>(op), std::forward<Args>(args)...);
  }

protected:
  UniquePtrType<I> m_impl;
};
} // namespace statismo

#endif