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

#ifndef __STATIMO_CORE_GENERIC_FACTORY_H_
#define __STATIMO_CORE_GENERIC_FACTORY_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Reflect.h"

#include <memory>

namespace statismo
{

/**
 * Generic factory used to gather common code
 * related to object creation
 */
template <typename T>
class GenericFactory
{
public:
  virtual ~GenericFactory() = default;
  /**
   * Generic object factory for model
   */
  template <typename... Args>
  static T *
  Create(Args &&... args)
  {
    return new T(std::forward<Args>(args)...);
  }

  template <typename... Args>
  static auto
  SafeCreate(Args &&... args)
  {
    return SafeCreateWithCustomDeletor<DefaultDeletor<T>>(std::forward<Args>(args)...);
  }

  template <typename Deletor, typename... Args>
  static std::unique_ptr<T, Deletor>
  SafeCreateWithCustomDeletor(Args &&... args)
  {
    std::unique_ptr<T, Deletor> ptr(new T(std::forward<Args>(args)...), Deletor());
    return ptr;
  }

  template <typename... Args>
  static std::unique_ptr<T>
  SafeCreateStd(Args &&... args)
  {
    return SafeCreateWithCustomDeletor<std::default_delete<T>>(std::forward<Args>(args)...);
  }
};
} // namespace statismo

#endif