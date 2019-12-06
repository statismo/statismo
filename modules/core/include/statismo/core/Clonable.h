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

#ifndef __STATIMO_CORE_CLONABLE_H_
#define __STATIMO_CORE_CLONABLE_H_

#include "statismo/core/CommonTypes.h"

#include <memory>

namespace statismo
{

/* \class Clonable base class
 */
template <typename Derived>
class Clonable
{
public:
  virtual ~Clonable() = default;
  Clonable() = default;
  Clonable(Clonable &&) = delete;
  Clonable &
  operator=(Clonable &&) = delete;
  Clonable &
  operator=(const Clonable &) = delete;

  Derived *
  CloneSelf() const
  {
    return this->CloneImpl();
  }

  UniquePtrType<Derived>
  SafeCloneSelf() const
  {
    return SafeCloneSelfWithCustomDeletor<DefaultDeletor<Derived>>();
  }

  template <typename Deletor>
  std::unique_ptr<Derived, Deletor>
  SafeCloneSelfWithCustomDeletor() const
  {
    std::unique_ptr<Derived, Deletor> ptr{ this->CloneImpl(), Deletor() };
    return ptr;
  }

protected:
  Clonable(const Clonable &) = default;

private:
  virtual Derived *
  CloneImpl() const = 0;
};
} // namespace statismo

#endif