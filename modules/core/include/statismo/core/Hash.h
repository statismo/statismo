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

#ifndef __STATIMO_CORE_HASH_H_
#define __STATIMO_CORE_HASH_H_

#include "statismo/core/Reflect.h"

// Hashing utilities
namespace statismo
{
namespace details
{
inline constexpr auto HasSize = is_valid([](auto x) -> decltype((void)value_t(x).size()) {});

inline constexpr auto HasGetPointDimension = is_valid([](auto x) -> decltype((void)value_t(x).GetPointDimension()) {});

template <typename T>
inline void
HashCombine(std::size_t & seed, const T & v)
{
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template <typename T>
inline auto
HashImpl(const T & v) -> std::enable_if_t<HasSize(type<T>), std::size_t>
{
  size_t value = 0;
  for (unsigned i = 0; i < v.size(); i++)
  {
    HashCombine(value, v(i));
  }
  return value;
}

template <typename T>
inline auto
HashImpl(const T & v) -> std::enable_if_t<HasGetPointDimension(type<T>), size_t>
{
  size_t value = 0;
  for (unsigned i = 0; i < v.GetPointDimension(); i++)
  {
    HashCombine(value, v[i]);
  }
  return value;
}
} // namespace details

/**
 * Custom hash functor used in hashmap
 */
template <typename T>
class Hash
{
public:
  size_t
  operator()(const T & v) const
  {
    return details::HashImpl(v);
  }
};
} // namespace statismo

#endif