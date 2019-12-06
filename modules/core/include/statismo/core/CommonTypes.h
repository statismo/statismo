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

#ifndef __STATIMO_CORE_COMMON_TYPES_H_
#define __STATIMO_CORE_COMMON_TYPES_H_

#include "statismo/core/CoreTraits.h"
#include "statismo/core/Exceptions.h"
#include "statismo/core/Config.h"

#include <memory>

namespace statismo
{

// numerical value
static constexpr double PI = 3.14159265358979323846;

// the type that is used for all vector and matrices throughout the library.
using ScalarType = float;

// aliases to generic eigen types
using MatrixType = GenericEigenTraits<ScalarType>::MatrixType;
using MatrixTypeDoublePrecision = GenericEigenTraits<double>::MatrixType;
using DiagMatrixType = GenericEigenTraits<ScalarType>::DiagMatrixType;
using VectorType = GenericEigenTraits<ScalarType>::VectorType;
using VectorTypeDoublePrecision = GenericEigenTraits<double>::VectorType;
using RowVectorType = GenericEigenTraits<ScalarType>::RowVectorType;

// type definitions used in the standard file format.
// Note that these are the same as used by VTK
static constexpr unsigned Void = 0; // not capitalized, as windows defines: #define VOID void, which causes trouble
static constexpr unsigned SIGNED_CHAR = 2;
static constexpr unsigned UNSIGNED_CHAR = 3;
static constexpr unsigned SIGNED_SHORT = 4;
static constexpr unsigned UNSIGNED_SHORT = 5;
static constexpr unsigned SIGNED_INT = 6;
static constexpr unsigned UNSIGNED_INT = 7;
static constexpr unsigned SIGNED_LONG = 8;
static constexpr unsigned UNSIGNED_LONG = 9;
static constexpr unsigned FLOAT = 10;
static constexpr unsigned DOUBLE = 11;

template <class T>
unsigned
GetDataTypeId();

template <>
inline unsigned
GetDataTypeId<signed char>()
{
  return SIGNED_CHAR;
}
template <>
inline unsigned
GetDataTypeId<unsigned char>()
{
  return UNSIGNED_CHAR;
}
template <>
inline unsigned
GetDataTypeId<signed short>()
{
  return SIGNED_SHORT;
}
template <>
inline unsigned
GetDataTypeId<unsigned short>()
{
  return UNSIGNED_SHORT;
}
template <>
inline unsigned
GetDataTypeId<signed int>()
{
  return SIGNED_INT;
}
template <>
inline unsigned
GetDataTypeId<unsigned int>()
{
  return UNSIGNED_INT;
}
template <>
inline unsigned
GetDataTypeId<signed long>()
{
  return SIGNED_LONG;
}
template <>
inline unsigned
GetDataTypeId<unsigned long>()
{
  return UNSIGNED_LONG;
}
template <>
inline unsigned
GetDataTypeId<float>()
{
  return FLOAT;
}
template <>
inline unsigned
GetDataTypeId<double>()
{
  return DOUBLE;
}

/**
 * Standard deletor
 */
template <typename T>
using StdDeletor = std::default_delete<T>;

/**
 * Custom deletor functor used as default deletor in smart memory management
 */
template <typename T>
struct DefaultDeletor
{
  void
  operator()(T * t)
  {
    t->Delete();
  }
};

/**
 * Custom unique pointer
 */
template <typename T>
using UniquePtrType = std::unique_ptr<T, DefaultDeletor<T>>;

/**
 * Custom shared pointer
 */
template <typename T>
using SharedPtrType = std::shared_ptr<T>;

/**
 * Factory function for shared pointer
 */
template <typename T>
auto
MakeSharedPointer(T && t)
{
  return std::shared_ptr<T>{ std::forward<T>(t), DefaultDeletor<T>() };
}

template <typename T>
auto
MakeSharedPointer(T * t)
{
  return std::shared_ptr<T>{ t, DefaultDeletor<T>() };
}

/**
 * Standard unique pointer
 */
template <typename T>
using StdUniquePtrType = std::unique_ptr<T, StdDeletor<T>>;

/**
 * RAII enforcer
 */
template <typename Callable>
class StackUnwinder
{
private:
  Callable m_c;
  bool     m_do{ true };

public:
  StackUnwinder(Callable && c)
    : m_c{ std::move(c) }
  {}

  ~StackUnwinder()
  {
    if (m_do)
    {
      m_c();
    }
  }

  void
  set()
  {
    m_do = true;
  }

  void
  unset()
  {
    m_do = false;
  }
};

template <typename Callable>
auto
MakeStackUnwinder(Callable && c)
{
  return StackUnwinder(std::forward<Callable>(c));
}

template <char D>
struct WordDelimiter : public std::string
{};

template <char D>
std::istream &
operator>>(std::istream & is, statismo::WordDelimiter<D> & output)
{
  std::getline(is, output, D);
  return is;
}

} // namespace statismo

#endif
