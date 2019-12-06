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

#ifndef __STATISMO_UTILS_H_
#define __STATISMO_UTILS_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Exceptions.h"

#ifdef _WIN32
#  define NOMINMAX // avoid including the min and max macro
#  include <windows.h>
#  include <tchar.h>
#endif

#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>
#include <iterator>

namespace statismo
{

/**
 * \brief A number of small utility functions - internal use only.
 */

namespace details
{
template <typename T>
inline T
LexicalCast(const std::string &)
{
  throw std::bad_cast();
}

template <>
inline double
LexicalCast<double>(const std::string & str)
{
  return std::stod(str);
}

template <>
inline float
LexicalCast<float>(const std::string & str)
{
  return std::stof(str);
}

template <>
inline int
LexicalCast<int>(const std::string & str)
{
  return std::stoi(str);
}

template <>
inline long
LexicalCast<long>(const std::string & str)
{
  return std::stol(str);
}

template <>
inline long long
LexicalCast<long long>(const std::string & str)
{
  return std::stoll(str);
}

template <>
inline unsigned int
LexicalCast<unsigned int>(const std::string & str)
{
  return std::stoul(str);
}

template <>
inline unsigned long
LexicalCast<unsigned long>(const std::string & str)
{
  return std::stoul(str);
}

template <>
inline unsigned long long
LexicalCast<unsigned long long>(const std::string & str)
{
  return std::stoull(str);
}
} // namespace details

namespace utils
{
/** return a N(0,1) vector of size n */
VectorType
GenerateNormalVector(unsigned n);

VectorType
ReadVectorFromTxtFile(const char * name);

std::string
CreateTmpName(const std::string & extension);

void
RemoveFile(const std::string & str);

void
ToLower(std::string & str);

std::string
ToLowerCopy(std::string str);

template <typename T>
static T
LexicalCast(const std::string & str)
{
  try
  {
    return details::LexicalCast<T>(str);
  }
  catch (...)
  {
    throw std::bad_cast();
  }
}

template <char D>
static auto
Split(const std::string & in)
{
  std::istringstream       iss(in);
  std::vector<std::string> vec{ std::istream_iterator<WordDelimiter<D>>{ iss },
                                std::istream_iterator<WordDelimiter<D>>{} };
  return vec;
}
} // namespace utils

} // namespace statismo

namespace std
{
inline string
to_string(const statismo::MatrixType & t)
{
  ostringstream os;
  os << t;
  return os.str();
}
} // namespace std

#endif
