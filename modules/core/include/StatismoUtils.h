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


#ifndef __UTILS_H_
#define __UTILS_H_

#include <cstdlib>
#include <ctime>
#include <cctype>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <list>

#include <random>

// TODO: replace by std::filesystem in the future
#include <cstdio>
#include <stdexcept>

#ifdef _WIN32
#  define NOMINMAX // avoid including the min and max macro
#  include <windows.h>
#  include <tchar.h>
#endif

#include "CommonTypes.h"
#include "Exceptions.h"

namespace statismo
{

/**
 * \brief A number of small utility functions - internal use only.
 */


#ifdef _MSC_VER
#  define is_deprecated __declspec(deprecated)
#elif defined(__GNUC__)
#  define is_deprecated __attribute__((deprecated))
#else
#  define is_deprecated // uncommon compiler, don't bother
#endif

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

namespace rand
{
inline std::minstd_rand &
randGen(unsigned seed = static_cast<unsigned>(time(0)))
{
  static std::minstd_rand rg{ seed };
  return rg;
}
} // namespace rand

class Utils
{
public:
  /**
   * return string representation of t
   */
  template <class T>
  static std::string
  toString(T t)
  {
    std::ostringstream os;
    os << t;
    return os.str();
  }


  /** return a N(0,1) vector of size n */
  static VectorType
  generateNormalVector(unsigned n)
  {
    // we would like to use tr1 here as well, but on some versions of visual studio it hangs forever.
    // therefore we use the functionality from boost

    // we make the random generate static, to ensure that the seed is only set once, and not with
    // every call
    static std::normal_distribution<> dist(0, 1);
    static auto                       r = std::bind(dist, rand::randGen());

    VectorType v = VectorType::Zero(n);
    for (unsigned i = 0; i < n; i++)
    {
      v(i) = r();
    }
    return v;
  }


  static VectorType
  ReadVectorFromTxtFile(const char * name)
  {
    typedef std::list<statismo::ScalarType> ListType;
    std::list<statismo::ScalarType>         values;
    std::ifstream                           inFile(name, std::ios::in);
    if (inFile.good())
    {
      std::copy(std::istream_iterator<statismo::ScalarType>(inFile),
                std::istream_iterator<statismo::ScalarType>(),
                std::back_insert_iterator<ListType>(values));
      inFile.close();
    }
    else
    {
      throw StatisticalModelException((std::string("Could not read text file ") + name).c_str());
    }

    VectorType v = VectorType::Zero(values.size());
    unsigned   i = 0;
    for (ListType::const_iterator it = values.begin(); it != values.end(); ++it)
    {
      v(i) = *it;
      i++;
    }
    return v;
  }

  // https://github.com/statismo/statismo/pull/268/files
  static std::string
  CreateTmpName(const std::string & extension)
  {
    // TODO: replace by std::filesystem in the future
    // Note (taken from PR Updated statismo to use C++11
    // imitates the path that was generated by boost::filesystem::unique_path to make sure we don't break anything
    static const char                      pathChars[] = "0123456789abcdefghiklmnopqrstuvwxyz";
    static std::uniform_int_distribution<> randIndex(
      0, sizeof(pathChars) - 2); //-1 for the \0 terminator and -1 because the index starts at 0

    std::string mask = "%%%%-%%%%-%%%%-%%%%";
    for (std::string::iterator iter = mask.begin(); iter != mask.end(); ++iter)
    {
      if (*iter == '%')
      {
        *iter = pathChars[randIndex(rand::randGen())];
      }
    }

    return mask + extension;
  }

  static void
  ToLower(std::string & str)
  {
    std::transform(std::begin(str), std::end(str), std::begin(str), [](unsigned char c) { return std::tolower(c); });
  }

  static std::string
  ToLowerCopy(const std::string & str)
  {
    std::string dup;
    ToLower(dup);
    return dup;
  }

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

  static void
  RemoveFile(const std::string & str)
  {
    std::remove(str.c_str());
  }

  template <char D>
  struct WordDelimiter : public std::string
  {};

  template <char D>
  static auto
  Split(const std::string & in)
  {
    std::istringstream       iss(in);
    std::vector<std::string> vec{ std::istream_iterator<WordDelimiter<D>>{ iss },
                                  std::istream_iterator<WordDelimiter<D>>{} };
    return vec;
  }
};

template <char D>
std::istream &
operator>>(std::istream & is, Utils::WordDelimiter<D> & output)
{
  std::getline(is, output, D);
  return is;
}

} // namespace statismo

#endif /* __UTILS_H_ */
