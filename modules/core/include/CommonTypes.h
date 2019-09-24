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

#ifndef __COMMON_TYPES_H
#define __COMMON_TYPES_H

#include <exception>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "Config.h"
#include "Domain.h"
#include "Exceptions.h"

namespace statismo {

const double PI	=	3.14159265358979323846;

/// the type that is used for all vector and matrices throughout the library.
typedef float ScalarType;

// wrapper struct that allows us to easily select matrix and vectors of an arbitrary
// type, wich has the same traits as the standard matrix / vector traits
template <typename TScalar> struct GenericEigenType {
    typedef Eigen::Matrix<TScalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixType;
    typedef Eigen::DiagonalMatrix<TScalar, Eigen::Dynamic> DiagMatrixType;
    typedef Eigen::Matrix<TScalar, Eigen::Dynamic, 1> VectorType;
    typedef Eigen::Matrix<TScalar, 1 , Eigen::Dynamic> RowVectorType;

};
typedef GenericEigenType<ScalarType>::MatrixType MatrixType;
typedef GenericEigenType<double>::MatrixType MatrixTypeDoublePrecision;
typedef GenericEigenType<ScalarType>::DiagMatrixType DiagMatrixType;
typedef GenericEigenType<ScalarType>::VectorType VectorType;
typedef GenericEigenType<double>::VectorType VectorTypeDoublePrecision;
typedef GenericEigenType<ScalarType>::RowVectorType RowVectorType;

// type definitions used in the standard file format.
// Note that these are the same as used by VTK
const static unsigned Void = 0; // not capitalized, as windows defines: #define VOID void, which causes trouble
const static unsigned SIGNED_CHAR = 2;
const static unsigned UNSIGNED_CHAR  = 3;
const static unsigned SIGNED_SHORT     =  4;
const static unsigned UNSIGNED_SHORT = 5;
const static unsigned SIGNED_INT             = 6;
const static unsigned UNSIGNED_INT   = 7;
const static unsigned SIGNED_LONG          =  8;
const static unsigned UNSIGNED_LONG  = 9;
const static unsigned FLOAT =         10;
const static unsigned DOUBLE      =   11;

template <class T> unsigned GetDataTypeId() {
    throw StatisticalModelException("The datatype that was provided is not a valid statismo data type ");
}
template <> inline unsigned GetDataTypeId<signed char>() {
    return SIGNED_CHAR;
}
template <> inline unsigned GetDataTypeId<unsigned char>() {
    return UNSIGNED_CHAR;
}
template <> inline unsigned GetDataTypeId<signed short>() {
    return SIGNED_SHORT;
}
template <> inline unsigned GetDataTypeId<unsigned short>() {
    return UNSIGNED_SHORT;
}
template <> inline unsigned GetDataTypeId<signed int>() {
    return SIGNED_INT;
}
template <> inline unsigned GetDataTypeId<unsigned int>() {
    return UNSIGNED_INT;
}
template <> inline unsigned GetDataTypeId<signed long>() {
    return SIGNED_LONG;
}
template <> inline unsigned GetDataTypeId<unsigned long>() {
    return UNSIGNED_LONG;
}
template <> inline unsigned GetDataTypeId<float>() {
    return FLOAT;
}
template <> inline unsigned GetDataTypeId<double>() {
    return DOUBLE;
}

template <typename T>
inline void hash_combine(std::size_t& seed, const T& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}

} //namespace statismo

namespace statismo::details
{
    // TODO: Move to MPL file
    // general compile-time validator (ref: C++ template the complete guide)

    template <typename F, typename... Args,
          typename = decltype(std::declval<F>()(std::declval<Args &&>()...))>
auto is_valid_impl(void *) -> std::true_type;

template <typename F, typename... Args>
auto is_valid_impl(...) -> std::false_type;
    inline constexpr auto is_valid = [](auto f) {
    using input_type = decltype(f);
    return [](auto &&... args) {
        return decltype(
            details::is_valid_impl<input_type, decltype(args) &&...>(nullptr)){};
    };
    };

    template <typename T> struct type_t { using type = T; };

template <typename T> constexpr auto type = type_t<T>{};

template <typename T> T value_t(type_t<T>);

    inline constexpr auto HasSize = is_valid(
      [](auto x) -> decltype((void)value_t(x).size()) {
      });

    inline constexpr auto HasGetPointDimension = is_valid(
      [](auto x) -> decltype((void)value_t(x).GetPointDimension()) {
      });

    template <typename T>
    inline auto HashImpl(const T &v) -> std::enable_if_t<details::HasSize(details::type<T>), size_t>
    {
        size_t value = 0;
        for (unsigned i = 0; i < v.size(); i++) {
            statismo::hash_combine(value, v(i));
        }
        return value;
    }

    template <typename T>
    inline auto HashImpl(const T &v) -> std::enable_if_t<details::HasGetPointDimension(details::type<T>), size_t>
    {
        size_t value = 0;
        for (unsigned i = 0; i < v.GetPointDimension(); i++) {
            statismo::hash_combine(value, v[i]);
        }
        return value;
    }
}

// Specific hash function (move it else where)
namespace statismo {
template<typename T>
class Hash {
public:

    size_t operator()(const T &v) const 
    {
       return details::HashImpl(v);
    }
 private:
};

}

#endif

