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

#include "Config.h"
#include "Domain.h"
#include "Exceptions.h"
#include <iostream>

#include <exception>
#include <string>
#include <list>
#include <vector>

#include <Eigen/Dense>



namespace statismo {


const double PI	=	3.14159265358979323846;

/// the type that is used for all vector and matrices throughout the library.
typedef float ScalarType;

// wrapper struct that allows us to easily select matrix and vectors of an arbitrary
// type, wich has the same traits as the standard matrix / vector traits
template <typename ScalarType> struct GenericEigenType {
	typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixType;
	typedef Eigen::DiagonalMatrix<ScalarType, Eigen::Dynamic> DiagMatrixType;
	typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> VectorType;
	typedef Eigen::Matrix<ScalarType, 1 , Eigen::Dynamic> RowVectorType;

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
const static unsigned  SIGNED_SHORT     =  4;
const static unsigned UNSIGNED_SHORT = 5;
const static unsigned SIGNED_INT             = 6;
const static unsigned UNSIGNED_INT   = 7;
const static unsigned  SIGNED_LONG          =  8;
const static unsigned  UNSIGNED_LONG  = 9;
const static unsigned FLOAT =         10;
const static unsigned DOUBLE      =   11;

template <class T> unsigned GetDataTypeId() {
	throw StatisticalModelException("The datatype that was provided is not a valid statismo data type ");
}
template <> unsigned GetDataTypeId<signed char>() { return SIGNED_CHAR; }
template <> unsigned GetDataTypeId<unsigned char>() { return UNSIGNED_CHAR; }
template <> unsigned GetDataTypeId<signed short>() { return SIGNED_SHORT; }
template <> unsigned GetDataTypeId<unsigned short>() { return UNSIGNED_SHORT; }
template <> unsigned GetDataTypeId<signed int>() { return SIGNED_INT; }
template <> unsigned GetDataTypeId<unsigned int>() { return UNSIGNED_INT; }
template <> unsigned GetDataTypeId<signed long>() { return SIGNED_LONG; }
template <> unsigned GetDataTypeId<unsigned long>() { return UNSIGNED_LONG; }
template <> unsigned GetDataTypeId<float>() { return FLOAT; }
template <> unsigned GetDataTypeId<double>() { return DOUBLE; }


} //namespace statismo

#endif

