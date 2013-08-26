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


#ifndef __ITK_TYPE_CONVERSION_TRAIT
#define __ITK_TYPE_CONVERSION_TRAIT

#include "statismo/Exceptions.h"

namespace itk {
typedef statismo::GenericEigenType<double>::VectorType DoubleVectorType;
typedef statismo::GenericEigenType<double>::MatrixType DoubleMatrixType;

// these traits are used to allow a conversion from the generic pixel type to a statismo vector.
// Currently only scalar types are supported.

template <typename T> struct PixelConversionTrait {
	static DoubleVectorType ToVector(const T& pixel) { throw statismo::StatisticalModelException("Unsupported PixelType (PixelTraits::ToVector not implemented)"); }
	static  T FromVector(const DoubleVectorType& v) {throw statismo::StatisticalModelException("Unsupported PixelType (PixelTraits::ToVector not implemented)"); }
	static unsigned GetDataType() { throw statismo::StatisticalModelException("Unsupported PixelType (PixelTraits::ToVector not implemented)"); }
	static unsigned GetPixelDimension() { throw statismo::StatisticalModelException("Unsupported PixelType (PixelTraits::ToVector not implemented)"); }
};

template <> struct PixelConversionTrait<double> {
	static DoubleVectorType ToVector(const double& pixel) { DoubleVectorType v(1); v << pixel; return v;}
	static  double FromVector(const DoubleVectorType& v) { assert(v.size() == 1); return v(0); }
	static  unsigned  GetDataType() {return statismo::DOUBLE; }
	static unsigned GetPixelDimension() { return 1;}
};
template <> struct PixelConversionTrait<float> {
	static DoubleVectorType ToVector(const float& pixel) { DoubleVectorType v(1); v << pixel; return v;}
	static  float FromVector(const DoubleVectorType& v) { assert(v.size() == 1); return v(0); }
	static  unsigned  GetDataType() {return statismo::FLOAT; }
	static unsigned GetPixelDimension() { return 1;}
};
template <> struct PixelConversionTrait<short> {
	static DoubleVectorType ToVector(const short& pixel) { DoubleVectorType v(1); v << pixel; return v;}
	static  short FromVector(const DoubleVectorType& v) { assert(v.size() == 1); return v(0); }
	static  unsigned  GetDataType() {return statismo::SIGNED_SHORT; }
	static unsigned GetPixelDimension() { return 1;}
};
template <> struct PixelConversionTrait<unsigned short> {
	static DoubleVectorType ToVector(const unsigned short& pixel) { DoubleVectorType v(1); v << pixel; return v;}
	static  unsigned short FromVector(const DoubleVectorType& v) { assert(v.size() == 1); return v(0); }
	static  unsigned  GetDataType() {return statismo::UNSIGNED_SHORT; }
	static unsigned GetPixelDimension() { return 1;}
};
template <> struct PixelConversionTrait<int> {
	static DoubleVectorType ToVector(const int& pixel) { DoubleVectorType v(1); v << pixel; return v;}
	static  int FromVector(const DoubleVectorType& v) { assert(v.size() == 1); return v(0); }
	static  unsigned  GetDataType() {return statismo::SIGNED_INT; }
	static unsigned GetPixelDimension() { return 1;}
};
template <> struct PixelConversionTrait<unsigned int> {
	static DoubleVectorType ToVector(const unsigned int& pixel) { DoubleVectorType v(1); v << pixel; return v;}
	static  unsigned int FromVector(const DoubleVectorType& v) { assert(v.size() == 1); return v(0); }
	static  unsigned  GetDataType() {return statismo::UNSIGNED_SHORT; }
	static unsigned GetPixelDimension() { return 1;}
};
template <> struct PixelConversionTrait<char> {
	static DoubleVectorType ToVector(const char& pixel) { DoubleVectorType v(1); v << pixel; return v;}
	static char FromVector(const DoubleVectorType& v) { assert(v.size() == 1); return v(0); }
	static  unsigned  GetDataType() {return statismo::SIGNED_CHAR; }
	static unsigned GetPixelDimension() { return 1;}
};
template <> struct PixelConversionTrait<unsigned char> {
	static DoubleVectorType ToVector(const unsigned char& pixel) { DoubleVectorType v(1); v << pixel; return v;}
	static  unsigned char FromVector(const DoubleVectorType& v) { assert(v.size() == 1); return v(0); }
	static  unsigned  GetDataType() {return statismo::UNSIGNED_CHAR; }
	static unsigned GetPixelDimension() { return 1;}
};

template <> struct PixelConversionTrait<long> {
	static DoubleVectorType ToVector(const long& pixel) { DoubleVectorType v(1); v << pixel; return v;}
	static long FromVector(const DoubleVectorType& v) { assert(v.size() == 1); return v(0); }
	static  unsigned  GetDataType() {return statismo::SIGNED_LONG; }
	static unsigned GetPixelDimension() { return 1;}
};
template <> struct PixelConversionTrait<unsigned long> {
	static DoubleVectorType ToVector(const unsigned long& pixel) { DoubleVectorType v(1); v << pixel; return v;}
	static  unsigned long FromVector(const DoubleVectorType& v) { assert(v.size() == 1); return v(0); }
	static  unsigned  GetDataType() {return statismo::UNSIGNED_LONG; }
	static unsigned GetPixelDimension() { return 1;}
};

template <> struct PixelConversionTrait<itk::Vector<float, 2> > {
	static DoubleVectorType ToVector(const itk::Vector<float, 2>& pixel) {
		DoubleVectorType v(2);
		v << pixel[0] , pixel[1];
		return v;
	}
	static itk::Vector<float, 2> FromVector(const DoubleVectorType& v) {
		assert(v.size() == 2);
		itk::Vector<double, 2> itkVec;
		itkVec[0] = v(0); itkVec[1] = v(1);
		return itkVec;
	}
	static  unsigned  GetDataType() {return statismo::FLOAT; }
	static unsigned GetPixelDimension() { return 2;}
};

template <> struct PixelConversionTrait<itk::Vector<float, 3> > {
	static DoubleVectorType ToVector(const itk::Vector<float, 3>& pixel) {
		DoubleVectorType v(3);
		v << pixel[0] , pixel[1], pixel[2];
		return v;
	}
	static itk::Vector<float, 3> FromVector(const DoubleVectorType& v) {
		assert(v.size() == 3);
		itk::Vector<double, 3> itkVec;
		itkVec[0] = v(0); itkVec[1] = v(1); itkVec[2] = v(2);
		return itkVec;
	}
	static  unsigned  GetDataType() {return statismo::FLOAT; }
	static unsigned GetPixelDimension() { return 3;}
};


} // namespace itk

#endif
