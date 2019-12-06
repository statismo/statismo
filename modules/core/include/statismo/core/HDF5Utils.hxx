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

#ifndef __STATIMO_CORE_HDF5_UTILS_HXX_
#define __STATIMO_CORE_HDF5_UTILS_HXX_

#include "statismo/core/HDF5Utils.h"
#include "statismo/core/CommonTypes.h"
#include "statismo/core/Exceptions.h"

#include <H5Cpp.h>

namespace statismo::hdf5utils
{
namespace details
{
template <typename Scalar>
struct HDF5PredTypeTraits;

template <>
struct HDF5PredTypeTraits<double>
{
  static const H5::PredType &
  GetPredRef()
  {
    return H5::PredType::NATIVE_DOUBLE;
  }
};

template <>
struct HDF5PredTypeTraits<float>
{
  static const H5::PredType &
  GetPredRef()
  {
    return H5::PredType::NATIVE_FLOAT;
  }
};

template <>
struct HDF5PredTypeTraits<unsigned int>
{
  static const H5::PredType &
  GetPredRef()
  {
    return H5::PredType::NATIVE_UINT;
  }
};

template <>
struct HDF5PredTypeTraits<int>
{
  static const H5::PredType &
  GetPredRef()
  {
    return H5::PredType::NATIVE_INT;
  }
};
} // namespace details

template <class T>
inline void
ReadMatrixOfType(const H5::H5Location & fg, const char * name, typename GenericEigenTraits<T>::MatrixType & matrix)
{
  H5::DataSet ds = fg.openDataSet(name);
  hsize_t     dims[2];
  ds.getSpace().getSimpleExtentDims(dims, NULL);

  // simply read the whole dataspace
  matrix.resize(dims[0], dims[1]);
  ds.read(matrix.data(), details::HDF5PredTypeTraits<T>::GetPredRef());
}

template <class T>
inline H5::DataSet
WriteMatrixOfType(const H5::H5Location &                             fg,
                  const char *                                       name,
                  const typename GenericEigenTraits<T>::MatrixType & matrix)
{
  // HDF5 does not like empty matrices.
  //
  if (matrix.rows() == 0 || matrix.cols() == 0)
  {
    throw StatisticalModelException("Empty matrix provided to writeMatrix", Status::INVALID_DATA_ERROR);
  }

  hsize_t     dims[2] = { static_cast<hsize_t>(matrix.rows()), static_cast<hsize_t>(matrix.cols()) };
  H5::DataSet ds = fg.createDataSet(name, details::HDF5PredTypeTraits<T>::GetPredRef(), H5::DataSpace(2, dims));
  ds.write(matrix.data(), details::HDF5PredTypeTraits<T>::GetPredRef());
  return ds;
}

template <class T>
inline void
ReadVectorOfType(const H5::H5Location & fg, const char * name, typename GenericEigenTraits<T>::VectorType & vector)
{
  H5::DataSet ds = fg.openDataSet(name);
  hsize_t     dims[1];
  ds.getSpace().getSimpleExtentDims(dims, NULL);
  vector.resize(dims[0], 1);
  ds.read(vector.data(), details::HDF5PredTypeTraits<T>::GetPredRef());
}

template <class T>
inline H5::DataSet
WriteVectorOfType(const H5::H5Location &                             fg,
                  const char *                                       name,
                  const typename GenericEigenTraits<T>::VectorType & vector)
{
  hsize_t     dims[1] = { static_cast<hsize_t>(vector.size()) };
  H5::DataSet ds = fg.createDataSet(name, details::HDF5PredTypeTraits<T>::GetPredRef(), H5::DataSpace(1, dims));
  ds.write(vector.data(), details::HDF5PredTypeTraits<T>::GetPredRef());
  return ds;
}

template <typename T>
inline void
ReadArray(const H5::H5Location & fg, const char * name, std::vector<T> & array)
{
  H5::DataSet ds = fg.openDataSet(name);
  hsize_t     dims[1];
  ds.getSpace().getSimpleExtentDims(dims, NULL);
  array.resize(dims[0]);
  ds.read(&array[0], details::HDF5PredTypeTraits<T>::GetPredRef());
}

template <typename T>
inline H5::DataSet
WriteArray(const H5::H5Location & fg, const char * name, std::vector<T> const & array)
{
  hsize_t     dims[1] = { array.size() };
  H5::DataSet ds = fg.createDataSet(name, details::HDF5PredTypeTraits<T>::GetPredRef(), H5::DataSpace(1, dims));
  ds.write(&array[0], details::HDF5PredTypeTraits<T>::GetPredRef());
  return ds;
}

} // namespace statismo::hdf5utils

#endif
