/* 
 * The Biomechanical ToolKit
 * Copyright (c) 2009-2013, Arnaud Barré
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name(s) of the copyright holders nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
%{
  #define SWIG_FILE_WITH_INIT
  #include <Eigen/Core>
%}

%include "numpy.i"

%init
%{
  import_array();
%}

%fragment("Eigen_Fragments", "header",  fragment="NumPy_Fragments")
%{
  template <typename T> int NumPyType() {return -1;};
  
  template <class Derived>
  void ConvertFromNumpyToEigenMatrix(Eigen::MatrixBase<Derived>* out, PyObject* in)
  {
    int rows = 0;
    int cols = 0;
    // Check object type
    if (!is_array(in))
    {
      PyErr_SetString(PyExc_ValueError, "The given input is not known as a NumPy array or matrix.");
      return;
    }
    // Check data type
    else if (array_type(in) != NumPyType<typename Derived::Scalar>())
    {
      PyErr_SetString(PyExc_ValueError, "Type mismatch between NumPy and Eigen objects.");
      return;
    }
    // Check dimensions
    else if (array_numdims(in) > 2)
    {
      PyErr_SetString(PyExc_ValueError, "Eigen only support 1D or 2D array.");
      return;
    }
    else if (array_numdims(in) == 1)
    {
      rows = array_size(in,0);
      cols = 1;
      if ((Derived::RowsAtCompileTime != Eigen::Dynamic) && (Derived::RowsAtCompileTime != rows))
      {
        PyErr_SetString(PyExc_ValueError, "Row dimension mismatch between NumPy and Eigen objects (1D).");
        return;
      }
      else if ((Derived::ColsAtCompileTime != Eigen::Dynamic) && (Derived::ColsAtCompileTime != 1))
      {
        PyErr_SetString(PyExc_ValueError, "Column dimension mismatch between NumPy and Eigen objects (1D).");
        return;
      }
    }
    else if (array_numdims(in) == 2)
    {
      rows = array_size(in,0);
      cols = array_size(in,1);
      if ((Derived::RowsAtCompileTime != Eigen::Dynamic) && (Derived::RowsAtCompileTime != array_size(in,0)))
      {
        PyErr_SetString(PyExc_ValueError, "Row dimension mismatch between NumPy and Eigen objects (2D).");
        return;
      }
      else if ((Derived::ColsAtCompileTime != Eigen::Dynamic) && (Derived::ColsAtCompileTime != array_size(in,1)))
      {
        PyErr_SetString(PyExc_ValueError, "Column dimension mismatch between NumPy and Eigen objects (2D).");
        return;
      }
    }
    // Extract data
    int isNewObject = 0;
    PyArrayObject* temp = obj_to_array_contiguous_allow_conversion(in, array_type(in), &isNewObject);
    if (temp == NULL)
    {
      PyErr_SetString(PyExc_ValueError, "Impossible to convert the input into a Python array object.");
      return;
    }
    out->derived().setZero(rows, cols);
    typename Derived::Scalar* data = static_cast<typename Derived::Scalar*>(PyArray_DATA(temp));
    for (int i = 0; i != rows; ++i)
      for (int j = 0; j != cols; ++j)
        out->coeffRef(i,j) = data[i*cols+j];
  };
  
  template <class Derived>
  void ConvertFromEigenToNumPyMatrix(PyObject** out, Eigen::MatrixBase<Derived>* in)
  {
    npy_intp dims[2] = {in->rows(), in->cols()};
    *out = PyArray_SimpleNew(2, dims, NumPyType<typename Derived::Scalar>());
    typename Derived::Scalar* data = static_cast<typename Derived::Scalar*>(PyArray_DATA(*out));
    for (int i = 0; i != dims[0]; ++i)
      for (int j = 0; j != dims[1]; ++j)
        data[i*dims[1]+j] = in->coeff(i,j);
  };
  
  template<> int NumPyType<double>() {return PyArray_DOUBLE;};
%}

// ----------------------------------------------------------------------------
// Macro to create the typemap for Eigen classes
// ----------------------------------------------------------------------------
%define %eigen_typemaps(CLASS)

// In: (nothing: no constness)
%typemap(in, fragment="Eigen_Fragments") CLASS (CLASS temp)
{
  ConvertFromNumpyToEigenMatrix<CLASS>(&temp, $input);
  $1 = temp;
}
// In: const&
%typemap(in, fragment="Eigen_Fragments") CLASS const& (CLASS temp)
{
  ConvertFromNumpyToEigenMatrix<CLASS>(&temp, $input);
  $1 = &temp;
}
// In: & (not yet implemented)
%typemap(in, fragment="Eigen_Fragments") CLASS &
{
  PyErr_SetString(PyExc_ValueError, "The input typemap for non-const reference is not yet implemented. Please report this problem to the developer.");
}
// In: const* (not yet implemented)
%typemap(in, fragment="Eigen_Fragments") CLASS const*
{
  PyErr_SetString(PyExc_ValueError, "The input typemap for const pointer is not yet implemented. Please report this problem to the developer.");
}
// In: * (not yet implemented)
%typemap(in, fragment="Eigen_Fragments") CLASS *
{
  PyErr_SetString(PyExc_ValueError, "The input typemap for non-const pointer is not yet implemented. Please report this problem to the developer.");
}

// Out: (nothing: no constness)
%typemap(out, fragment="Eigen_Fragments") CLASS
{
  ConvertFromEigenToNumPyMatrix<CLASS>(&$result, &$1);
}
// Out: const
%typemap(out, fragment="Eigen_Fragments") CLASS const
{
  ConvertFromEigenToNumPyMatrix<CLASS>(&$result, &$1);
}
// Out: const&
%typemap(out, fragment="Eigen_Fragments") CLASS const&
{
  ConvertFromEigenToNumPyMatrix<CLASS>(&$result, $1);
}
// Out: & (not yet implemented)
%typemap(out, fragment="Eigen_Fragments") CLASS &
{
  PyErr_SetString(PyExc_ValueError, "The output typemap for non-const reference is not yet implemented. Please report this problem to the developer.");
}
// Out: const* (not yet implemented)
%typemap(out, fragment="Eigen_Fragments") CLASS const*
{
  PyErr_SetString(PyExc_ValueError, "The output typemap for const pointer is not yet implemented. Please report this problem to the developer.");
}
// Out: * (not yet implemented)
%typemap(out, fragment="Eigen_Fragments") CLASS *
{
  PyErr_SetString(PyExc_ValueError, "The output typemap for non-const pointer is not yet implemented. Please report this problem to the developer.");
}

%enddef
