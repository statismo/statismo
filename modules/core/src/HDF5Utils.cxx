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
#include "statismo/core/HDF5Utils.h"
#include "statismo/core/CommonTypes.h"
#include "statismo/core/Exceptions.h"

#include <H5Cpp.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

namespace statismo::hdf5utils
{

H5::H5File
OpenOrCreateFile(const std::string & filename)
{

  // check if file exists
  std::ifstream ifile(filename.c_str());
  H5::H5File    file;

  if (!ifile)
  {
    // create it
    file = H5::H5File(filename.c_str(), H5F_ACC_EXCL);
  }
  else
  {
    // open it
    file = H5::H5File(filename.c_str(), H5F_ACC_RDWR);
  }

  return file;
}

H5::Group
OpenPath(H5::H5File & file, const std::string & path, bool createPath)
{
  H5::Group group;

  // take the first part of the path
  std::size_t curpos = 1;
  std::size_t nextpos = path.find_first_of("/", curpos);
  auto        g = file.openGroup("/");
  auto        name = path.substr(curpos, nextpos - 1);

  while (curpos != std::string::npos && !name.empty())
  {
    if (ExistsObjectWithName(g, name))
    {
      g = g.openGroup(name);
    }
    else
    {
      if (createPath)
      {
        g = g.createGroup(name);
      }
      else
      {
        std::string msg = std::string("the path ") + path + " does not exist";
        throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
      }
    }

    curpos = nextpos + 1;
    nextpos = path.find_first_of("/", curpos);
    if (nextpos != std::string::npos)
    {
      name = path.substr(curpos, nextpos - curpos);
    }
    else
    {
      name = path.substr(curpos);
    }
  }

  return g;
}

void
ReadMatrix(const H5::H5Location & fg, const char * name, MatrixType & matrix)
{
  ReadMatrixOfType<ScalarType>(fg, name, matrix);
}

void
ReadMatrix(const H5::H5Location & fg, const char * name, unsigned maxNumColumns, MatrixType & matrix)
{
  auto    ds = fg.openDataSet(name);
  hsize_t dims[2];
  ds.getSpace().getSimpleExtentDims(dims, NULL);

  auto nRows = dims[0]; // take the number of rows defined in the hdf5 file
  auto nCols = std::min(dims[1], static_cast<hsize_t>(maxNumColumns)); // take the number of cols provided by the user

  hsize_t offset[2] = { 0, 0 }; // hyperslab offset in the file
  hsize_t count[2];
  count[0] = nRows;
  count[1] = nCols;

  H5::DataSpace dataspace = ds.getSpace();
  dataspace.selectHyperslab(H5S_SELECT_SET, count, offset);

  /* Define the memory dataspace. */
  hsize_t dimsm[2];
  dimsm[0] = nRows;
  dimsm[1] = nCols;
  H5::DataSpace memspace(2, dimsm);

  /* Define memory hyperslab. */
  hsize_t offset_out[2] = { 0, 0 }; // hyperslab offset in memory
  hsize_t count_out[2];             // size of the hyperslab in memory

  count_out[0] = nRows;
  count_out[1] = nCols;
  memspace.selectHyperslab(H5S_SELECT_SET, count_out, offset_out);

  matrix.resize(nRows, nCols);
  ds.read(matrix.data(), H5::PredType::NATIVE_FLOAT, memspace, dataspace);
}

H5::DataSet
WriteMatrix(const H5::H5Location & fg, const char * name, const MatrixType & matrix)
{
  return WriteMatrixOfType<ScalarType>(fg, name, matrix);
}

void
ReadVector(const H5::H5Location & fg, const char * name, VectorType & vector)
{
  ReadVectorOfType<ScalarType>(fg, name, vector);
}

void
ReadVector(const H5::H5Location & fg, const char * name, unsigned maxNumElements, VectorType & vector)
{
  H5::DataSet ds = fg.openDataSet(name);
  hsize_t     dims[1];
  ds.getSpace().getSimpleExtentDims(dims, NULL);

  hsize_t nElements =
    std::min(dims[0], static_cast<hsize_t>(maxNumElements)); // take the number of rows defined in the hdf5 file

  hsize_t offset[1] = { 0 }; // hyperslab offset in the file
  hsize_t count[1];
  count[0] = nElements;

  H5::DataSpace dataspace = ds.getSpace();
  dataspace.selectHyperslab(H5S_SELECT_SET, count, offset);

  /* Define the memory dataspace. */
  hsize_t dimsm[1];
  dimsm[0] = nElements;
  H5::DataSpace memspace(1, dimsm);

  /* Define memory hyperslab. */
  hsize_t offset_out[1] = { 0 }; // hyperslab offset in memory
  hsize_t count_out[1];          // size of the hyperslab in memory

  count_out[0] = nElements;
  memspace.selectHyperslab(H5S_SELECT_SET, count_out, offset_out);

  vector.resize(nElements);
  ds.read(vector.data(), H5::PredType::NATIVE_FLOAT, memspace, dataspace);
}

H5::DataSet
WriteVector(const H5::H5Location & fg, const char * name, const VectorType & vector)
{
  return WriteVectorOfType<ScalarType>(fg, name, vector);
}

H5::DataSet
WriteString(const H5::H5Location & fg, const char * name, const std::string & s)
{
  H5::StrType flsType(H5::PredType::C_S1, s.length() + 1); // + 1 for trailing zero
  H5::DataSet ds = fg.createDataSet(name, flsType, H5::DataSpace(H5S_SCALAR));
  ds.write(s, flsType);
  return ds;
}

std::string
ReadString(const H5::H5Location & fg, const char * name)
{
  H5std_string outputString;
  H5::DataSet  ds = fg.openDataSet(name);
  ds.read(outputString, ds.getStrType());
  return outputString;
}

void
WriteStringAttribute(const H5::H5Object & fg, const char * name, const std::string & s)
{
  H5::StrType   strdatatype(H5::PredType::C_S1, s.length() + 1); // + 1 for trailing 0
  H5::Attribute att = fg.createAttribute(name, strdatatype, H5::DataSpace(H5S_SCALAR));
  att.write(strdatatype, s);
}

std::string
ReadStringAttribute(const H5::H5Object & fg, const char * name)
{
  H5std_string outputString;

  H5::Attribute attOut = fg.openAttribute(name);
  attOut.read(attOut.getStrType(), outputString);
  return outputString;
}

void
WriteIntAttribute(const H5::H5Object & fg, const char * name, int value)
{
  H5::IntType   intType(H5::PredType::NATIVE_INT32);
  H5::DataSpace attSpace(H5S_SCALAR);
  H5::Attribute att = fg.createAttribute(name, intType, attSpace);
  att.write(intType, &value);
}

int
ReadIntAttribute(const H5::H5Object & fg, const char * name)
{
  H5::IntType   flsType(H5::PredType::NATIVE_INT32);
  int           value = 0;
  H5::Attribute attOut = fg.openAttribute(name);
  attOut.read(flsType, &value);
  return value;
}

H5::DataSet
WriteInt(const H5::H5Location & fg, const char * name, int value)
{
  H5::IntType flsType(H5::PredType::NATIVE_INT32); // 0 is a dummy argument
  H5::DataSet ds = fg.createDataSet(name, flsType, H5::DataSpace(H5S_SCALAR));
  ds.write(&value, flsType);
  return ds;
}

int
ReadInt(const H5::H5Location & fg, const char * name)
{
  H5::IntType flsType(H5::PredType::NATIVE_INT32);
  H5::DataSet ds = fg.openDataSet(name);

  int value = 0;
  ds.read(&value, flsType);
  return value;
}

H5::DataSet
WriteFloat(const H5::H5Location & fg, const char * name, float value)
{
  H5::FloatType flsType(H5::PredType::NATIVE_FLOAT); // 0 is a dummy argument
  H5::DataSet   ds = fg.createDataSet(name, flsType, H5::DataSpace(H5S_SCALAR));
  ds.write(&value, flsType);
  return ds;
}

float
ReadFloat(const H5::H5Location & fg, const char * name)
{
  H5::FloatType flsType(H5::PredType::NATIVE_FLOAT);
  H5::DataSet   ds = fg.openDataSet(name);

  float value = 0;
  ds.read(&value, flsType);
  return value;
}

void
GetFileFromHDF5(const H5::H5Location & fg, const char * name, const char * filename)
{
  H5::DataSet ds = fg.openDataSet(name);
  hsize_t     dims[1];
  ds.getSpace().getSimpleExtentDims(dims, NULL);
  std::vector<char> buffer(dims[0]);
  if (!buffer.empty())
  {
    ds.read(&buffer[0], H5::PredType::NATIVE_CHAR);
  }

  using ostream_iterator = std::ostream_iterator<char>;

  std::ofstream ofile(filename, std::ios::binary);
  if (!ofile)
  {
    std::string s = std::string("could not open file ") + filename;
    throw StatisticalModelException(s.c_str(), Status::IO_ERROR);
  }

  std::copy(std::begin(buffer), std::end(buffer), ostream_iterator(ofile));
}

void
DumpFileToHDF5(const char * filename, const H5::H5Location & fg, const char * name)
{

  using istream_iterator = std::istream_iterator<char>;

  std::ifstream ifile(filename, std::ios::binary);
  if (!ifile)
  {
    std::string s = std::string("could not open file ") + filename;
    throw StatisticalModelException(s.c_str(), Status::IO_ERROR);
  }

  std::vector<char> buffer;
  ifile >> std::noskipws;
  std::copy(istream_iterator(ifile), istream_iterator(), std::back_inserter(buffer));

  hsize_t     dims[] = { buffer.size() };
  H5::DataSet ds = fg.createDataSet(name, H5::PredType::NATIVE_CHAR, H5::DataSpace(1, dims));
  ds.write(&buffer[0], H5::PredType::NATIVE_CHAR);
}

bool
ExistsObjectWithName(const H5::H5Location & fg, const std::string & name)
{
  for (hsize_t i = 0; i < fg.getNumObjs(); ++i)
  {
    if (fg.getObjnameByIdx(i) == name)
    {
      return true;
    }
  }
  return false;
}

} // namespace statismo::hdf5utils
