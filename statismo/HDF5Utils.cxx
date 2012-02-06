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

#ifndef __HDF5_UTILS_CXX
#define __HDF5_UTILS_CXX

#include "HDF5Utils.h"
#include "CommonTypes.h"
#include "Exceptions.h"
#include "H5Cpp.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iterator>

namespace statismo {

using namespace H5;


inline
void HDF5Utils::readMatrix(const H5::CommonFG& fg, const char* name, MatrixType& matrix) {
	DataSet ds = fg.openDataSet( name );
	hsize_t dims[2];
	ds.getSpace().getSimpleExtentDims(dims, NULL);

	// simply read the whole dataspace
	matrix.resize(dims[0], dims[1]);
	ds.read(matrix.data(), H5::PredType::NATIVE_FLOAT);
}


inline
void HDF5Utils::readMatrix(const H5::CommonFG& fg, const char* name, unsigned maxNumColumns, MatrixType& matrix) {
	DataSet ds = fg.openDataSet( name );
	hsize_t dims[2];
	ds.getSpace().getSimpleExtentDims(dims, NULL);

	hsize_t nRows = dims[0]; // take the number of rows defined in the hdf5 file
	hsize_t nCols = std::min(dims[1], static_cast<hsize_t>(maxNumColumns)); // take the number of cols provided by the user

	hsize_t offset[2] = {0,0};   // hyperslab offset in the file
	hsize_t count[2];
	count[0] = nRows; count[1] =  nCols;

	DataSpace dataspace = ds.getSpace();
	dataspace.selectHyperslab( H5S_SELECT_SET, count, offset );

	/* Define the memory dataspace. */
	hsize_t     dimsm[2];
	dimsm[0] = nRows; dimsm[1] = nCols;
	DataSpace memspace( 2, dimsm );

	/* Define memory hyperslab. */
	hsize_t      offset_out[2] = {0, 0};       // hyperslab offset in memory
	hsize_t      count_out[2];        // size of the hyperslab in memory

	count_out[0]  = nRows; count_out[1] = nCols;
	memspace.selectHyperslab( H5S_SELECT_SET, count_out, offset_out );

	matrix.resize(nRows, nCols);
	ds.read(matrix.data(), H5::PredType::NATIVE_FLOAT, memspace, dataspace);

}

inline
void HDF5Utils::writeMatrix(const H5::CommonFG& fg, const char* name, const MatrixType& matrix) {
	// HDF5 does not like empty matrices.
	//
	if (matrix.rows() == 0 || matrix.cols() == 0) {
		throw StatisticalModelException("Empty matrix provided to writeMatrix");
	}

	hsize_t dims[2] = {matrix.rows(), matrix.cols()};
	DataSet ds = fg.createDataSet( name, H5::PredType::NATIVE_FLOAT, DataSpace(2, dims));
	ds.write( matrix.data(), H5::PredType::NATIVE_FLOAT );
}

inline
void HDF5Utils::readVector(const H5::CommonFG& fg, const char* name, VectorType& vector) {
	DataSet ds = fg.openDataSet( name );
	hsize_t dims[1];
	ds.getSpace().getSimpleExtentDims(dims, NULL);
	vector.resize(dims[0], 1);
	ds.read(vector.data(), H5::PredType::NATIVE_FLOAT);
}

inline
void HDF5Utils::readVector(const H5::CommonFG& fg, const char* name, unsigned maxNumElements, VectorType& vector) {
	DataSet ds = fg.openDataSet( name );
	hsize_t dims[1];
	ds.getSpace().getSimpleExtentDims(dims, NULL);

	hsize_t nElements = std::min(dims[0], static_cast<hsize_t>(maxNumElements)); // take the number of rows defined in the hdf5 file

	hsize_t offset[1] = {0};   // hyperslab offset in the file
	hsize_t count[1];
	count[0] = nElements;

	DataSpace dataspace = ds.getSpace();
	dataspace.selectHyperslab( H5S_SELECT_SET, count, offset );

	/* Define the memory dataspace. */
	hsize_t     dimsm[1];
	dimsm[0] = nElements;
	DataSpace memspace( 1, dimsm );

	/* Define memory hyperslab. */
	hsize_t      offset_out[1] = {0};       // hyperslab offset in memory
	hsize_t      count_out[1];        // size of the hyperslab in memory

	count_out[0]  = nElements;
	memspace.selectHyperslab( H5S_SELECT_SET, count_out, offset_out );

	vector.resize(nElements);
	ds.read(vector.data(), H5::PredType::NATIVE_FLOAT, memspace, dataspace);
}



inline
void HDF5Utils::writeVector(const H5::CommonFG& fg, const char* name, const VectorType& vector) {
	hsize_t dims[1] = {vector.size()};
	DataSet ds = fg.createDataSet( name, H5::PredType::NATIVE_FLOAT, DataSpace(1, dims));
	ds.write( vector.data(), H5::PredType::NATIVE_FLOAT );

}

inline
void HDF5Utils::writeString(const H5::CommonFG& fg, const char* name, const std::string& s) {
	StrType fls_type(0, s.length()); // 0 is a dummy argument
	DataSet ds = fg.createDataSet(name, fls_type, DataSpace(H5S_SCALAR));
	ds.write(s.data(), fls_type);
}

inline
std::string
HDF5Utils::readString(const H5::CommonFG& fg, const char* name) {
	StrType strdatatype(PredType::C_S1, 256);
	char strreadbuf[256];
	DataSet ds = fg.openDataSet(name);
	ds.read(strreadbuf, strdatatype);
	return std::string(strreadbuf);
}

inline
void HDF5Utils::writeStringAttribute(const H5::Group& fg, const char* name, const std::string& s) {
	StrType strdatatype(PredType::C_S1, 256);
	Attribute att = fg.createAttribute(name, strdatatype, DataSpace(H5S_SCALAR));
	att.write(strdatatype, s.data());
	att.close();
}

inline
std::string
HDF5Utils::readStringAttribute(const H5::Group& fg, const char* name) {
	StrType strdatatype(PredType::C_S1, 256);
	char strreadbuf[256];
	Attribute myatt_out = fg.openAttribute(name);
	myatt_out.read(strdatatype, strreadbuf);
	return std::string(strreadbuf);
}

inline
void HDF5Utils::writeInt(const H5::CommonFG& fg, const char* name, int value) {
	IntType fls_type(PredType::NATIVE_INT32); // 0 is a dummy argument
	DataSet ds = fg.createDataSet(name, fls_type, DataSpace(H5S_SCALAR));
	ds.write(&value, fls_type);
}

inline
int HDF5Utils::readInt(const H5::CommonFG& fg, const char* name) {
	IntType fls_type(PredType::NATIVE_INT32);
	DataSet ds = fg.openDataSet( name );

	int value = 0;
	ds.read(&value, fls_type);
	return value;
}

inline
void HDF5Utils::writeFloat(const H5::CommonFG& fg, const char* name, float value) {
	FloatType fls_type(PredType::NATIVE_FLOAT); // 0 is a dummy argument
	DataSet ds = fg.createDataSet(name, fls_type, DataSpace(H5S_SCALAR));
	ds.write(&value, fls_type);
}

inline
float HDF5Utils::readFloat(const H5::CommonFG& fg, const char* name) {
	FloatType fls_type(PredType::NATIVE_FLOAT);
	DataSet ds = fg.openDataSet( name );

	float value = 0;
	ds.read(&value, fls_type);
	return value;
}

inline
void HDF5Utils::getFileFromHDF5(const H5::CommonFG& fg, const char* name, const char* filename) {
	DataSet ds = fg.openDataSet( name );
	hsize_t dims[1];
	ds.getSpace().getSimpleExtentDims(dims, NULL);
	std::vector<char> buffer(dims[0]);
	if(!buffer.empty()) ds.read(&buffer[0], H5::PredType::NATIVE_CHAR);

	typedef std::ostream_iterator<char> ostream_iterator;
	std::ofstream ofile(filename, std::ios::binary);
	if (!ofile) {
		std::string s= std::string("could not open file ") +filename;
		throw StatisticalModelException(s.c_str());
	}

	std::copy(buffer.begin(), buffer.end(), ostream_iterator(ofile));
	ofile.close();
}

inline
void
HDF5Utils::dumpFileToHDF5( const char* filename, const H5::CommonFG& fg, const char* name) {

	typedef std::istream_iterator<char> istream_iterator;

	std::ifstream ifile(filename, std::ios::binary);
	if (!ifile) {
		std::string s= std::string("could not open file ") +filename;
		throw StatisticalModelException(s.c_str());
	}

	std::vector<char> buffer;
	ifile >> std::noskipws;
	std::copy(istream_iterator(ifile), istream_iterator(), std::back_inserter(buffer));

	ifile.close();

	hsize_t dims[] = {buffer.size()};
	DataSet ds = fg.createDataSet( name,  H5::PredType::NATIVE_CHAR, DataSpace(1, dims));
	ds.write( &buffer[0], H5::PredType::NATIVE_CHAR );

}

} //namespace statismo

#endif
