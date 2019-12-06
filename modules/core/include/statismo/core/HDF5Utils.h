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

#ifndef __STATIMO_CORE_HDF5_UTILS_H_
#define __STATIMO_CORE_HDF5_UTILS_H_

#include "statismo/core/CommonTypes.h"

namespace H5
{
class H5Location;
class Group;
class H5File;
class H5Object;
class DataSet;
} // namespace H5

namespace statismo::hdf5utils
{
/**
 * Opens the hdf5 file with the given name, or creates it if the file does not exist
 */
H5::H5File
OpenOrCreateFile(const std::string & filename);

/**
 * Opens the hdf5 group or creates it if it doesn't exist.
 * @param fg A file object
 * @param path An absolute path that defines a group
 * @param createPath if true, creates the path if it does not exist
 *
 * @return the group object representing the path in the hdf5 file
 */
H5::Group
OpenPath(H5::H5File & fg, const std::string & path, bool createPath = false);

/**
 * Read a Matrix from a HDF5 File
 * @param fg The group
 * @param name The name of the entry
 * @param matrix The output matrix
 */
void
ReadMatrix(const H5::H5Location & fg, const char * name, MatrixType & matrix);

/**
 * Read a submatrix from the file, with the given number of Columns
 * @param fg The group
 * @param name The name of the entry
 * @param nCols The number of columns to be Read
 * @param matrix The output matrix
 */
void
ReadMatrix(const H5::H5Location & fg, const char * name, unsigned nCols, MatrixType & matrix);

/**
 * Read a Matrix of a given type from a HDF5 File
 * @param fg The group
 * @param name The name of the entry
 * @param matrix The output matrix
 */
template <class T>
void
ReadMatrixOfType(const H5::H5Location & fg, const char * name, typename GenericEigenTraits<T>::MatrixType & matrix);

/**
 * Write a Matrix to the HDF5 File
 * @param fg The group
 * @param name the name of the entry
 * @param the matrix to be written
 */
H5::DataSet
WriteMatrix(const H5::H5Location & fg, const char * name, const MatrixType & matrix);

/**
 * Write a Matrix of the given type to the HDF5 File
 * @param fg The group
 * @param name the name of the entry
 * @param the matrix to be written
 */
template <class T>
H5::DataSet
WriteMatrixOfType(const H5::H5Location &                             fg,
                  const char *                                       name,
                  const typename GenericEigenTraits<T>::MatrixType & matrix);


/**
 * Read a Vector from a HDF5 File with the given number of elements
 * @param fg The group
 * @param name the name of the entry
 * @param numElements The number of elements to be Read from the file
 * @param the output vector
 */
void
ReadVector(const H5::H5Location & fg, const char * name, unsigned nElements, VectorType & vector);

/**
 * Read a Vector from a HDF5 File
 * @param fg The group
 * @param name the name of the entry
 * @param numElements The number of elements to be Read from the file
 * @param the output vector
 */
void
ReadVector(const H5::H5Location & fg, const char * name, VectorType & vector);

template <class T>
void
ReadVectorOfType(const H5::H5Location & fg, const char * name, typename GenericEigenTraits<T>::VectorType & vector);

/**
 * Write a vector to the HDF5 File
 * @param fg The hdf5 group
 * @param name the name of the entry
 * @param the vector to be written
 */
H5::DataSet
WriteVector(const H5::H5Location & fg, const char * name, const VectorType & vector);

template <class T>
H5::DataSet
WriteVectorOfType(const H5::H5Location &                             fg,
                  const char *                                       name,
                  const typename GenericEigenTraits<T>::VectorType & vector);


/**
 * Reads a file (in binary mode) and saves it as a byte array in the hdf5 file.
 * @param filename The filename of the file to be stored
 * @param fg The hdf5 group
 * @param name The name of the entry
 */
void
DumpFileToHDF5(const char * filename, const H5::H5Location & fg, const char * name);

/**
 * Reads an entry from an HDF5 byte array and Writes it to a file
 * @param fg The hdf5 group
 * @param name the name of the entry
 * @param filename The filename where the data from the HDF5 file is stored.
 */
void
GetFileFromHDF5(const H5::H5Location & fg, const char * name, const char * filename);

/** Writes a string to the hdf5 file
 * @param fg The hdf5 group
 * @param name The name of the entry in the group
 * @param s The string to be written
 */
H5::DataSet
WriteString(const H5::H5Location & fg, const char * name, const std::string & s);

/** Reads a string from the given group
 * @param group the hdf5 group
 * @param name the name of the entry in the group
 * @return the string
 */
std::string
ReadString(const H5::H5Location & fg, const char * name);

/** Writes a string attribute for the given group
 * @param fg The hdf5 group
 * @param name The name of the entry in the group
 * @param s The string to be written
 */
void
WriteStringAttribute(const H5::H5Object & group, const char * name, const std::string & s);

/** Writes an int attribute for the given group
 * @param fg The hdf5 group
 * @param name The name of the entry in the group
 * @param value the int value to be written
 */
void
WriteIntAttribute(const H5::H5Object & fg, const char * name, int value);


/** Reads a string attribute from the given group
 * @param group the hdf5 group
 * @param name the name of the entry in the group
 * @return the value
 */
std::string
ReadStringAttribute(const H5::H5Object & group, const char * name);

/** Reads a int attribute from the given group
 * @param group the hdf5 group
 * @param name the name of the entry in the group
 * @return the value
 */
int
ReadIntAttribute(const H5::H5Object & group, const char * name);


/** Reads an integer from the hdf5 file
 * @param fg The hdf5 group
 * @param name The name
 * @returns the integeter
 */
int
ReadInt(const H5::H5Location & fg, const char * name);

/** Writes an integer to the hdf5 file
 * @param fg The hdf5 group
 * @param name The name
 * @param value The value to be written
 */
H5::DataSet
WriteInt(const H5::H5Location & fg, const char * name, int value);

/** Reads an dobule from the hdf5 file
 * @param fg The hdf5 group
 * @param name The name
 * @returns the Read number
 */
float
ReadFloat(const H5::H5Location & fg, const char * name);

/** Writes an double to the hdf5 file
 * @param fg The hdf5 group
 * @param name The name
 * @param value The value to be written
 */
H5::DataSet
WriteFloat(const H5::H5Location & fg, const char * name, float value);

/** Reads an array from the hdf5 group
 * @param fg The hdf5 group
 * @param name The name
 * @param array The array (type std::vector<T>) to be Read, contents will be lost
 */
template <typename T>
void
ReadArray(const H5::H5Location & fg, const char * name, std::vector<T> & array);

/** Writes an array to the hdf5 group
 * @param fg The hdf5 group
 * @param name The name
 * @param array The array (type std::vector<T>) to be written
 */
template <typename T>
H5::DataSet
WriteArray(const H5::H5Location & fg, const char * name, std::vector<T> const & array);


/** Check whether an object (direct child) of fg with the given name exists
 */
bool
ExistsObjectWithName(const H5::H5Location & fg, const std::string & name);

} // namespace statismo::hdf5utils

#include "HDF5Utils.hxx"

#endif
