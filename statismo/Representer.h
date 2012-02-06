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


#ifndef REPRESENTER_H_
#define REPRESENTER_H_

/**
 * \brief Provides the interface between statismo and the dataset type the application uses.
 *
 * A Representer is a type that provides the connection between the statismo library
 * and the application. It serves two purposes:
 * - Provides an adapter to the datatype that it represents (i.e. defines methods to create, delete, load datasets, etc)
 * - Implements methods that convert the datasets into a vector representation and back.
 *
 * In the following the methods and types that have to be implemented to write a new
 * Representer for your application are given.
 *
 * \warning This class is never actually used, but serves only for documentation purposes.
 */
class Representer {
public:

	/**
	 * \name Type definitions
	 */
	///@{

	/// Defines (a pointer to) the type of the dataset that is represented.
	/// This could either be a naked pointer or a smart pointer.
	typedef TDataset* DatasetPointerType;

	/// Defines the const pointer type o fthe datset that is represented
	typedef const TDataset* DatasetConstPointerType;

	/// Defines the pointtype of the dataset
	typedef TPointType PointType;

	/// Defines the type of the value when the dataset is evaluated at a given point
	/// (for a image, this could for example be a scalar value or an RGB value)
	typedef TValueType ValueType;

	/// Returns a name that identifies the representer
	static std::string GetName();

	/// Returns the dimensionality of the dataset (for a mesh this is 3, for a scalar image
	/// this would be 1)
	static unsigned GetDimensions();
	///@}

	/**
	 * \name Object creation and destruction
	 */
	///@{

	/** Creates a new representer object, with the
	 * the information defined inthe given hdf5 group
	 * \sa Save
	 */
	static Representer* Load(const H5::CommonFG& fg);


	/** Clone the representer */
	Representer* Clone() const;

	/** Delete the representer object */
	void Delete() const;

	///@}

	/**
	 * \name Adapter methods
	 */
	///@{
	/// Create a new dataset
	static DatasetPointerType NewDataset() ;

	/// Delete a dataset
	static void DeleteDataset(DatasetPointerType d) ;

	/// Read a dataset with the given filename
	static DatasetPointerType ReadDataset(const std::string& filename);

	/// Write a dataset to the given filename
	static void WriteDataset(const std::string& filename, DatasetConstPointerType pd) ;
    ///@}


    /**
     * \name Conversion from the dataset to a vector representation and back
     */
    ///@{

	/**
	 * Returns the number of points of the datasets that are represented by this class.
	 */
	unsigned GetNumberOfPoints() const;

	/**
	 * Takes the given dataset (say of n points) and returns a representation of the dataset as
	 * a vector of nd elements (where d is the dimensionality returned by GetDimensions).
	 */
	statismo::VectorType DatasetToSampleVector(DatasetConstPointerType ds) const;

	/**
	 * Takes a vector of nd elements and converts it to a sample. The sample is a type
	 * that is represnter (e.g. an image, a mesh, etc).
	 */
	DatasetPointerType SampleVectorToSample(const statismo::VectorType& sample) const;

	/**
	 * Take a vector representing the values at a given point and converts it to a
	 * value of the dataset (e.g. for a mesh, the ValueType could be a 3D point,
	 * for a scalar image this would simply be a scalar)
	 */
	ValueType PointSampleToValue(const statismo::VectorType& pointSample) const;

	/**
	 * Given a value, convert it to a vector (of dimensionality given by GetDimensions())
	 */
	statismo::VectorType ValueToPointSample(const ValueType& v) const;


	/**
	 * Defines the mapping between the point ids and the position in the vector.
	 * Assume for example that a 3D mesh type is representerd.
	 * A conversion strategy used in DatasetToSampleVector could be to return
	 * a vector \f$(pt1_x, pt1_y, pt1_z, ..., ptn_x, ptn_y, ptn_z\f$.
	 * In this case, this method would return for inputs ptId, componentId
	 * the value ptId * 3 + componentId
	 */
	static unsigned MapPointIdToInternalIdx(unsigned ptId, unsigned componentInd) {
		return ptId * GetDimensions() + componentInd;
	}

	/**
	 * Given a point (the coordinates) return the pointId of this point.
	 */
	unsigned GetPointIdForPoint(const PointType& point) const;

	///@}

	/**
	 * \name Persistence
	 */
	///@{

	/**
	 * Save the informatino that define this representer to the group
	 * in the HDF5 file given by fg.
	 */
	void Save(const H5::CommonFG& fg);

	///@}
};


#endif /* REPRESENTER_H_ */




