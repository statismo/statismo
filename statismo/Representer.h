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
 * and the application. It distinguishes three different representations of the data, and provides methods for conversion between those representations:
 * - a Dataset, typically as read from a file on the disk
 * - a Sample, which is a geometric (generally a rigid or affine) transform of the dataset
 * - a SampleVector, which is an internal representation (vector) useful from the statistical analysis.
 *
 * In the following the methods and types that have to be implemented to write a new
 * Representer for your application are given.
 *
 * \warning This class is never actually used, but serves only for documentation purposes.
 */
 //RB: would it be possible to make all representers inherit from it, so as to strictly enforce the interface?
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


	/// This struct can be used to pass additional information for a given dataste
	/// A typical example are landmark points, that are used to align a datset.
	/// This field is here for future use, Statismo does curently not support it.
	struct DatasetInfo {};

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
	/// Delete a dataset of the type DatasetPointerType. This method may do nothing, if there
	/// is no need to delete the dataset explicitely (e.g. if DatasetPointerType is a smart pointer).
	static void DeleteDataset(DatasetPointerType d) ;
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
	 * Returns the Domain for this representers. The domain is essentially a list of all the points on which the model is defined.
	 * \sa statismo::Domain
	 */
	const statismo::Domain<PointType>& GetDomain() const;


	/**
	 * Takes the given dataset and converts it to a sample, as it is internally used by statismo.
	 * Typical steps that are performed to convert a dataset into a sample are alignment and registration.
	 */
	DatasetPointerType DatasetToSample(DatasetConstPointerType ds, DatasetInfo* notUsed) const;

	/**
	 * Returns a vectorial representation of the given sample.
	 */
	statismo::VectorType SampleToSampleVector(DatasetConstPointerType sample) const;

	/**
	 * Takes a vector of nd elements and converts it to a sample. The sample is a type
	 * that is represnter (e.g. an image, a mesh, etc).
	 */
	DatasetPointerType SampleVectorToSample(const statismo::VectorType& sample) const;


	/**
	 * Returns the value of the sample at the point with the given id.
	 */
	ValueType PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const;


	/**
	 * Take a point sample (i.e. the value of a sample at a given point) and converts it
	 * to its vector representation.
	 * The type of the point sample is a ValueType, that depends on the type of the dataset.
	 * For a mesh this would for example be a 3D point,
	 * while for a scalar image this would be a scalar value representing the intensity.
	 */
	statismo::VectorType PointSampleVectorToPointSample(const ValueType& v) const;

	/**
	 * Convert the given vector represenation of a pointSample back to its ValueType
	 * \sa PointSampleVectorToPointSample
	 */
	ValueType PointSampleToPointSampleVector(const statismo::VectorType& pointSample) const;

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




