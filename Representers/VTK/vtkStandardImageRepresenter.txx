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


#include "vtkStandardImageRepresenter.h"
#include "vtkStructuredPointsReader.h"
#include "vtkStructuredPointsWriter.h"
#include "vtkPointData.h"
#include "vtkDataArray.h"
#include "statismo/HDF5Utils.h"
#include "statismo/utils.h"
#include "statismo/CommonTypes.h"

using statismo::VectorType;
using statismo::HDF5Utils;
using statismo::StatisticalModelException;

namespace statismo {

template <class TScalar, unsigned PixelDimensions>
vtkStandardImageRepresenter<TScalar, PixelDimensions>::vtkStandardImageRepresenter(DatasetConstPointerType reference)
  : m_reference(vtkStructuredPoints::New())
{
	this->SetReference(reference);
}



template <class TScalar, unsigned PixelDimensions>
vtkStandardImageRepresenter<TScalar, PixelDimensions>::~vtkStandardImageRepresenter() {

	if (m_reference != 0) {
		m_reference->Delete();
		m_reference = 0;
	}
}

template <class TScalar, unsigned PixelDimensions>
vtkStandardImageRepresenter<TScalar, PixelDimensions>*
vtkStandardImageRepresenter<TScalar, PixelDimensions>::Clone() const
{
	// this works since Create deep copies the reference
	return Create(m_reference);
}

template <class TScalar, unsigned PixelDimensions>
void
vtkStandardImageRepresenter<TScalar, PixelDimensions>::Load(const H5::Group& fg) {

	std::string type = HDF5Utils::readStringAttribute(fg, "datasetType");
	if (type != "POLYGON_MESH") {
		throw StatisticalModelException((std::string("Cannot load representer data: The ")
				+"representer specified in the given hdf5 file is of the wrong type: ("
				+type +", expected IMAGE)").c_str());
	}


	statismo::VectorType originVec;
	HDF5Utils::readVector(fg, "origin", originVec);

	unsigned imageDimension = static_cast<unsigned>(HDF5Utils::readInt(fg, "imageDimension"));
	unsigned pixelDimension = static_cast<unsigned>(HDF5Utils::readInt(fg, "pixelDimension"));

	if (imageDimension > 3) {
		throw StatisticalModelException("this representer does not support images of dimensionality > 3");
	}

	if (pixelDimension != PixelDimensions) {
		throw statismo::StatisticalModelException("the pixel dimension specified in the statismo file does not match the one specified as template parameter");
	}

	double origin[3] = {0, 0, 0};
	for (unsigned i = 0; i < imageDimension; i++) {
		origin[i] = originVec[i];
	}

	statismo::VectorType spacingVec;
	HDF5Utils::readVector(fg, "spacing", spacingVec);
	float spacing[3] = {0, 0, 0};
	for (unsigned i = 0; i < imageDimension; i++) {
		spacing[i] = spacingVec[i];
	}

	typename statismo::GenericEigenType<int>::VectorType sizeVec;
	HDF5Utils::readVectorOfType<int>(fg, "size", sizeVec);
	int size[3] = { 1, 1, 1};
	for (unsigned i = 0; i < imageDimension; i++) {
		size[i] = sizeVec[i];
	}

	H5::Group pdGroup = fg.openGroup("./pointData");
	unsigned int readPixelDimension = static_cast<unsigned>(HDF5Utils::readInt(pdGroup, "pixelDimension"));
	if (readPixelDimension != pixelDimension) {
		std::ostringstream os;
		os << "The pixel dimension that was specified does not match the pixel dimension in the file" <<  " (" << readPixelDimension << " != " << pixelDimension << ")";
		throw StatisticalModelException(os.str().c_str());
	}


	typename statismo::GenericEigenType<double>::MatrixType pixelMat;
	HDF5Utils::readMatrixOfType<double>(pdGroup, "pixelValues", pixelMat);
	H5::DataSet ds = pdGroup.openDataSet("pixelValues");
	unsigned int datatype = static_cast<unsigned>(HDF5Utils::readIntAttribute(ds, "datatype"));

	if (statismo::GetDataTypeId<TScalar>() != datatype) {
		std::cout << "Warning: The datatype specified for the scalars does not match the TPixel template argument used in this representer." << std::endl;
	}
	pdGroup.close();

	vtkStructuredPoints* newImage = vtkStructuredPoints::New();
	newImage->SetDimensions(size[0], size[1], size[2]);

	newImage->SetExtent( 0 , size[0] -1  , 0 , size[1] -1  , 0 , size[2] -1);

	newImage->SetOrigin( origin[0], origin[1], origin[2]);
	newImage->SetSpacing( spacing[0], spacing[1], spacing[2] );
	switch(datatype) {
	case statismo::SIGNED_CHAR:
		newImage->SetScalarTypeToSignedChar();
		break;
	case statismo::UNSIGNED_CHAR:
		newImage->SetScalarTypeToUnsignedChar();
		break;
	case statismo::SIGNED_SHORT:
		newImage->SetScalarTypeToShort();
		break;
	case statismo::UNSIGNED_SHORT:
		newImage->SetScalarTypeToUnsignedShort();
		break;
	case statismo::SIGNED_INT:
		newImage->SetScalarTypeToInt();
		break;
	case statismo::UNSIGNED_INT:
		newImage->SetScalarTypeToUnsignedInt();
		break;
	case statismo::FLOAT:
		newImage->SetScalarTypeToFloat();
		break;
	case statismo::DOUBLE:
		newImage->SetScalarTypeToDouble();
		break;
	default:
		throw statismo::StatisticalModelException("the datatype found in the statismo model cannot be used for the vtkStandardImageRepresenter");
	}

	newImage->SetNumberOfScalarComponents( pixelDimension );
	newImage->AllocateScalars();
	  for( int i = 0 ; i < size[2] ; i++ ) {
		 for( int j = 0 ; j < size[1] ; j++ ) {
			 for (int k = 0; k < size[0]; k++) {
				 unsigned index = size[1] * size[0] * i + size[0] * j + k;
				 TScalar* scalarPtr = static_cast<TScalar*>(newImage->GetScalarPointer(k,j,i));
				 for (unsigned d = 0; d < pixelDimension; d++) {
					 scalarPtr[d] = pixelMat(d, index);
				 }
			 }
		 }
	  }

	this->SetReference(newImage);
}

template <class TScalar, unsigned PixelDimensions>
statismo::VectorType vtkStandardImageRepresenter<TScalar, PixelDimensions>::PointToVector(const PointType& pt) const {
	// a vtk point is always 3 dimensional
	VectorType v(3);
	for (unsigned i = 0; i < 3; i++) {
		v(i) = pt[i];
	}
	return v;
}



template <class TScalar, unsigned PixelDimensions>
typename vtkStandardImageRepresenter<TScalar, PixelDimensions>::DatasetPointerType
vtkStandardImageRepresenter<TScalar, PixelDimensions>::DatasetToSample(DatasetConstPointerType dataset) const
{
	// for this representer, a dataset is always the same as a sample
	vtkStructuredPoints* clone = vtkStructuredPoints::New();
	clone->DeepCopy(const_cast<vtkStructuredPoints*>(dataset));


	if (const_cast<vtkStructuredPoints*>(m_reference)->GetNumberOfPoints() != const_cast<vtkStructuredPoints*>(dataset)->GetNumberOfPoints()) {
		throw StatisticalModelException("The dataset need to have the same number of points as the reference");
	}


	return clone;
}

template <class TScalar, unsigned PixelDimensions>
VectorType
vtkStandardImageRepresenter<TScalar, PixelDimensions>::SampleToSampleVector(DatasetConstPointerType _sp) const
{

	vtkStructuredPoints* reference = const_cast<vtkStructuredPoints*>(this->m_reference);
	vtkStructuredPoints* sp = const_cast<vtkStructuredPoints*>(_sp);

	if (reference->GetNumberOfPoints() != sp->GetNumberOfPoints()) {
		throw StatisticalModelException("The sample does not have the correct number of points");
	}

	VectorType sample = VectorType::Zero(m_reference->GetNumberOfPoints() * PixelDimensions);

	vtkDataArray* scalars = sp->GetPointData()->GetScalars();
	// TODO make this more efficient using SetVoidArray of vtk
	for (unsigned i = 0 ; i < m_reference->GetNumberOfPoints(); i++) {

		double val[PixelDimensions];
		scalars->GetTuple(i, val);
		for (unsigned d = 0; d < PixelDimensions; d++) {
			unsigned idx = MapPointIdToInternalIdx(i, d);
			sample(idx) = val[d];
		}
	}
	return sample;
}



template <class TScalar, unsigned PixelDimensions>
typename vtkStandardImageRepresenter<TScalar, PixelDimensions>::DatasetPointerType
vtkStandardImageRepresenter<TScalar, PixelDimensions>::SampleVectorToSample(const VectorType& sample) const
{
	vtkStructuredPoints* sp = vtkStructuredPoints::New();
	vtkStructuredPoints* reference = const_cast<vtkStructuredPoints*>(m_reference);
	sp->DeepCopy(reference);

	vtkDataArray* scalars = sp->GetPointData()->GetScalars();
	for (unsigned i = 0; i < GetNumberOfPoints(); i++) {
		double val[PixelDimensions];
		for (unsigned d = 0; d < PixelDimensions; d++) {
			unsigned idx = MapPointIdToInternalIdx(i, d);
			val[d] = sample(idx);
		}
		scalars->SetTuple(i, val);
	}
	sp->GetPointData()->SetScalars(scalars);
	return sp;
}


template <class TScalar, unsigned PixelDimensions>
typename vtkStandardImageRepresenter<TScalar, PixelDimensions>::ValueType
vtkStandardImageRepresenter<TScalar, PixelDimensions>::PointSampleFromSample(DatasetConstPointerType sample_, unsigned ptid) const {
	DatasetPointerType sample = const_cast<DatasetPointerType>(sample_);
	if (ptid >= sample->GetNumberOfPoints()) {
		throw StatisticalModelException("invalid ptid provided to PointSampleFromSample");
	}

	double doubleVal[PixelDimensions];
	sample->GetPointData()->GetScalars()->GetTuple(ptid, doubleVal);
	ValueType val(PixelDimensions);

	// vtk returns double. We need to convert it to whatever precision we are using in NDPixel
	for (unsigned i = 0; i < PixelDimensions; i++) {
		val[i] = static_cast<TScalar>(doubleVal[i]);
	}
	return val;
}


template <class TScalar, unsigned PixelDimensions>
statismo::VectorType
vtkStandardImageRepresenter<TScalar, PixelDimensions>::PointSampleToPointSampleVector(const ValueType& v) const
{
	VectorType vec(PixelDimensions);
	for (unsigned i = 0; i < PixelDimensions; i++) {
		vec[i] = const_cast<ValueType&>(v)[i];
		vec[i] = v[i];
	}
	return vec;
}

template <class TScalar, unsigned PixelDimensions>
typename vtkStandardImageRepresenter<TScalar, PixelDimensions>::ValueType
vtkStandardImageRepresenter<TScalar, PixelDimensions>::PointSampleVectorToPointSample(const VectorType& pointSample) const
{
	ValueType value(PixelDimensions);

	for (unsigned i = 0; i < PixelDimensions; i++)
		value[i] = pointSample[i];

	return value;
}




template <class TScalar, unsigned Dimensions>
void
vtkStandardImageRepresenter<TScalar, Dimensions>::Save(const H5::Group& fg) const {
	using namespace H5;


	// get the effective image dimension, by check the size
	int* size = m_reference->GetDimensions();
	unsigned imageDimension = 0;
	if (size[2] == 1 && size[1] == 1) imageDimension = 1;
	else if (size[2] == 1) imageDimension = 2;
	else imageDimension = 3;

	HDF5Utils::writeInt(fg, "imageDimension", imageDimension);

	double* origin = m_reference->GetOrigin();
	statismo::VectorType originVec = statismo::VectorType::Zero(imageDimension);
	for (unsigned i = 0; i < imageDimension; i++) {
		originVec(i) = origin[i];
	}
	HDF5Utils::writeVector(fg, "origin", originVec);

	double* spacing = m_reference->GetSpacing();
	statismo::VectorType spacingVec  = statismo::VectorType::Zero(imageDimension);
	for (unsigned i = 0; i < imageDimension; i++) {
		spacingVec(i) = spacing[i];
	}
	HDF5Utils::writeVector(fg, "spacing", spacingVec);

	statismo::GenericEigenType<int>::VectorType sizeVec= statismo::GenericEigenType<int>::VectorType::Zero(imageDimension);
	for (unsigned i = 0; i < imageDimension; i++) {
		sizeVec(i) = size[i];
	}
	HDF5Utils::writeVectorOfType<int>(fg, "size", sizeVec);

	// VTK does not support image direction. We write the identity matrix
	statismo::MatrixType directionMat = statismo::MatrixType::Identity(imageDimension, imageDimension);
	HDF5Utils::writeMatrix(fg, "direction", directionMat);

	typedef statismo::GenericEigenType<double>::MatrixType DoubleMatrixType;
	DoubleMatrixType pixelMat(GetDimensions(), GetNumberOfPoints());


	for (unsigned i = 0; i < static_cast<unsigned>(size[2]); i++) {
		for (unsigned j = 0; j < static_cast<unsigned>(size[1]); j++) {
			for (unsigned k = 0; k < static_cast<unsigned>(size[0]); k++) {
				unsigned ind = i * size[1] * size[0] + j * size[0] + k;
				TScalar * pixel = static_cast<TScalar*>(m_reference->GetScalarPointer(k,j,i));
				for (unsigned d = 0; d < GetDimensions(); d++) {
					pixelMat(d, ind) = pixel[d];
				}
			}
		}
	}

	H5::Group pdGroup = fg.createGroup("pointData");
	HDF5Utils::writeInt(pdGroup, "pixelDimension", GetDimensions());

	H5::DataSet ds = HDF5Utils::writeMatrixOfType<double>(pdGroup, "pixelValues", pixelMat);
	HDF5Utils::writeIntAttribute(ds, "datatype", statismo::GetDataTypeId<TScalar>());
	pdGroup.close();

}

template <class TScalar, unsigned Dimensions>
unsigned
vtkStandardImageRepresenter<TScalar, Dimensions>::GetNumberOfPoints() const {
	return GetNumberOfPoints(this->m_reference);
}

template <class TScalar, unsigned Dimensions>
void vtkStandardImageRepresenter<TScalar, Dimensions>::SetReference(const vtkStructuredPoints* reference) {
	m_reference->DeepCopy(const_cast<vtkStructuredPoints*>(reference));
	// set the domain
   DomainType::DomainPointsListType ptList;
   for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++) {
	   double* d = m_reference->GetPoint(i);
	   ptList.push_back(vtkPoint(d));
   }
	m_domain = DomainType(ptList);
}




template <class TScalar, unsigned Dimensions>
unsigned
vtkStandardImageRepresenter<TScalar, Dimensions>::GetPointIdForPoint(const PointType& pt) const {
	assert (m_reference != 0);
    return this->m_reference->FindPoint(const_cast<double*>(pt.data()));
}



template <class TScalar, unsigned Dimensions>
unsigned vtkStandardImageRepresenter<TScalar, Dimensions>::GetNumberOfPoints(vtkStructuredPoints* reference) {
    return reference->GetNumberOfPoints();
}

} // namespace statismo
