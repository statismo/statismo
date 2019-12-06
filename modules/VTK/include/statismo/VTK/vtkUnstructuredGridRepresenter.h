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


#ifndef __STATISMO_VTK_UNSTRUCTURED_GRID_REPRESENTER_HXX_
#define __STATISMO_VTK_UNSTRUCTURED_GRID_REPRESENTER_HXX_

#include <H5Cpp.h>

#include <vtkLandmarkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkUnstructuredGrid.h>

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Domain.h"
#include "statismo/VTK/vtkHelper.h"
#include "statismo/VTK/vtkPoint.h"
#include "statismo/core/Representer.h"

/**
 * \brief A representer for vtkUnstructuredGrid using Procrustes alignment to align the datasets
 *
 * This class provides a specialization of the Representer for the type vtkUnstructuredGrid.
 * Procrustes is used to align the given datasets with the reference.
 * The user can choose between Rigid, Similarity and Affine alignment.
 *
 * Hint: In order to use GPA alignment, simply set the Procrustes Mean as the reference.
 *
 * Warning: This class does currently not provide any registration, which implies
 * that the dataset that are read by the class need to be aligned.
 *
 * See Representer for more details about representer classes
 */
class vtkUnstructuredGridRepresenter
{
public:
  /// The type of the data set to be used
  typedef vtkUnstructuredGrid *       DatasetPointerType;
  typedef const vtkUnstructuredGrid * DatasetConstPointerType;

  typedef statismo::vtkPoint PointType;
  typedef statismo::vtkPoint ValueType;

  typedef statismo::Domain<PointType> DomainType;

  struct DatasetInfo
  {}; // not used for this representer, but needs to be here as it is part of the generic interface


  enum AlignmentType
  {
    NONE = 999, // something that VTK does not define
    RIGID = VTK_LANDMARK_RIGIDBODY,
    SIMILARITY = VTK_LANDMARK_SIMILARITY,
    AFFINE = VTK_LANDMARK_AFFINE
  };

  static vtkUnstructuredGridRepresenter *
  Create(DatasetConstPointerType reference, AlignmentType alignment)
  {
    return new vtkUnstructuredGridRepresenter(reference, alignment);
  }

  static vtkUnstructuredGridRepresenter *
  Load(const H5::H5Location & fg);

  vtkUnstructuredGridRepresenter *
  CloneSelf() const;
  void
  Delete() const
  {
    delete this;
  }

  virtual ~vtkUnstructuredGridRepresenter();


  static std::string
  GetName()
  {
    return "vtkUnstructuredGridRepresenter";
  }
  static unsigned
  GetDimensions()
  {
    return 3;
  }

  const DomainType &
  GetDomain() const
  {
    return m_domain;
  }

  AlignmentType
  GetAlignment() const
  {
    return m_alignment;
  }

  DatasetConstPointerType
  GetReference() const
  {
    return m_reference;
  }

  statismo::VectorType
  PointToVector(const PointType & pt) const;
  DatasetPointerType
  DatasetToSample(DatasetConstPointerType ds, DatasetInfo * notUsed) const;
  statismo::VectorType
  SampleToSampleVector(DatasetConstPointerType sample) const;
  DatasetPointerType
  SampleVectorToSample(const statismo::VectorType & sample) const;

  ValueType
  PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const;
  statismo::VectorType
  PointSampleToPointSampleVector(const ValueType & v) const;
  ValueType
  PointSampleVectorToPointSample(const statismo::VectorType & pointSample) const;


  void
  Save(const H5::H5Location & fg) const;
  unsigned
  GetNumberOfPoints() const;
  unsigned
  GetPointIdForPoint(const PointType & point) const;


  static void
  DeleteDataset(DatasetPointerType d);

  /* Maps a (Pointid,component) tuple to a component of the internal matrix.
   * This is used to locate the place in the matrix to store the elements for a given point.
   * @params ptId The point id
   * @params the Component Index (range 0, Dimensionality)
   * @returns an index.
   */
  static unsigned
  MapPointIdToInternalIdx(unsigned ptId, unsigned componentInd)
  {
    return ptId * GetDimensions() + componentInd;
  }


private:
  vtkUnstructuredGridRepresenter(const std::string & reference, AlignmentType alignment);
  vtkUnstructuredGridRepresenter(const DatasetConstPointerType reference, AlignmentType alignment);
  vtkUnstructuredGridRepresenter(const vtkUnstructuredGridRepresenter & orig);
  vtkUnstructuredGridRepresenter &
  operator=(const vtkUnstructuredGridRepresenter & rhs);

  static DatasetPointerType
  ReadDataset(const std::string & filename);
  static void
  WriteDataset(const std::string & filename, DatasetConstPointerType pd);


  DatasetPointerType m_reference;

  vtkTransformPolyDataFilter * m_pdTransform;
  AlignmentType                m_alignment;
  DomainType                   m_domain;
};

#endif /* vtkUnstructuredGridREPRESENTER_H_ */
