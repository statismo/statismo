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


#ifndef __STATIMO_CORE_TRIVIAL_VECTORIAL_REPRESENTER_H_
#define __STATIMO_CORE_TRIVIAL_VECTORIAL_REPRESENTER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Domain.h"
#include "statismo/core/HDF5Utils.h"
#include "statismo/core/Representer.h"

#include <H5Cpp.h>

#include <memory>

namespace statismo
{

// A pointId is actually just an unsigned. However, we need to create a distinct type, to disambiguate some of
// the methods.
struct PointIdType
{
  PointIdType(unsigned ptId_)
    : ptId(ptId_)
  {}
  PointIdType()
    : ptId(0)
  {}

  unsigned ptId;
};


template <>
struct RepresenterTraits<statismo::VectorType>
{
  using DatasetPointerType = statismo::VectorType;
  using DatasetConstPointerType = statismo::VectorType;

  using PointType = PointIdType;
  using ValueType = statismo::ScalarType;
};


/**
 * \brief A trivial representer, that does no representation at all, but works directly with vectorial data
 *
 * \warning This representer is mainly for debugging purposes and not intended to be used for real projets
 */
class TrivialVectorialRepresenter : public RepresenterBase<statismo::VectorType, TrivialVectorialRepresenter>
{
public:
  using Superclass = RepresenterBase<statismo::VectorType, TrivialVectorialRepresenter>;
  friend Superclass;
  friend typename Superclass::ObjectFactoryType;
  using ValueType = statismo::ScalarType;
  using DomainType = typename statismo::Domain<PointType>;

  void
  Load(const H5::Group & fg) override
  {
    unsigned numPoints = static_cast<unsigned>(statismo::hdf5utils::ReadInt(fg, "numberOfPoints"));
    InitializeObject(numPoints);
  }

  void DeleteDataset(DatasetPointerType) const override{};
  DatasetPointerType
  CloneDataset(DatasetConstPointerType d) const override
  {
    return d;
  }

  const DomainType &
  GetDomain() const override
  {
    return m_domain;
  }

  DatasetConstPointerType
  GetReference() const override
  {
    return VectorType::Zero(m_domain.GetNumberOfPoints());
  }

  VectorType
  PointToVector(const PointType & pt) const override
  {
    // here, the pt type is simply an id (the index into the vector).
    VectorType v(1);
    v(0) = pt.ptId;
    return v;
  }

  VectorType
  SampleToSampleVector(DatasetConstPointerType sample) const override
  {
    return sample;
  }

  DatasetPointerType
  SampleVectorToSample(const statismo::VectorType & sample) const override
  {
    return sample;
  }

  VectorType
  PointSampleToPointSampleVector(const ValueType & v) const override
  {
    VectorType vec = VectorType::Zero(1);
    vec(0) = v;
    return vec;
  }

  ValueType
  PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const override
  {
    return sample[ptid];
  }
  ValueType
  PointSampleVectorToPointSample(const VectorType & pointSample) const override
  {
    return pointSample(0);
  }

  void
  Save(const H5::Group & fg) const override
  {
    hdf5utils::WriteInt(fg, "numberOfPoints", static_cast<int>(m_domain.GetNumberOfPoints()));
  }

  unsigned
  GetPointIdForPoint(const PointType & point) const override
  {
    return point.ptId;
  }

protected:
  static std::string
  GetNameImpl()
  {
    return "TrivialVectorialRepresenter";
  }
  static unsigned
  GetDimensionsImpl()
  {
    return 1;
  }
  static std::string
  GetVersionImpl()
  {
    return "0.1";
  }
  static RepresenterDataType
  GetTypeImpl()
  {
    return RepresenterDataType::VECTOR;
  }


private:
  TrivialVectorialRepresenter *
  CloneImpl() const override
  {
    return TrivialVectorialRepresenter::Create(m_domain.GetNumberOfPoints());
  }

  TrivialVectorialRepresenter() = default;
  TrivialVectorialRepresenter(unsigned numberOfPoints) { InitializeObject(numberOfPoints); }

  void
  InitializeObject(unsigned numberOfPoints)
  {
    // the domain for vectors correspond to the valid indices.
    typename DomainType::DomainPointsListType domainPoints;
    for (unsigned i = 0; i < numberOfPoints; i++)
    {
      domainPoints.push_back(PointIdType(i));
    }
    m_domain = DomainType(domainPoints);
  }

  DomainType m_domain;
};

} // namespace statismo

#endif
