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
 * PROFITS; OR BUSINESS addINTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __STATIMO_CORE_GENERIC_REPRESENTER_VALIDATOR_H_
#define __STATIMO_CORE_GENERIC_REPRESENTER_VALIDATOR_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/HDF5Utils.h"
#include "statismo/core/Utils.h"

#include <iostream>
#include <string>
#include <algorithm>

/**
 * \brief Generic validator for representers. The tests need to hold for all representers.
 * It is used on the framework to validate statismo representer and can be used to validate
 * user provided representers.
 */
template <typename Representer>
class GenericRepresenterValidator
{
  // we define aliases for all required typenames, to force a compilation error if one of them
  // is not defined
  using DatasetConstPointerType = typename Representer::DatasetConstPointerType;
  using DatasetPointerType = typename Representer::DatasetPointerType;
  using PointType = typename Representer::PointType;
  using ValueType = typename Representer::ValueType;
  using DomainType = typename Representer::DomainType;

public:
  /**
   * Create a new validator. Tests are performed using the given testDataset and the pointValuePair.
   * \warning It is assumed that the PointValuePair is taken from the testDataset (otherwise some tests will fail).
   */
  GenericRepresenterValidator(const Representer *                     representer,
                              DatasetConstPointerType                 testDataset,
                              const std::pair<PointType, ValueType> & pointValuePair)
    : m_representer(representer)
    , m_testDataset(testDataset)
    , m_testPoint(pointValuePair.first)
    , m_testValue(pointValuePair.second)
  {}


  bool
  TestSamplePointEvaluation() const
  {
    m_errString = "";
    DatasetConstPointerType sample = m_testDataset;
    auto                    id = m_representer->GetPointIdForPoint(m_testPoint);
    auto                    val = m_representer->PointSampleFromSample(sample, id);
    auto                    valVec = m_representer->PointSampleToPointSampleVector(val);

    // the obtained value should correspond to the value that is obtained by obtaining the sample vector, and evaluating
    // it at the given position
    auto sampleVector = m_representer->SampleToSampleVector(sample);
    for (unsigned i = 0; i < m_representer->GetDimensions(); ++i)
    {
      if (sampleVector(i) != valVec(i))
      {
        return false;
      }
    }
    return true;
  }

  bool
  TestDomainValid() const
  {
    m_errString = "";
    auto domain = m_representer->GetDomain();
    auto domPoints = domain.GetDomainPoints();

    if (domPoints.size() == 0)
    {
      m_errString = "Representer defined on empty domain";
      return false;
    }

    if (domPoints.size() != domain.GetNumberOfPoints())
    {
      m_errString = "domPoints.size() != domain.GetNumberOfPoints() (" + std::to_string(domPoints.size()) +
                    " != " + std::to_string(domain.GetNumberOfPoints());
      return false;
    }
    // if we convert a dataset to a samplevector, the resulting vector needs to have
    // as many entries as there are points * dimensions
    auto sampleVector = m_representer->SampleToSampleVector(m_testDataset);
    if (sampleVector.rows() != m_representer->GetDimensions() * domain.GetNumberOfPoints())
    {
      m_errString = "the dimension of the sampleVector does not agree with the number of points in the domain (#points "
                    "* dimensionality)";
      return false;
    }

    unsigned ptCounter{ 0 };
    for (const auto & pt : domPoints)
    {
      // sample to every 10th points to improve performance
      if (ptCounter % 10 != 0)
      {
        continue;
      }

      if (m_representer->GetPointIdForPoint(pt) >= domain.GetNumberOfPoints())
      {
        m_errString = "a point in the domain did not evaluate to a valid point it";
        return false;
      }
      ++ptCounter;
    }

    return true;
  }

  /**
   * Test whether converting a sample to a vector and back to a sample yields the original sample
   */
  bool
  TestSampleToVectorAndBack() const
  {
    m_errString = "";
    // as we don't know anything about how to compare samples, we compare their vectorial representation
    auto sampleVec = GetSampleVectorFromTestDataset();
    return AssertSampleVectorsEqual(
      sampleVec, m_representer->SampleToSampleVector(m_representer->SampleVectorToSample(sampleVec)));
  }

  /**
   * Test point sample dimensionnality
   */
  bool
  TestPointSampleDimension() const
  {
    m_errString = "";
    auto valVec = m_representer->PointSampleToPointSampleVector(m_testValue);
    return (valVec.rows() == m_representer->GetDimensions());
  }

  /**
   * Test if the conversion from a pointSample and the pointSampleVector, and back to a pointSample
   * yields the original sample.
   */
  bool
  TestPointSampleToPointSampleVectorAndBack() const
  {
    m_errString = "";
    auto valVec = m_representer->PointSampleToPointSampleVector(m_testValue);
    return AssertSampleVectorsEqual(
      valVec, m_representer->PointSampleToPointSampleVector(m_representer->PointSampleVectorToPointSample(valVec)));
  }

  /**
   * Test if the testSample contains the same entries in the vector as those obtained by taking the
   * pointSample at the corresponding position.
   */
  bool
  TestSampleVectorHasCorrectValueAtPoint() const
  {
    m_errString = "";
    auto ptId = m_representer->GetPointIdForPoint(m_testPoint);
    if (ptId < 0 || ptId >= m_representer->GetNumberOfPoints())
    {
      m_errString = "invalid point id for test point " + std::to_string(ptId);
      return false;
    }

    // the value of the point in the sample vector needs to correspond the the value that was provided
    auto sampleVec = GetSampleVectorFromTestDataset();
    auto pointSampleVec = m_representer->PointSampleToPointSampleVector(m_testValue);
    for (unsigned d = 0; d < m_representer->GetDimensions(); ++d)
    {
      unsigned idx = m_representer->MapPointIdToInternalIdx(ptId, d);
      if (sampleVec[idx] != pointSampleVec[d])
      {
        m_errString = "the sample vector does not contain the correct value of the pointSample";
        return false;
      }
    }
    return true;
  }

  /**
   * Test Save and Load restore the representer
   */
  bool
  TestSaveLoad() const
  {
    m_errString = "";
    using namespace H5;

    auto   filename = statismo::utils::CreateTmpName(".rep");
    H5File file;
    try
    {
      file = H5File(filename, H5F_ACC_TRUNC);
    }
    catch (const Exception & e)
    {
      m_errString = "Error: Could not open HDF5 file for writing \n" + std::string(e.getCDetailMsg());
      return false;
    }
    auto representerGroup = file.createGroup("/representer");
    m_representer->Save(representerGroup);

    // We add the required attributes, which are usually written by the StatisticalModel class.
    // This is needed, as some representers check on these values.
    statismo::hdf5utils::WriteStringAttribute(representerGroup, "name", m_representer->GetName());
    std::string dataTypeStr = TypeToString(m_representer->GetType());
    statismo::hdf5utils::WriteStringAttribute(representerGroup, "datasetType", dataTypeStr);

    file.close();
    try
    {
      file = H5File(filename.c_str(), H5F_ACC_RDONLY);
    }
    catch (const Exception & e)
    {
      m_errString = "Error: could not open HDF5 file \n" + std::string(e.getCDetailMsg());
      return false;
    }

    representerGroup.close();
    representerGroup = file.openGroup("/representer");

    auto newRep = Representer::SafeCreate();
    newRep->Load(representerGroup);

    return AssertRepresenterEqual(newRep.get(), m_representer);
  }

  /**
   * Test cloning do not change representer state and behavior
   */
  bool
  TestClone() const
  {
    m_errString = "";
    auto          repUnique = m_representer->SafeCloneSelf();
    Representer * rep = dynamic_cast<Representer *>(repUnique.get());

    return AssertRepresenterEqual(rep, m_representer);
  }

  /**
   * Test sample dimensions
   */
  bool
  TestSampleVectorDimensions() const
  {
    m_errString = "";
    statismo::VectorType testSampleVec = GetSampleVectorFromTestDataset();
    return (m_representer->GetDimensions() * m_representer->GetNumberOfPoints() == testSampleVec.rows());
  }

  /**
   * Test representer name
   */
  bool
  TestGetName() const
  {
    m_errString = "";
    return !m_representer->GetName().empty();
  }

  /**
   * Test dimensions
   */
  bool
  TestDimensions() const
  {
    m_errString = "";
    return m_representer->GetDimensions() > 0;
  }

  /// run all the tests
  bool
  RunAllTests() const
  {
    static std::array<std::pair<std::string, bool (GenericRepresenterValidator::*)() const>, 11> testSuite{
      { { "TestPointSampleDimension", &GenericRepresenterValidator::TestPointSampleDimension },
        { "TestSamplePointEvaluation", &GenericRepresenterValidator::TestSamplePointEvaluation },
        { "TestDomainValid", &GenericRepresenterValidator::TestDomainValid },
        { "TestPointSampleToPointSampleVectorAndBack",
          &GenericRepresenterValidator::TestPointSampleToPointSampleVectorAndBack },
        { "TestSampleVectorHasCorrectValueAtPoint",
          &GenericRepresenterValidator::TestSampleVectorHasCorrectValueAtPoint },
        { "TestSampleToVectorAndBack", &GenericRepresenterValidator::TestSampleToVectorAndBack },
        { "TestSaveLoad", &GenericRepresenterValidator::TestSaveLoad },
        { "TestClone", &GenericRepresenterValidator::TestClone },
        { "TestSampleVectorDimensions", &GenericRepresenterValidator::TestSampleVectorDimensions },
        { "TestGetName", &GenericRepresenterValidator::TestGetName },
        { "TestDimensions", &GenericRepresenterValidator::TestDimensions } }
    };

    return std::all_of(std::cbegin(testSuite), std::cend(testSuite), [this](const auto & p) {
      bool ok = (this->*(p.second))();

      if (!ok)
      {
        std::cerr << "[FAILED] Test " << p.first << " failed with error: " << m_errString << std::endl;
      }

      std::cout << "[SUCCESS] Test " << p.first << " succeeded" << std::endl;
      return ok;
    });
  }

  std::string
  GetErrorString() const
  {
    return (m_errString.empty() ? "Unknown error" : m_errString);
  }

private:
  bool
  AssertRepresenterEqual(const Representer * representer1, const Representer * representer2) const
  {
    if (representer1->GetNumberOfPoints() != representer2->GetNumberOfPoints())
    {
      m_errString = "the representers do not have the same nubmer of points ";
      return false;
    }

    statismo::VectorType sampleRep1 = GetSampleVectorFromTestDataset(representer1);
    statismo::VectorType sampleRep2 = GetSampleVectorFromTestDataset(representer2);
    if (AssertSampleVectorsEqual(sampleRep1, sampleRep2) == false)
    {
      m_errString = "the representers produce different sample vectors for the same sample";
      return false;
    }

    return true;
  }


  bool
  AssertSampleVectorsEqual(const statismo::VectorType & v1, const statismo::VectorType & v2) const
  {
    if (v1.rows() != v2.rows())
    {
      m_errString = "mismatch in dimensionality of SampleVectors";
      return false;
    }

    for (unsigned i = 0; i < v1.rows(); ++i)
    {
      if (v1[i] != v2[i])
      {
        m_errString = "the sample vectors are not the same";
        return false;
      }
    }

    return true;
  }


  statismo::VectorType
  GetSampleVectorFromTestDataset() const
  {
    return GetSampleVectorFromTestDataset(m_representer);
  }

  statismo::VectorType
  GetSampleVectorFromTestDataset(const Representer * representer) const
  {
    DatasetConstPointerType sample = m_testDataset;
    statismo::VectorType    sampleVec = representer->SampleToSampleVector(sample);
    return sampleVec;
  }


  const Representer *     m_representer;
  DatasetConstPointerType m_testDataset;
  PointType               m_testPoint;
  ValueType               m_testValue;
  mutable std::string     m_errString;
};

#endif
