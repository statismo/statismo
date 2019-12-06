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

#ifndef __STATIMO_CORE_DATA_MANAGER_HXX_
#define __STATIMO_CORE_DATA_MANAGER_HXX_

#include "statismo/core/DataManager.h"
#include "statismo/core/ModelInfo.h"
#include "statismo/core/Exceptions.h"
#include "statismo/core/Utils.h"
#include "statismo/core/HDF5Utils.h"

#include <iostream>
#include <random>

namespace statismo
{

template <typename T, typename Derived>
DataManagerBase<T, Derived>::DataManagerBase(const RepresenterType * representer)
  : m_representer{ representer->SafeCloneSelf() }
{}

template <typename T, typename Derived>
UniquePtrType<DataManagerBase<T, Derived>>
DataManagerBase<T, Derived>::Load(Representer<T> * representer, const std::string & filename)
{
  using namespace H5;

  H5File file;
  try
  {
    file = H5File(filename.c_str(), H5F_ACC_RDONLY);
  }
  catch (const H5::Exception & e)
  {
    std::string msg(std::string("could not open HDF5 file \n") + e.getCDetailMsg());
    throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
  }

  UniquePtrType<DataManagerBase<T, Derived>> newDataManagerBase;
  try
  {
    // loading representer
    auto representerGroup = file.openGroup("./representer");
    auto repName = hdf5utils::ReadStringAttribute(representerGroup, "name");
    auto repTypeStr = hdf5utils::ReadStringAttribute(representerGroup, "datasetType");
    auto versionStr = hdf5utils::ReadStringAttribute(representerGroup, "version");
    auto type = RepresenterType::TypeFromString(repTypeStr);
    if (type == RepresenterType::CUSTOM || type == RepresenterType::UNKNOWN)
    {
      if (repName != representer->GetName())
      {
        std::ostringstream os;
        os << "A different representer was used to create the file and the representer is not of a standard type ";
        os << ("(RepresenterName = ") << repName << " does not match required name = " << representer->GetName() << ")";
        os << "Cannot load hdf5 file";
        throw StatisticalModelException(os.str().c_str(), Status::INVALID_DATA_ERROR);
      }

      if (versionStr != representer->GetVersion())
      {
        std::ostringstream os;
        os << "The version of the representers do not match ";
        os << ("(Version = ") << versionStr << " != = " << representer->GetVersion() << ")";
        os << "Cannot load hdf5 file";
      }
    }

    if (type != representer->GetType())
    {
      std::ostringstream os;
      os << "The representer that was provided cannot be used to load the dataset ";
      os << "(" << type << " != " << representer->GetType() << ").";
      os << "Cannot load hdf5 file.";
      throw StatisticalModelException(os.str().c_str(), Status::INVALID_DATA_ERROR);
    }

    representer->Load(representerGroup);
    newDataManagerBase = std::make_unique<DataManagerBase<T, Derived>>(representer);

    auto     dataGroup = file.openGroup("/data");
    unsigned numds = hdf5utils::ReadInt(dataGroup, "./NumberOfDatasets");
    for (unsigned num = 0; num < numds; num++)
    {
      std::ostringstream ss;
      ss << "./dataset-" << num;

      auto dsGroup = file.openGroup(ss.str().c_str());
      newDataManagerBase->m_dataItemList.push_back(DataItemType::Load(representer, dsGroup));
    }
  }
  catch (const H5::Exception & e)
  {
    std::string msg =
      std::string("an exception occurred while reading data matrix to HDF5 file \n") + e.getCDetailMsg();
    throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
  }

  assert(newDataManagerBase);
  return newDataManagerBase;
}

template <typename T, typename Derived>
void
DataManagerBase<T, Derived>::Save(const std::string & filename) const
{
  using namespace H5;

  assert(m_representer);

  H5File file;
  try
  {
    file = H5File(filename.c_str(), H5F_ACC_TRUNC);
  }
  catch (H5::Exception & e)
  {
    std::string msg(std::string("Could not open HDF5 file for writing \n") + e.getCDetailMsg());
    throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
  }

  try
  {
    auto representerGroup = file.createGroup("./representer");
    auto dataTypeStr = TypeToString(m_representer->GetType());

    hdf5utils::WriteStringAttribute(representerGroup, "name", m_representer->GetName());
    hdf5utils::WriteStringAttribute(representerGroup, "version", m_representer->GetVersion());
    hdf5utils::WriteStringAttribute(representerGroup, "datasetType", dataTypeStr);

    this->m_representer->Save(representerGroup);

    auto dataGroup = file.createGroup("./data");
    hdf5utils::WriteInt(dataGroup, "./NumberOfDatasets", this->m_dataItemList.size());

    unsigned num{ 0 };
    for (const auto & item : m_dataItemList)
    {
      std::ostringstream ss;
      ss << "./dataset-" << num;

      Group dsGroup = file.createGroup(ss.str().c_str());
      item->Save(dsGroup);

      num++;
    }
  }
  catch (H5::Exception & e)
  {
    std::string msg =
      std::string("an exception occurred while writing data matrix to HDF5 file \n") + e.getCDetailMsg();
    throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
  }
}

template <typename T, typename Derived>
void
DataManagerBase<T, Derived>::AddDataset(DatasetConstPointerType dataset, const std::string & URI)
{
  auto sample = m_representer->CloneDataset(dataset);
  auto uw = MakeStackUnwinder([&]() { m_representer->DeleteDataset(sample); });

  m_dataItemList.push_back(MakeSharedPointer<DataItemType>(
    ConcreteDataItemType::Create(m_representer.get(), URI, m_representer->SampleToSampleVector(sample))));
}

template <typename T, typename Derived>
typename DataManagerBase<T, Derived>::DataItemListType
DataManagerBase<T, Derived>::GetData() const
{
  return m_dataItemList;
}

template <typename T, typename Derived>
typename DataManagerBase<T, Derived>::CrossValidationFoldListType
DataManagerBase<T, Derived>::GetCrossValidationFolds(unsigned nFolds, bool isRandomized) const
{
  if (nFolds <= 1 || nFolds > GetNumberOfSamples())
  {
    throw StatisticalModelException("Invalid number of folds specified in GetCrossValidationFolds",
                                    Status::BAD_INPUT_ERROR);
  }

  unsigned nElemsPerFold = GetNumberOfSamples() / nFolds;

  // we create a vector with as many entries as datasets. Each entry contains the
  // fold the entry belongs to
  std::vector<unsigned> batchAssignment(GetNumberOfSamples());

  for (unsigned i = 0; i < GetNumberOfSamples(); i++)
  {
    batchAssignment[i] = std::min(i / nElemsPerFold, nFolds);
  }

  // randomly shuffle the vector
  if (isRandomized)
  {
    std::random_device rd;
    std::mt19937       g(rd());
    std::shuffle(batchAssignment.begin(), batchAssignment.end(), g);
  }

  // now we create the folds
  CrossValidationFoldListType foldList;
  for (unsigned currentFold = 0; currentFold < nFolds; currentFold++)
  {
    DataItemListType trainingData;
    DataItemListType testingData;

    unsigned sampleNum{ 0 };
    for (const auto & item : m_dataItemList)
    {
      if (batchAssignment[sampleNum] != currentFold)
      {
        trainingData.push_back(item);
      }
      else
      {
        testingData.push_back(item);
      }
      ++sampleNum;
    }
    foldList.emplace_back(trainingData, testingData);
  }
  return foldList;
}

template <typename T, typename Derived>
typename DataManagerBase<T, Derived>::CrossValidationFoldListType
DataManagerBase<T, Derived>::GetLeaveOneOutCrossValidationFolds() const
{
  CrossValidationFoldListType foldList;
  for (unsigned currentFold = 0; currentFold < GetNumberOfSamples(); currentFold++)
  {
    DataItemListType trainingData;
    DataItemListType testingData;

    unsigned sampleNum{ 0 };
    for (const auto & item : m_dataItemList)
    {
      if (sampleNum == currentFold)
      {
        testingData.push_back(item);
      }
      else
      {
        trainingData.push_back(item);
      }
      ++sampleNum;
    }
    CrossValidationFoldType fold(trainingData, testingData);
    foldList.push_back(fold);
  }
  return foldList;
}

} // Namespace statismo

#endif
