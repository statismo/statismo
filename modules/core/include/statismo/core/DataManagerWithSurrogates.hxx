/*
 * DataManagerWithSurrogates.hxx
 *
 * Created by: Marcel Luethi and  Remi Blanc
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

#ifndef __STATIMO_CORE_DATA_MANAGER_WITH_SURROGATES_HXX_
#define __STATIMO_CORE_DATA_MANAGER_WITH_SURROGATES_HXX_

#include "statismo/core/DataManagerWithSurrogates.h"
#include "statismo/core/Utils.h"
#include "statismo/core/HDF5Utils.h"

#include <iostream>

namespace statismo
{
template <typename T>
DataManagerWithSurrogates<T>::DataManagerWithSurrogates(const RepresenterType * representer,
                                                        const std::string &     filename)
  : Superclass{ representer }
{
  LoadSurrogateTypes(filename);
}

template <typename T>
void
DataManagerWithSurrogates<T>::LoadSurrogateTypes(const std::string & filename)
{
  m_typeInfo.typeFilename = filename;
  m_typeInfo.types.clear();

  auto surrogateTypes = utils::ReadVectorFromTxtFile(filename.c_str());
  for (unsigned i = 0; i < surrogateTypes.size(); ++i)
  {
    if (surrogateTypes(i) == 0)
    {
      m_typeInfo.types.push_back(DataItemWithSurrogatesType::SurrogateType::Categorical);
    }
    else
    {
      m_typeInfo.types.push_back(DataItemWithSurrogatesType::SurrogateType::Continuous);
    }
  }
}


template <typename T>
void
DataManagerWithSurrogates<T>::AddDatasetWithSurrogates(DatasetConstPointerType ds,
                                                       const std::string &     datasetURI,
                                                       const std::string &     surrogateFilename)
{
  assert(this->m_representer);
  auto surrogateVector = utils::ReadVectorFromTxtFile(surrogateFilename.c_str());

  if (static_cast<std::size_t>(surrogateVector.size()) != m_typeInfo.types.size())
  {
    throw StatisticalModelException("Trying to loading a dataset with unexpected number of surrogates",
                                    Status::INVALID_DATA_ERROR);
  }

  DatasetPointerType sample = this->m_representer->CloneDataset(ds);
  auto               uw = MakeStackUnwinder([&]() { this->m_representer->DeleteDataset(sample); });

  this->m_dataItemList.push_back(MakeSharedPointer<DataItemType>(
    DataItemWithSurrogatesType::Create(this->m_representer.get(),
                                       datasetURI,
                                       this->m_representer->SampleToSampleVector(sample),
                                       surrogateFilename,
                                       surrogateVector)));
}

} // namespace statismo

#endif
