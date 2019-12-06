/*
 * DataItem.h
 *
 * Created by Marcel Luethi
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

#ifndef __STATIMO_CORE_DATA_ITEM_HXX_
#define __STATIMO_CORE_DATA_ITEM_HXX_

#include "statismo/core/DataItem.h"

namespace statismo
{

template <typename T, typename Derived>
UniquePtrType<DataItemBase<T, Derived>>
DataItemBase<T, Derived>::Load(const RepresenterType * representer, const H5::Group & dsGroup)
{
  auto                                    sampleType = hdf5utils::ReadString(dsGroup, "./sampletype");
  UniquePtrType<DataItemBase<T, Derived>> newSample;
  if (sampleType == "DataItem")
  {
    newSample = std::make_unique<BasicDataItem<T>>(representer);
  }
  else if (sampleType == "DataItemWithSurrogates")
  {
    newSample = std::make_unique<DataItemWithSurrogates<T>>(representer);
  }
  else
  {
    throw StatisticalModelException((std::string("Unknown sampletype in hdf5 group: ") + sampleType).c_str(),
                                    Status::INVALID_DATA_ERROR);
  }

  newSample->LoadInternal(dsGroup);

  return newSample;
}

template <typename T, typename Derived>
void
DataItemBase<T, Derived>::Save(const H5::Group & dsGroup) const
{
  SaveInternal(dsGroup);
}

} // namespace statismo

#endif
