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


#ifndef __STATIMO_ITK_DATA_MANAGER_H_
#define __STATIMO_ITK_DATA_MANAGER_H_

#include <functional>
#include <utility>

#include <itkObject.h>
#include <itkObjectFactory.h>

#include "statismo/ITK/itkUtils.h"
#include "statismo/core/DataManager.h"
#include "statismo/ITK/itkConfig.h"
#include "statismo/core/ImplWrapper.h"

namespace itk
{


/**
 * \brief ITK Wrapper for the statismo::DataManager class.
 * \see statismo::DataManager for detailed documentation.
 */
template <class T>
class DataManager
  : public Object
  , public statismo::ImplWrapper<statismo::BasicDataManager<T>>
{
public:
  typedef DataManager              Self;
  typedef Object                   Superclass;
  typedef SmartPointer<Self>       Pointer;
  typedef SmartPointer<const Self> ConstPointer;

  itkNewMacro(Self);
  itkTypeMacro(DataManager, Object);

  typedef typename statismo::BasicDataManager<T>::DataItemType     DataItemType;
  typedef typename statismo::BasicDataManager<T>::DataItemListType DataItemListType;
  typedef statismo::Representer<T>                                 RepresenterType;

  using ImplType = typename statismo::ImplWrapper<statismo::BasicDataManager<T>>::ImplType;

  DataManager() {}

  virtual ~DataManager() {}

  void
  SetRepresenter(const RepresenterType * representer)
  {
    this->SetStatismoImplObj(representer);
  }

  void
  AddDataset(typename RepresenterType::DatasetType * ds, const char * filename)
  {
    this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::AddDataset, ds, filename);
  }

  void
  Load(const char * filename)
  {
    try
    {
      this->SetStatismoImplObj(ImplType::Load(filename));
    }
    catch (statismo::StatisticalModelException & s)
    {
      itkExceptionMacro(<< s.what());
    }
  }

  void
  Save(const char * filename)
  {
    this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::Save, filename);
  }

  typename statismo::DataManager<T>::DataItemListType
  GetData() const
  {
    return this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::GetData);
  }
};


} // namespace itk

#endif /* ITK_DATAMANAGER_H_ */
