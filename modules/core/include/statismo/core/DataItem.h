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

#ifndef __STATIMO_CORE_DATA_ITEM_H_
#define __STATIMO_CORE_DATA_ITEM_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/HDF5Utils.h"
#include "statismo/core/Representer.h"
#include "statismo/core/NonCopyable.h"

#include <vector>

namespace statismo
{
/* \class DataItem
 * \brief Holds all the information for a given sample.
 * Use GetSample() to recover a Sample
 */
template <typename T>
class DataItem : public NonCopyable
{
public:
  using RepresenterType = Representer<T>;
  using DatasetPointerType = typename RepresenterType::DatasetPointerType;

  /**
   *  Save the sample data to the hdf5 group dsGroup.
   */
  virtual void
  Save(const H5::Group & dsGroup) const = 0;

  /**
   * Get the URI of the original dataset
   */
  virtual std::string
  GetDatasetURI() const = 0;

  /**
   * Get the representer used to create this sample
   */
  virtual const RepresenterType *
  GetRepresenter() const = 0;

  /**
   * Get the vectorial representation of this sample
   */
  virtual VectorType
  GetSampleVector() const = 0;

  /**
   * Returns the sample in the representation given by the representer
   * \warning This method generates a new object containing the sample. If the Representer does not provide a smart
   * pointer, the user is responsible for releasing memory.
   */
  virtual DatasetPointerType
  GetSample() const = 0;

  /**
   * Generic delete function
   */
  virtual void
  Delete() const = 0;
};

/**
 * \brief Base class for data item
 */
template <typename T, typename Derived>
class DataItemBase
  : public DataItem<T>
  , public GenericFactory<Derived>
{
public:
  using RepresenterType = typename DataItem<T>::RepresenterType;
  using DatasetPointerType = typename DataItem<T>::DatasetPointerType;
  using ObjectFactoryType = GenericFactory<Derived>;
  friend ObjectFactoryType;

  /**
   * Create a new DataItem object, using the data from the group in the HDF5 file
   * \param dsGroup. The group in the hdf5 file for this dataset
   */
  static UniquePtrType<DataItemBase>
  Load(const RepresenterType * representer, const H5::Group & dsGroup);

  virtual void
  Save(const H5::Group & dsGroup) const override;

  std::string
  GetDatasetURI() const override
  {
    return m_URI;
  }

  const RepresenterType *
  GetRepresenter() const override
  {
    return m_representer;
  }

  VectorType
  GetSampleVector() const override
  {
    return m_sampleVector;
  }

  DatasetPointerType
  GetSample() const override
  {
    return m_representer->SampleVectorToSample(m_sampleVector);
  }

  virtual void
  Delete() const override
  {
    delete this;
  }

protected:
  DataItemBase(const RepresenterType * representer, const std::string & URI, const VectorType & sampleVector)
    : m_representer(representer)
    , m_URI(URI)
    , m_sampleVector(sampleVector)
  {}

  explicit DataItemBase(const RepresenterType * representer)
    : m_representer(representer)
  {}

  // loads the internal state from the hdf5 file
  virtual void
  LoadInternal(const H5::Group & dsGroup)
  {
    hdf5utils::ReadVector(dsGroup, "./samplevector", m_sampleVector);
    m_URI = hdf5utils::ReadString(dsGroup, "./URI");
  }

  void
  SaveInternal(const H5::Group & dsGroup) const
  {
    hdf5utils::WriteVector(dsGroup, "./samplevector", m_sampleVector);
    hdf5utils::WriteString(dsGroup, "./URI", m_URI);

    SaveInternalImpl(dsGroup);
  }

  virtual void
  SaveInternalImpl(const H5::Group & dsGroup) const
  {
    hdf5utils::WriteString(dsGroup, "./sampletype", "DataItem");
  }

  const RepresenterType * m_representer;
  std::string             m_URI;
  VectorType              m_sampleVector;
};

/* \class BasicDataItem
 * \brief Standard data item implementation
 */
template <typename T>
class BasicDataItem : public DataItemBase<T, BasicDataItem<T>>
{
public:
  using Superclass = DataItemBase<T, BasicDataItem<T>>;
  using RepresenterType = typename Superclass::RepresenterType;
  friend typename Superclass::ObjectFactoryType;

private:
  BasicDataItem(const RepresenterType * representer, const std::string & URI, const VectorType & sampleVector)
    : Superclass(representer, URI, sampleVector)
  {}

  explicit BasicDataItem(const RepresenterType * representer)
    : Superclass(representer)
  {}
};

/* \class DataItemWithSurrogates
 * \brief Specific data item implementation that associates surrogates to the data
 *
 * In particular, it enables to associate categorical or continuous variables with a sample,
 * in a vectorial representation. The vector is provided by a file providing the
 * values in ascii format (empty space or EOL separating the values) \sa DataItem \sa DataManagerWithSurrogates
 */
template <typename T>
class DataItemWithSurrogates : public DataItemBase<T, DataItemWithSurrogates<T>>
{
  using Superclass = DataItemBase<T, DataItemWithSurrogates<T>>;
  using RepresenterType = typename Superclass::RepresenterType;
  friend typename Superclass::ObjectFactoryType;

public:
  enum class SurrogateType
  {
    Categorical = 0, // e.g. Gender
    Continuous = 1   // e.g. Size, weight
  };
  using SurrogateTypeVectorType = std::vector<SurrogateType>;

  VectorType
  GetSurrogateVector() const
  {
    return m_surrogateVector;
  }

  std::string
  GetSurrogateFilename() const
  {
    return m_surrogateFilename;
  }

private:
  DataItemWithSurrogates(const RepresenterType * representer,
                         const std::string &     datasetURI,
                         const VectorType &      sampleVector,
                         const std::string &     surrogateFilename,
                         const VectorType &      surrogateVector)
    : Superclass(representer, datasetURI, sampleVector)
    , m_surrogateFilename(surrogateFilename)
    , m_surrogateVector(surrogateVector)
  {}

  explicit DataItemWithSurrogates(const RepresenterType * representer)
    : Superclass(representer)
  {}

  // loads the internal state from the hdf5 file
  virtual void
  LoadInternal(const H5::Group & dsGroup) override
  {
    Superclass::LoadInternal(dsGroup);
    hdf5utils::ReadVector(dsGroup, "./surrogateVector", this->m_surrogateVector);
    m_surrogateFilename = hdf5utils::ReadString(dsGroup, "./surrogateFilename");
  }

  virtual void
  SaveInternalImpl(const H5::Group & dsGroup) const override
  {
    hdf5utils::WriteString(dsGroup, "./sampletype", "DataItemWithSurrogates");
    hdf5utils::WriteVector(dsGroup, "./surrogateVector", this->m_surrogateVector);
    hdf5utils::WriteString(dsGroup, "./surrogateFilename", this->m_surrogateFilename);
  }

  std::string m_surrogateFilename;
  VectorType  m_surrogateVector;
};


} // namespace statismo

#include "DataItem.hxx"

#endif
