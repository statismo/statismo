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

#ifndef __DATA_ITEM_H_
#define __DATA_ITEM_H_

#include "CommonTypes.h"
#include "HDF5Utils.h"
#include "Representer.h"

namespace statismo
{
/* \class DataItem
 * \brief Holds all the information for a given sample.
 * Use GetSample() to recover a Sample
 * \warning This method generates a new object containing the sample. If the Representer does not provide a smart
 * pointer, the user is responsible for releasing memory.
 */
template <typename T>
class DataItem
{
public:
  typedef Representer<T>                               RepresenterType;
  typedef typename RepresenterType::DatasetPointerType DatasetPointerType;

  /**
   * Dtor
   */
  virtual ~DataItem() {}

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
  virtual const VectorType &
  GetSampleVector() const = 0;

  /**
   * Returns the sample in the representation given by the representer
   * \warning This method generates a new object containing the sample. If the Representer does not provide a smart
   * pointer, the user is responsible for releasing memory.
   */
  virtual const DatasetPointerType
  GetSample() const = 0;
};

/**
 * \brief Base class for data item
 */
template <typename T, typename Derived>
class DataItemBase : public DataItem<T>
  , public GenericFactory<Derived>
{
  public:
    using ObjectFactoryType = GenericFactory<Derived>;
    friend ObjectFactoryType;

  public:
  typedef Representer<T>                               RepresenterType;
  typedef typename RepresenterType::DatasetPointerType DatasetPointerType;

  /**
   * Dtor
   */
  virtual ~DataItemBase() {}

  /** Create a new DataItem object, using the data from the group in the HDF5 file
   * \param dsGroup. The group in the hdf5 file for this dataset
   */
  static DataItemBase *
  Load(const RepresenterType * representer, const H5::Group & dsGroup);
  
  /**
   *  Save the sample data to the hdf5 group dsGroup.
   */
  virtual void
  Save(const H5::Group & dsGroup) const;

  /**
   * Get the URI of the original dataset
   */
  std::string
  GetDatasetURI() const
  {
    return m_URI;
  }

  /**
   * Get the representer used to create this sample
   */
  const RepresenterType *
  GetRepresenter() const
  {
    return m_representer;
  }

  /**
   * Get the vectorial representation of this sample
   */
  const VectorType &
  GetSampleVector() const
  {
    return m_sampleVector;
  }

  /**
   * Returns the sample in the representation given by the representer
   * \warning This method generates a new object containing the sample. If the Representer does not provide a smart
   * pointer, the user is responsible for releasing memory.
   */
  const DatasetPointerType
  GetSample() const
  {
    return m_representer->SampleVectorToSample(m_sampleVector);
  }

protected:
  DataItemBase(const RepresenterType * representer, const std::string & URI, const VectorType & sampleVector)
    : m_representer(representer)
    , m_URI(URI)
    , m_sampleVector(sampleVector)
  {}

  DataItemBase(const RepresenterType * representer)
    : m_representer(representer)
  {}

  // loads the internal state from the hdf5 file
  virtual void
  LoadInternal(const H5::Group & dsGroup)
  {
    VectorType v;
    HDF5Utils::readVector(dsGroup, "./samplevector", m_sampleVector);
    m_URI = HDF5Utils::readString(dsGroup, "./URI");
  }

  virtual void
  SaveInternal(const H5::Group & dsGroup) const
  {
    HDF5Utils::writeVector(dsGroup, "./samplevector", m_sampleVector);
    HDF5Utils::writeString(dsGroup, "./URI", m_URI);
  }


  const RepresenterType * m_representer;
  std::string             m_URI;
  VectorType              m_sampleVector;

};

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

  BasicDataItem(const RepresenterType * representer)
    : Superclass(representer)
  {}
};


/* \class DataItemWithSurrogates
 * \brief Holds all the information for a given sample.
 * Use GetSample() to recover a Sample
 * \warning This method generates a new object containing the sample. If the Representer does not provide a smart
 * pointer, the user is responsible for releasing memory. In particular, it enables to associate categorical or
 * continuous variables with a sample, in a vectorial representation. The vector is provided by a file providing the
 * values in ascii format (empty space or EOL separating the values) \sa DataItem \sa DataManagerWithSurrogates
 */

template <typename T>
class DataItemWithSurrogates : public DataItemBase<T, DataItemWithSurrogates<T>>
{
  using Superclass = DataItemBase<T, DataItemWithSurrogates<T>>;
  using RepresenterType = typename Superclass::RepresenterType;
  friend typename Superclass::ObjectFactoryType;
  
public:
  enum SurrogateType
  {
    Categorical = 0,
    Continuous = 1
  };


  typedef std::vector<SurrogateType> SurrogateTypeVectorType;

  virtual ~DataItemWithSurrogates() {}

  const VectorType &
  GetSurrogateVector() const
  {
    return m_surrogateVector;
  }
  const std::string &
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

  DataItemWithSurrogates(const RepresenterType * r)
    : Superclass(r)
  {}

  // loads the internal state from the hdf5 file
  virtual void
  LoadInternal(const H5::Group & dsGroup)
  {
    Superclass::LoadInternal(dsGroup);
    VectorType v;
    HDF5Utils::readVector(dsGroup, "./surrogateVector", this->m_surrogateVector);
    m_surrogateFilename = HDF5Utils::readString(dsGroup, "./surrogateFilename");
  }

  virtual void
  SaveInternal(const H5::Group & dsGroup) const
  {
    Superclass::SaveInternal(dsGroup);
    HDF5Utils::writeVector(dsGroup, "./surrogateVector", this->m_surrogateVector);
    HDF5Utils::writeString(dsGroup, "./surrogateFilename", this->m_surrogateFilename);
  }

  std::string m_surrogateFilename;
  VectorType  m_surrogateVector;
};


} // namespace statismo

#include "DataItem.hxx"

#endif // __SAMPLE_DATA_H
