/*
 * SampleData.h
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

#ifndef __SAMPLE_DATA_H
#define __SAMPLE_DATA_H

#include "CommonTypes.h"

namespace statismo {
/*
 * \brief Holds all the information for a given sample.
 */
template <typename Representer>
class SampleData {
public:
	typedef typename Representer::DatasetPointerType DatasetPointerType;

	/**
	 * Ctor. Usually not called from outside of the library
	 */
	static SampleData* Create(const Representer* representer, const std::string& filename, const VectorType& sampleVector)
	{
		return new SampleData(representer, filename, sampleVector);
	}

	/**
	 * Dtor
	 */
	virtual ~SampleData() {}

	/** Create a new SampleData object, using the data from the group in the HDF5 file
	 * \param dsGroup. The group in the hdf5 file for this dataset
	 */
	static SampleData* Load(const Representer* representer, const H5::Group& dsGroup);
	/**
	 *  Save the sample data to the hdf5 group dsGroup.
	 */
	virtual void Save(const H5::Group& dsGroup) const;

	/**
	 * Get the filename of the original dataset
	 */
	std::string GetDatasetFilename() const { return m_filename; }

	/**
	 * Get the representer used to create this sample
	 */
	const Representer* GetRepresenter() const { return m_representer; }

	/**
	 * Get the vectorial representation of this sample
	 */
	const VectorType& GetSampleVector() const { return m_sampleVector; }

	/**
	 * Returns the sample in the representation given by the representer
	 * \warning A new sample is created. The user is responsible for releasing it.
	 */
	const DatasetPointerType GetAsNewSample() const { return m_representer->SampleVectorToSample(m_sampleVector); }

protected:

	SampleData(const Representer* representer, const std::string& filename, const VectorType& sampleVector)
		: m_representer(representer), m_filename(filename), m_sampleVector(sampleVector)
	{
	}

	SampleData(const Representer* representer) : m_representer(representer)
	{}

	// loads the internal state from the hdf5 file
	virtual void LoadInternal(const H5::Group& dsGroup) {
		VectorType v;
		HDF5Utils::readVector(dsGroup, "./samplevector", m_sampleVector);
		m_filename = HDF5Utils::readString(dsGroup, "./filename");
	}

	virtual void SaveInternal(const H5::Group& dsGroup) const {
		HDF5Utils::writeVector(dsGroup, "./samplevector", m_sampleVector);
		HDF5Utils::writeString(dsGroup, "./filename", m_filename);
	}


	const Representer* m_representer;
	std::string m_filename;
	VectorType m_sampleVector;
};

template <typename Representer>
class SampleDataWithSurrogates : public SampleData<Representer>
{
	friend class SampleData<Representer>;

public:

	enum SurrogateType {
		Categorical = 0,
		Continuous = 1
	};


	typedef std::vector<SurrogateType>	SurrogateTypeVectorType;


	static SampleDataWithSurrogates* Create(const Representer* representer,
									 const std::string& datasetFilename,
									 const VectorType& sampleVector,
									 const std::string& surrogateFilename,
									 const VectorType& surrogateVector)
	{
		return new SampleDataWithSurrogates(representer, datasetFilename, sampleVector, surrogateFilename, surrogateVector);
	}




	virtual ~SampleDataWithSurrogates() {}

	const VectorType& GetSurrogateVector() const { return m_surrogateVector; }
	const std::string& GetSurrogateFilename() const { return m_surrogateFilename; }

private:

	SampleDataWithSurrogates(const Representer* representer,
							const std::string& datasetFilename,
							const VectorType& sampleVector,
							const std::string& surrogateFilename,
							const VectorType& surrogateVector)
	: SampleData<Representer>(representer, datasetFilename, sampleVector),
	  m_surrogateFilename(surrogateFilename),
	  m_surrogateVector(surrogateVector)
	{
	}

	SampleDataWithSurrogates(const Representer* r) : SampleData<Representer>(r) {}

	// loads the internal state from the hdf5 file
	virtual void LoadInternal(const H5::Group& dsGroup) {
		SampleData<Representer>::LoadInternal(dsGroup);
		VectorType v;
		HDF5Utils::readVector(dsGroup, "./surrogateVector", this->m_surrogateVector);
		m_surrogateFilename = HDF5Utils::readString(dsGroup, "./surrogateFilename");
	}

	virtual void SaveInternal(const H5::Group& dsGroup) const {
		SampleData<Representer>::SaveInternal(dsGroup);
		HDF5Utils::writeVector(dsGroup, "./surrogateVector", this->m_surrogateVector);
		HDF5Utils::writeString(dsGroup, "./surrogateFilename", this->m_surrogateFilename);
	}

	std::string m_surrogateFilename;
	VectorType m_surrogateVector;
};


} // namespace statismo

#include "SampleData.txx"

#endif // __SAMPLE_DATA_H

