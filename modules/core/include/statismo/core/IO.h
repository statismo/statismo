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

#ifndef __STATISMO_IO_H_
#define __STATISMO_IO_H_

#include "statismo/core/StatisticalModel.h"

namespace H5
{
class Group;
}

namespace statismo
{
/**
 * \brief The IO class is used to Load() and or Save() a StatisticalModel. The Load and Save functions are static and as
 * such there's no need to create an instance of this class.
 *
 * The Template parameter is the same as the one of the StatisticalModel class.
 *
 * \note Here we use a bunch of static methods instead of namespaces because it
 * is easier to manipulate (just alias the class instantiation instead of setting
 * the template param for each function) and the class can be passed as template
 * parameter.
 *
 */
template <typename T>
class IO
{
private:
  // This class is made up of static methods only and as such the Constructor is private to prevent misunderstandings.
  IO() = default;

public:
  using StatisticalModelType = StatisticalModel<T>;

  /**
   * Returns a new statistical model, which is loaded from the given HDF5 file
   * \param filename The filename
   * \param maxNumberOfPCAComponents The maximal number of pca components that are loaded
   * to create the model.
   */
  static UniquePtrType<StatisticalModelType>
  LoadStatisticalModel(typename StatisticalModelType::RepresenterType * representer,
                       const std::string &                              filename,
                       unsigned maxNumberOfPCAComponents = std::numeric_limits<unsigned>::max())
  {

    UniquePtrType<StatisticalModelType> newModel;

    H5::H5File file;
    try
    {
      file = H5::H5File(filename.c_str(), H5F_ACC_RDONLY);
    }
    catch (const H5::Exception & e)
    {
      std::string msg(std::string("could not open HDF5 file \n") + e.getCDetailMsg());
      throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
    }

    return LoadStatisticalModel(representer, file.openGroup("/"), maxNumberOfPCAComponents);
  }

  /**
   * Returns a new statistical model, which is stored in the given HDF5 Group
   *
   * \param modelroot A h5 group where the model is saved
   * \param maxNumberOfPCAComponents The maximal number of pca components that are loaded
   * to create the model.
   */
  static UniquePtrType<StatisticalModelType>
  LoadStatisticalModel(typename StatisticalModelType::RepresenterType * representer,
                       const H5::Group &                                modelRoot,
                       unsigned maxNumberOfPCAComponents = std::numeric_limits<unsigned>::max())
  {

    UniquePtrType<StatisticalModelType> newModel;
    ModelInfo                           modelInfo;

    try
    {
      representer->Load(modelRoot.openGroup("./representer"));

      int minorVersion = 0;
      int majorVersion = 0;

      if (hdf5utils::ExistsObjectWithName(modelRoot, "version") == false)
      {
        // this is an old statismo format, that had not been versioned. We set the version to 0.8 as this is the last
        // version that stores the old format
        std::cout << "Warning: version attribute does not exist in hdf5 file. Assuming version 0.8" << std::endl;
        minorVersion = s_oldFileVersionMinor;
        majorVersion = s_oldFileVersionMajor;
      }
      else
      {
        auto versionGroup = modelRoot.openGroup("./version");
        minorVersion = hdf5utils::ReadInt(versionGroup, "./minorVersion");
        majorVersion = hdf5utils::ReadInt(versionGroup, "./majorVersion");
      }

      auto       modelGroup = modelRoot.openGroup("./model");
      VectorType mean;
      hdf5utils::ReadVector(modelGroup, "./mean", mean);
      VectorType pcaVariance;
      hdf5utils::ReadVector(modelGroup, "./pcaVariance", maxNumberOfPCAComponents, pcaVariance);

      auto noiseVariance = hdf5utils::ReadFloat(modelGroup, "./noiseVariance");

      // Depending on the statismo version, the pcaBasis matrix was stored as U*D or U (where U are the orthonormal PCA
      // Basis functions and D the standard deviations). Here we make sure that we fill the pcaBasisMatrix (which
      // statismo stores as U*D) with the right values.
      MatrixType pcaBasisMatrix;
      if (majorVersion == s_oldFileVersionMinor && minorVersion == s_oldFileVersionMajor)
      {
        hdf5utils::ReadMatrix(modelGroup, "./pcaBasis", maxNumberOfPCAComponents, pcaBasisMatrix);
        VectorType D = pcaVariance.array().sqrt();
        MatrixType orthonormalPCABasisMatrix = pcaBasisMatrix * DiagMatrixType(D).inverse();
        newModel =
          StatisticalModelType::SafeCreate(representer, mean, orthonormalPCABasisMatrix, pcaVariance, noiseVariance);
      }
      else if (majorVersion == s_currenFileVersionMajor && minorVersion == s_currentFileVersionMinor)
      {
        hdf5utils::ReadMatrix(modelGroup, "./pcaBasis", maxNumberOfPCAComponents, pcaBasisMatrix);
        newModel = StatisticalModelType::SafeCreate(representer, mean, pcaBasisMatrix, pcaVariance, noiseVariance);
      }
      else
      {
        std::ostringstream os;
        os << "an invalid statismo version was provided (" << majorVersion << "." << minorVersion << ")";
        throw StatisticalModelException(os.str().c_str(), Status::BAD_VERSION_ERROR);
      }

      modelInfo.Load(modelRoot);
    }
    catch (H5::Exception & e)
    {
      std::string msg(std::string("an exeption occured while reading HDF5 file") +
                      "The most likely cause is that the hdf5 file does not contain the required objects. \n" +
                      e.getCDetailMsg());
      throw StatisticalModelException(msg.c_str(), Status::INVALID_DATA_ERROR);
    }

    newModel->SetModelInfo(modelInfo);
    return newModel;
  }


  /**
   * Saves the statistical model to a HDF5 file
   * \param model A pointer to the model you'd like to save.
   * \param filename The filename (preferred extension is .h5)
   * */
  static void
  SaveStatisticalModel(const StatisticalModelType * const model, const std::string & filename)
  {
    if (!model)
    {
      throw new StatisticalModelException("invalid null model", Status::BAD_INPUT_ERROR);
    }
    SaveStatisticalModel(*model, filename);
  }

  /**
   * Saves the statistical model to a HDF5 file
   * \param model The model to save
   * \param filename The filename (preferred extension is .h5)
   * */
  static void
  SaveStatisticalModel(const StatisticalModelType & model, const std::string & filename)
  {
    using namespace H5;

    H5File        file;
    std::ifstream ifile(filename.c_str());

    try
    {
      file = H5::H5File(filename.c_str(), H5F_ACC_TRUNC);
    }
    catch (H5::FileIException & e)
    {
      std::string msg(std::string("Could not open HDF5 file for writing \n") + e.getCDetailMsg());
      throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
    }

    auto modelRoot = file.openGroup("/");
    auto versionGroup = modelRoot.createGroup("version");

    hdf5utils::WriteInt(versionGroup, "majorVersion", 0);
    hdf5utils::WriteInt(versionGroup, "minorVersion", 9);

    SaveStatisticalModel(model, modelRoot);
  };

  /**
   * Saves the statistical model to the given HDF5 group.
   * \param model the model you'd like to save
   * \param modelRoot the group where to store the model
   * */
  static void
  SaveStatisticalModel(const StatisticalModelType & model, const H5::Group & modelRoot)
  {
    try
    {
      // create the group structure
      auto dataTypeStr = TypeToString(model.GetRepresenter()->GetType());
      auto representerGroup = modelRoot.createGroup("./representer");

      hdf5utils::WriteStringAttribute(representerGroup, "name", model.GetRepresenter()->GetName());
      hdf5utils::WriteStringAttribute(representerGroup, "version", model.GetRepresenter()->GetVersion());
      hdf5utils::WriteStringAttribute(representerGroup, "datasetType", dataTypeStr);

      model.GetRepresenter()->Save(representerGroup);

      auto modelGroup = modelRoot.createGroup("./model");
      hdf5utils::WriteMatrix(modelGroup, "./pcaBasis", model.GetOrthonormalPCABasisMatrix());
      hdf5utils::WriteVector(modelGroup, "./pcaVariance", model.GetPCAVarianceVector());
      hdf5utils::WriteVector(modelGroup, "./mean", model.GetMeanVector());
      hdf5utils::WriteFloat(modelGroup, "./noiseVariance", model.GetNoiseVariance());

      model.GetModelInfo().Save(modelRoot);
    }
    catch (H5::Exception & e)
    {
      std::string msg(std::string("an exception occurred while writing HDF5 file \n") + e.getCDetailMsg());
      throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
    }
  }
};

} // namespace statismo

#endif
