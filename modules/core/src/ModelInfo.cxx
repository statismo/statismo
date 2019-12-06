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

#include "statismo/core/DataManager.h"
#include "statismo/core/Exceptions.h"
#include "statismo/core/HDF5Utils.h"
#include "statismo/core/ModelInfo.h"

#include <ctime>
#include <iostream>
#include <ctime>
#include <memory>
#include <vector>

namespace statismo
{

ModelInfo::ModelInfo(const MatrixType & scores, const ModelInfo::BuilderInfoList & builderInfos)
  : m_scores(scores)
  , m_builderInfo(builderInfos)
{}

ModelInfo::ModelInfo(const MatrixType & scores)
  : m_scores(scores)
{}

ModelInfo::BuilderInfoList
ModelInfo::GetBuilderInfoList() const
{
  return m_builderInfo;
}

const MatrixType &
ModelInfo::GetScoresMatrix() const
{
  return m_scores;
}

void
ModelInfo::Save(const H5::H5Location & publicFg) const
{
  using namespace H5;

  // get time and date
  time_t      rawtime;
  struct tm * timeinfo;
  std::time(&rawtime);
  timeinfo = std::localtime(&rawtime);

  try
  {
    Group publicInfo = publicFg.createGroup("./modelinfo");
    hdf5utils::WriteString(publicInfo, "./build-time", std::asctime(timeinfo));
    if (m_scores.rows() != 0 && m_scores.cols() != 0)
    {
      hdf5utils::WriteMatrix(publicInfo, "./scores", m_scores);
    }
    else
    {
      // HDF5 does not allow us to write empty matrices. Therefore, we write a dummy matrix with 1 element
      hdf5utils::WriteMatrix(publicInfo, "./scores", MatrixType::Zero(1, 1));
    }

    for (unsigned i = 0; i < m_builderInfo.size(); i++)
    {
      std::ostringstream ss;
      ss << "./modelBuilder-" << i;
      m_builderInfo[i].Save(publicInfo.createGroup(ss.str().c_str()));
    }
  }
  catch (const H5::Exception & e)
  {
    std::string msg(std::string("an exception occurred while writing model info HDF5 file \n") + e.getCDetailMsg());
    throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
  }
}

void
ModelInfo::Load(const H5::H5Location & publicFg)
{
  using namespace H5;
  auto publicModelGroup = publicFg.openGroup("./modelinfo");
  try
  {
    hdf5utils::ReadMatrix(publicModelGroup, "./scores", m_scores);
  }
  catch (const H5::Exception & e)
  {
    // the likely cause is that there are no scores. so we set them as empty
    m_scores.resize(0, 0);
  }

  if (m_scores.cols() == 1 && m_scores.rows() == 1 && m_scores(0, 0) == 0.0)
  {
    // we observed a dummy matrix, that was created when saving the model info.
    m_scores.resize(0, 0);
  }

  m_builderInfo.clear();
  auto numEntries = publicModelGroup.getNumObjs();

  for (unsigned i = 0; i < numEntries; i++)
  {
    H5std_string key = publicModelGroup.getObjnameByIdx(i);

    // Compatibility to older statismo file-format.
    // if we find at this level a dataInfo object, then it needs to be an old statismo file.
    if (key.find("dataInfo") != std::string::npos || key.find("builderInfo") != std::string::npos)
    {
      m_builderInfo.emplace_back(LoadDataInfoOldStatismoFormat(publicModelGroup));
      // we have all the information that is stored in the info block of an old statismo file.
      // hence we can leave
      break;
    }

    // check for all modelBuilder objects and compile them into a list
    if (key.find("modelBuilder") != std::string::npos)
    {
      auto        modelBuilderGroup = publicModelGroup.openGroup(key.c_str());
      BuilderInfo bi;
      bi.Load(modelBuilderGroup);
      m_builderInfo.emplace_back(std::move(bi));
    }
  }
}

inline BuilderInfo
ModelInfo::LoadDataInfoOldStatismoFormat(const H5::H5Location & publicModelGroup) const
{
  using namespace H5;

  auto                      dataInfoGroup = publicModelGroup.openGroup("./dataInfo");
  BuilderInfo::KeyValueList dataInfo;
  BuilderInfo::FillKeyValueListFromInfoGroup(dataInfoGroup, dataInfo);

  auto                      builderInfoGroup = publicModelGroup.openGroup("./builderInfo");
  BuilderInfo::KeyValueList paramInfo;
  BuilderInfo::FillKeyValueListFromInfoGroup(builderInfoGroup, paramInfo);

  auto buildTime = hdf5utils::ReadString(publicModelGroup, "build-time");

  // add the information to a new BuilderInfo object
  // as a first step we need to find the builderName from the parameter list
  std::string builderName = "";
  for (auto it = std::begin(paramInfo); it != std::end(paramInfo); ++it)
  {
    if (it->first.find("BuilderName") != std::string::npos)
    {
      builderName = it->second;
      paramInfo.erase(it);
      break;
    }
  }

  return BuilderInfo(builderName, buildTime, dataInfo, paramInfo);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BuilderInfo
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

BuilderInfo::BuilderInfo(const std::string &                    modelBuilderName,
                         const std::string &                    buildTime,
                         const BuilderInfo::DataInfoList &      di,
                         const BuilderInfo::ParameterInfoList & pi)
  : m_modelBuilderName(modelBuilderName)
  , m_buildtime(buildTime)
  , m_dataInfo(di)
  , m_parameterInfo(pi)
{}

BuilderInfo::BuilderInfo(const std::string &                    modelBuilderName,
                         const BuilderInfo::DataInfoList &      di,
                         const BuilderInfo::ParameterInfoList & pi)
  : m_modelBuilderName(modelBuilderName)
  , m_dataInfo(di)
  , m_parameterInfo(pi)
{

  // get time and date
  time_t      rawtime;
  struct tm * timeinfo;

  std::time(&rawtime);
  timeinfo = std::localtime(&rawtime);
  m_buildtime = std::asctime(timeinfo);
}

void
BuilderInfo::Save(const H5::H5Location & modelBuilderGroup) const
{
  using namespace H5;

  try
  {
    hdf5utils::WriteString(modelBuilderGroup, "./builderName", m_modelBuilderName);
    hdf5utils::WriteString(modelBuilderGroup, "./buildTime", m_buildtime);

    auto dataInfoGroup = modelBuilderGroup.createGroup("./dataInfo");
    for (const auto & it : m_dataInfo)
    {
      hdf5utils::WriteString(dataInfoGroup, it.first.c_str(), it.second.c_str());
    }

    auto parameterGroup = modelBuilderGroup.createGroup("./parameters");
    for (const auto & it : m_parameterInfo)
    {
      hdf5utils::WriteString(parameterGroup, it.first.c_str(), it.second.c_str());
    }
  }
  catch (const H5::Exception & e)
  {
    std::string msg(std::string("an exception occurred while writing model info HDF5 file \n") + e.getCDetailMsg());
    throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
  }
}

void
BuilderInfo::Load(const H5::H5Location & modelBuilderGroup)
{
  using namespace H5;

  m_modelBuilderName = hdf5utils::ReadString(modelBuilderGroup, "./builderName");
  m_buildtime = hdf5utils::ReadString(modelBuilderGroup, "./buildTime");

  auto dataInfoGroup = modelBuilderGroup.openGroup("./dataInfo");
  FillKeyValueListFromInfoGroup(dataInfoGroup, m_dataInfo);

  auto parameterGroup = modelBuilderGroup.openGroup("./parameters");
  FillKeyValueListFromInfoGroup(parameterGroup, m_parameterInfo);
}

const BuilderInfo::DataInfoList &
BuilderInfo::GetDataInfo() const
{
  return m_dataInfo;
}

const BuilderInfo::ParameterInfoList &
BuilderInfo::GetParameterInfo() const
{
  return m_parameterInfo;
}

void
BuilderInfo::FillKeyValueListFromInfoGroup(const H5::H5Location & group, KeyValueList & keyValueList)
{
  keyValueList.clear();
  unsigned numEntries = group.getNumObjs();
  for (unsigned i = 0; i < numEntries; i++)
  {
    auto key = group.getObjnameByIdx(i);
    auto value = hdf5utils::ReadString(group, key.c_str());
    keyValueList.emplace_back(key, value);
  }
}


} // namespace statismo
