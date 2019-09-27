/*
 * Copyright (c) 2015 University of Basel
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

#include <itkCompensatedSummation.h>
#include <itkIdentityTransform.h>
#include <itkTransformMeshFilter.h>

#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

typedef std::list<std::string> StringList;
StringList
getFileList(std::string path);

template <class MeshType>
typename MeshType::Pointer
calculateMeanMesh(std::vector<typename MeshType::Pointer> meshes);

template <class MeshType>
float
calculateMeshDistance(typename MeshType::Pointer mesh1, typename MeshType::Pointer mesh2);

template <class MeshType, class LandmarkBasedTransformInitializerType, class TransformType, class FilterType>
std::vector<typename MeshType::Pointer>
superimposeMeshes(std::vector<typename MeshType::Pointer> originalMeshes,
                  typename MeshType::Pointer              referenceMesh,
                  std::set<unsigned>                      landmarkIndices);

template <class MeshType, class LandmarkBasedTransformInitializerType, class TransformType, class FilterType>
typename MeshType::Pointer
calculateProcrustesMeanMesh(std::vector<typename MeshType::Pointer> meshes,
                            unsigned                                maxIterations,
                            unsigned                                nrOfLandmarks,
                            float                                   breakIfChangeBelow);

template <class DataType>
typename DataType::Pointer
cloneMesh(typename DataType::Pointer pMesh);


template <class DataType>
typename DataType::Pointer
cloneMesh(typename DataType::Pointer pMesh)
{
  typedef itk::IdentityTransform<float, DataType::PointDimension>             IdentityTransformType;
  typedef itk::TransformMeshFilter<DataType, DataType, IdentityTransformType> TransformFilterType;
  typename TransformFilterType::Pointer   transformMeshFilter = TransformFilterType::New();
  typename IdentityTransformType::Pointer pIdentityTransform = IdentityTransformType::New();
  transformMeshFilter->SetInput(pMesh);
  transformMeshFilter->SetTransform(pIdentityTransform);
  transformMeshFilter->Update();
  return transformMeshFilter->GetOutput();
}

template <class MeshType, class LandmarkBasedTransformInitializerType, class TransformType, class FilterType>
typename MeshType::Pointer
calculateProcrustesMeanMesh(std::vector<typename MeshType::Pointer> meshes,
                            unsigned                                maxIterations,
                            unsigned                                nrOfLandmarks,
                            float                                   breakIfChangeBelow)
{
  // the initial mesh to which all others will be aligned to is the first one in the list here. Any other mesh could be
  // chosen as well
  typename MeshType::Pointer referenceMesh = *meshes.begin();

  unsigned rngSeed = time(0);
  unsigned meshVerticesCount = referenceMesh->GetNumberOfPoints();
  srand(rngSeed);
  std::set<unsigned> pointNumbers;
  while (pointNumbers.size() < std::min(nrOfLandmarks, meshVerticesCount))
  {
    unsigned randomIndex = ((unsigned)rand()) % meshVerticesCount;
    pointNumbers.insert(randomIndex);
  }

  float fPreviousDifference = -1;

  for (unsigned i = 0; i < maxIterations; ++i)
  {
    // calculate the difference to the previous iteration's mesh and break if the difference is very small
    std::vector<typename MeshType::Pointer> translatedMeshes =
      superimposeMeshes<MeshType, LandmarkBasedTransformInitializerType, TransformType, FilterType>(
        meshes, referenceMesh, pointNumbers);
    typename MeshType::Pointer meanMesh = calculateMeanMesh<MeshType>(translatedMeshes);
    float                      fDifference = calculateMeshDistance<MeshType>(meanMesh, referenceMesh);
    float                      fDifferenceDelta = std::abs(fDifference - fPreviousDifference);
    fPreviousDifference = fDifference;
    referenceMesh = meanMesh;

    if (fDifferenceDelta < breakIfChangeBelow)
    {
      break;
    }
  }
  return referenceMesh;
}

template <class MeshType, class LandmarkBasedTransformInitializerType, class TransformType, class FilterType>
std::vector<typename MeshType::Pointer>
superimposeMeshes(std::vector<typename MeshType::Pointer> originalMeshes,
                  typename MeshType::Pointer              referenceMesh,
                  std::set<unsigned>                      landmarkIndices)
{
  std::vector<typename MeshType::Pointer> translatedMeshes(originalMeshes.begin(), originalMeshes.end());
  for (typename std::vector<typename MeshType::Pointer>::iterator it = translatedMeshes.begin();
       it != translatedMeshes.end();
       ++it)
  {
    typedef typename LandmarkBasedTransformInitializerType::LandmarkPointContainer LandmarkContainerType;
    LandmarkContainerType                                                          movingLandmarks;
    LandmarkContainerType                                                          fixedLandmarks;
    typename MeshType::Pointer                                                     movingMesh = *it;

    if (movingMesh->GetNumberOfPoints() != referenceMesh->GetNumberOfPoints() ||
        movingMesh->GetNumberOfCells() != referenceMesh->GetNumberOfCells())
    {
      itkGenericExceptionMacro(<< "All meshes must have the same number of Edges & Vertices");
    }

    // Only use a subset of the meshes' points for the alignment since we don't have that many degrees of freedom
    // anyways and since calculating a SVD with too many points is expensive
    for (std::set<unsigned>::const_iterator rng = landmarkIndices.begin(); rng != landmarkIndices.end(); ++rng)
    {
      movingLandmarks.push_back(movingMesh->GetPoint(*rng));
      fixedLandmarks.push_back(referenceMesh->GetPoint(*rng));
    }

    // only rotate & translate the moving mesh to best fit with the fixed mesh; there's no scaling taking place.
    typename LandmarkBasedTransformInitializerType::Pointer landmarkBasedTransformInitializer =
      LandmarkBasedTransformInitializerType::New();
    landmarkBasedTransformInitializer->SetFixedLandmarks(fixedLandmarks);
    landmarkBasedTransformInitializer->SetMovingLandmarks(movingLandmarks);
    typename TransformType::Pointer transform = TransformType::New();
    transform->SetIdentity();
    landmarkBasedTransformInitializer->SetTransform(transform);
    landmarkBasedTransformInitializer->InitializeTransform();

    typename FilterType::Pointer filter = FilterType::New();
    filter->SetInput(movingMesh);
    filter->SetTransform(transform);
    filter->Update();

    *it = filter->GetOutput();
  }
  return translatedMeshes;
}


template <class MeshType>
float
calculateMeshDistance(typename MeshType::Pointer mesh1, typename MeshType::Pointer mesh2)
{
  if (mesh1->GetNumberOfPoints() != mesh2->GetNumberOfPoints() ||
      mesh1->GetNumberOfCells() != mesh2->GetNumberOfCells())
  {
    itkGenericExceptionMacro(<< "Both meshes must have the same number of Edges & Vertices");
  }

  float                                                fDifference = 0;
  typedef typename MeshType::PointsContainer::Iterator IteratorType;
  IteratorType                                         point1 = mesh1->GetPoints()->Begin();
  IteratorType                                         point2 = mesh2->GetPoints()->Begin();
  for (; point1 != mesh1->GetPoints()->End(); ++point1, ++point2)
  {
    fDifference += point1->Value().SquaredEuclideanDistanceTo(point2->Value());
  }
  fDifference /= (mesh1->GetNumberOfPoints() * MeshType::PointDimension);
  return fDifference;
}

template <class MeshType>
typename MeshType::Pointer
calculateMeanMesh(std::vector<typename MeshType::Pointer> meshes)
{
  if (meshes.size() == 0)
  {
    itkGenericExceptionMacro(<< "Can't calculate the mean since no meshes were provided.");
  }

  typedef itk::CompensatedSummation<typename MeshType::PixelType> CompensatedSummationType;
  typedef std::vector<CompensatedSummationType>                   MeshPointsVectorType;

  typename MeshType::Pointer pFirstMesh = *meshes.begin();

  // prepare for summation
  MeshPointsVectorType vMeshPoints;
  unsigned             uDataSize = pFirstMesh->GetNumberOfPoints() * MeshType::PointDimension;
  vMeshPoints.reserve(uDataSize);
  for (int i = 0; i < uDataSize; ++i)
  {
    CompensatedSummationType sum;
    vMeshPoints.push_back(sum);
  }

  for (typename std::vector<typename MeshType::Pointer>::const_iterator i = meshes.begin(); i != meshes.end(); ++i)
  {
    typename MeshType::Pointer pMesh = *i;
    if (vMeshPoints.size() != pMesh->GetNumberOfPoints() * MeshType::PointDimension)
    {
      itkGenericExceptionMacro(<< "All meshes must have the same number of Edges");
    }

    typename MeshPointsVectorType::iterator           sum = vMeshPoints.begin();
    typename MeshType::PointsContainer::ConstIterator pointData = pMesh->GetPoints()->Begin();
    // sum up all meshes
    for (; pointData != pMesh->GetPoints()->End(); ++pointData)
    {
      const typename MeshType::PointType point = pointData->Value();
      for (typename MeshType::PointType::ConstIterator pointIter = point.Begin(); pointIter != point.End();
           ++pointIter, ++sum)
      {
        (*sum) += *pointIter;
      }
    }
  }

  float                      fInvNumberOfMeshes = 1.0f / meshes.size();
  typename MeshType::Pointer pMeanMesh = cloneMesh<MeshType>(pFirstMesh);

  // write the data to the mean mesh
  typename MeshPointsVectorType::iterator sum = vMeshPoints.begin();
  for (typename MeshType::PointsContainer::Iterator pointData = pMeanMesh->GetPoints()->Begin();
       pointData != pMeanMesh->GetPoints()->End();
       ++pointData)
  {
    for (typename MeshType::PointType::Iterator pointIter = pointData->Value().Begin();
         pointIter != pointData->Value().End();
         ++pointIter, ++sum)
    {
      *pointIter = sum->GetSum() * fInvNumberOfMeshes;
    }
  }

  return pMeanMesh;
}


StringList
getFileList(std::string path)
{
  StringList fileList;

  std::ifstream file;
  try
  {
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    file.open(path.c_str(), std::ifstream::in);
    std::string line;
    while (getline(file, line))
    {
      if (line != "")
      {
        // reading files with windows EOL on linux results in the \r not being removed from the line ending
        if (*line.rbegin() == '\r')
        {
          line.erase(line.length() - 1, 1);
        }
        fileList.push_back(line);
      }
    }
  }
  catch (std::ifstream::failure e)
  {
    if (file.eof() == false)
    {
      throw std::ifstream::failure("Failed to read the file '" + path + "'.");
    }
  }

  return fileList;
}