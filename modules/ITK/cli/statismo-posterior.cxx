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

#include "lpo.h"

#include <itkCompositeTransform.h>
#include <itkImage.h>
#include <itkMesh.h>
#include <itkPointsLocator.h>
#include <statismo/ITK/itkStandardImageRepresenter.h>
#include <statismo/ITK/itkStandardMeshRepresenter.h>
#include <statismo/ITK/itkIO.h>
#include <statismo/ITK/itkStatisticalModel.h>
#include <itkVersorRigid3DTransform.h>

#include "utils/statismo-fitting-utils.h"

const unsigned Dimensionality3D = 3;
const unsigned Dimensionality2D = 2;

namespace po = lpo;
using namespace std;

struct ProgramOptions
{
  string strModelType;
  string strInputModelFileName;
  string strOutputModelFileName;

  string strInputMeshFileName;

  string strInputFixedLandmarksFileName;
  string strInputMovingLandmarksFileName;
  double dVariance;

  unsigned uNumberOfDimensions;
};

bool
isOptionsConflictPresent(ProgramOptions & opt);
void
buildPosteriorShapeModel(const ProgramOptions & opt);
template <unsigned Dimensionality>
void
buildPosteriorDeformationModel(const ProgramOptions & opt);


int
main(int argc, char ** argv)
{

  ProgramOptions                                      poParameters;
  lpo::program_options<std::string, unsigned, double> parser{ argv[0], "Program help:" };

  parser
    .add_opt<std::string>({ "type",
                            "t",
                            "Specifies the type of the model: SHAPE and DEFORMATION are the two available types",
                            &poParameters.strModelType,
                            "SHAPE" },
                          true)
    .add_opt<unsigned>({ "dimensionality",
                         "d",
                         "Dimensionality of the input image (only available if you're building a deformation model)",
                         &poParameters.uNumberOfDimensions,
                         3,
                         2,
                         3 },
                       true)
    .add_opt<std::string>({ "input-file",
                            "i",
                            "The path to the model file from which the posterior model will be built.",
                            &poParameters.strInputModelFileName },
                          true)
    .add_opt<std::string>({ "landmarks-fixed",
                            "f",
                            "Name of the file where the fixed Landmarks are saved.",
                            &poParameters.strInputFixedLandmarksFileName })
    .add_opt<std::string>({ "landmarks-moving",
                            "m",
                            "Name of the file where the moving Landmarks are saved.",
                            &poParameters.strInputMovingLandmarksFileName })
    .add_opt<std::string>({ "corresponding-mesh",
                            "c",
                            "Path to the Mesh in correspondence. This is only available if the type is SHAPE.",
                            &poParameters.strInputMeshFileName })
    .add_opt<double>({ "landmarks-variance",
                       "v",
                       "The variance that will be used to build the posterior model.",
                       &poParameters.dVariance,
                       1.0f })
    .add_pos_opt<std::string>({ "Name of the output file where the posterior model will be written to.",
                                &poParameters.strOutputModelFileName });

  if (!parser.parse(argc, argv))
  {
    return EXIT_FAILURE;
  }

  if (isOptionsConflictPresent(poParameters))
  {
    cerr << "A conflict in the options exists or insufficient options were set." << endl;
    cout << parser << endl;
    return EXIT_FAILURE;
  }

  try
  {
    if (poParameters.strModelType == "shape")
    {
      buildPosteriorShapeModel(poParameters);
    }
    else
    {
      if (poParameters.uNumberOfDimensions == Dimensionality2D)
      {
        buildPosteriorDeformationModel<Dimensionality2D>(poParameters);
      }
      else
      {
        buildPosteriorDeformationModel<Dimensionality3D>(poParameters);
      }
    }
  }
  catch (ifstream::failure & e)
  {
    cerr << "Could not read a file:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }
  catch (itk::ExceptionObject & e)
  {
    cerr << "Could not build the posterior model:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

bool
isOptionsConflictPresent(ProgramOptions & opt)
{
  // if one set of the landmarks-file is provided, then both have to be provided (-> use XOR)
  if (((opt.strInputFixedLandmarksFileName != "") ^ (opt.strInputMovingLandmarksFileName != "")) == true)
  {
    return true;
  }

  // either a mesh is used as reference or landmarks are used. It's not possible to use both at the same time or use
  // none. (-> again XOR)
  if (((opt.strInputFixedLandmarksFileName != "") ^ (opt.strInputMeshFileName != "")) == false)
  {
    return true;
  }

  if (opt.strInputModelFileName == "" || opt.strOutputModelFileName == "")
  {
    return true;
  }

  statismo::utils::ToLower(opt.strModelType);
  if (opt.strModelType != "shape" && opt.strModelType != "deformation")
  {
    return true;
  }

  // A mesh in correspondence can only be used if the model type is shape
  if (opt.strModelType == "deformation" && opt.strInputMeshFileName != "")
  {
    return true;
  }

  if (opt.strModelType == "deformation" && opt.uNumberOfDimensions != 2 && opt.uNumberOfDimensions != 3)
  {
    return true;
  }

  if (opt.strModelType == "shape" && opt.uNumberOfDimensions != 3)
  {
    return true;
  }

  return false;
}


void
buildPosteriorShapeModel(const ProgramOptions & opt)
{
  typedef itk::Mesh<float, Dimensionality3D>                    DataType;
  typedef itk::StatisticalModel<DataType>                       StatisticalModelType;
  typedef itk::StandardMeshRepresenter<float, Dimensionality3D> RepresenterType;
  RepresenterType::Pointer                                      pRepresenter = RepresenterType::New();
  StatisticalModelType::Pointer                                 pModel = StatisticalModelType::New();
  pModel =
    itk::StatismoIO<DataType>::LoadStatisticalModel(pRepresenter.GetPointer(), opt.strInputModelFileName.c_str());

  StatisticalModelType::Pointer pConstrainedModel;
  if (opt.strInputMeshFileName == "")
  {
    typedef itk::PointsLocator<DataType::PointsContainer> PointsLocatorType;
    pConstrainedModel = buildPosteriorShapeModel<DataType, StatisticalModelType, PointsLocatorType>(
      pModel, opt.strInputFixedLandmarksFileName, opt.strInputMovingLandmarksFileName, opt.dVariance);
  }
  else
  {
    typedef itk::MeshFileReader<DataType> MeshReaderType;
    MeshReaderType::Pointer               pMeshReader = MeshReaderType::New();
    pMeshReader->SetFileName(opt.strInputMeshFileName.c_str());
    pMeshReader->Update();
    DataType::Pointer pMeshInCorrespondence = pMeshReader->GetOutput();
    pConstrainedModel =
      buildPosteriorShapeModel<DataType, StatisticalModelType>(pModel, pMeshInCorrespondence, opt.dVariance);
  }

  itk::StatismoIO<DataType>::SaveStatisticalModel(pConstrainedModel, opt.strOutputModelFileName.c_str());
}

template <unsigned Dimensionality>
void
buildPosteriorDeformationModel(const ProgramOptions & opt)
{
  typedef itk::Vector<float, Dimensionality>                             VectorPixelType;
  typedef itk::Image<VectorPixelType, Dimensionality>                    DataType;
  typedef itk::StatisticalModel<DataType>                                StatisticalModelType;
  typedef itk::StandardImageRepresenter<VectorPixelType, Dimensionality> RepresenterType;
  typename RepresenterType::Pointer                                      pRepresenter = RepresenterType::New();
  typename StatisticalModelType::Pointer                                 pModel = StatisticalModelType::New();
  pModel =
    itk::StatismoIO<DataType>::LoadStatisticalModel(pRepresenter.GetPointer(), opt.strInputModelFileName.c_str());

  typename StatisticalModelType::Pointer pConstrainedModel;
  pConstrainedModel = buildPosteriorDeformationModel<DataType, StatisticalModelType>(
    pModel, opt.strInputFixedLandmarksFileName, opt.strInputMovingLandmarksFileName, opt.dVariance);

  itk::StatismoIO<DataType>::SaveStatisticalModel(pConstrainedModel, opt.strOutputModelFileName.c_str());
}