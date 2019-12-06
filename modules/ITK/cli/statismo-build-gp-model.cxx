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

// Add new kernels in this file (and document their usage in the statismo-build-gp-model.md file)
#include "utils/statismo-build-gp-model-kernels.h"
#include "statismo/core/Utils.h"
#include "lpo.h"

#include <statismo/ITK/itkDataManager.h>
#include <itkDirectory.h>
#include <itkImageFileReader.h>
#include <statismo/ITK/itkLowRankGPModelBuilder.h>
#include <itkMeshFileReader.h>
#include <statismo/ITK/itkStandardImageRepresenter.h>
#include <statismo/ITK/itkStandardMeshRepresenter.h>
#include <statismo/ITK/itkIO.h>
#include <statismo/ITK/itkStatisticalModel.h>

#include <iostream>
#include <memory>

namespace po = lpo;
using namespace std;

struct ProgramOptions
{
  string         strOptionalModelPath;
  string         strReferenceFile;
  string         strKernel;
  string         strType;
  vector<string> vKernelParameters;
  float          fKernelScale;
  int            iNrOfBasisFunctions;
  unsigned       uNumberOfDimensions;
  string         strOutputFileName;
};

bool
isOptionsConflictPresent(ProgramOptions & opt);
template <class DataType, class RepresenterType, class DataReaderType, bool isShapeModel, unsigned Dimenstionality>
void
buildAndSaveModel(const ProgramOptions & opt);
string
getAvailableKernelsStr();


int
main(int argc, char ** argv)
{
  statismo::cli::createKernelMap();

  ProgramOptions poParameters;
  string         kernelHelp =
    "Specifies the kernel (covariance function). The following kernels are available: " + getAvailableKernelsStr();
  lpo::program_options<std::string, float, int, unsigned, std::vector<std::string>> parser{ argv[0], "Program help:" };

  parser
    .add_opt<std::string>({ "type",
                            "t",
                            "Specifies the type of the model: shape and deformation are the two available types",
                            &poParameters.strType,
                            "shape" },
                          true)
    .add_opt<unsigned>({ "dimensionality",
                         "d",
                         "Dimensionality of the input image (only available if you're building a deformation model)",
                         &poParameters.uNumberOfDimensions,
                         3,
                         2,
                         3 },
                       true)
    .add_opt<std::string>({ "kernel", "k", kernelHelp, &poParameters.strKernel }, true)
    .add_opt<std::vector<std::string>>({ "parameters",
                                         "p",
                                         "Specifies the kernel parameters. The Parameters depend on the kernel",
                                         &poParameters.vKernelParameters },
                                       true)
    .add_opt<float>(
      { "scale", "s", "A Scaling factor with which the Kernel will be scaled", &poParameters.fKernelScale, 1.0f }, true)
    .add_opt<int>({ "numberofbasisfunctions",
                    "n",
                    "Number of basis functions/parameters the model will have",
                    &poParameters.iNrOfBasisFunctions,
                    0,
                    1 },
                  true)
    .add_opt<std::string>(
      { "reference", "r", "The reference that will be used to build the model", &poParameters.strReferenceFile })
    .add_opt<std::string>({ "input-model",
                            "m",
                            "Extends an existing model with data from the specified kernel. This is useful to extend "
                            "existing models in case of insufficient data.",
                            &poParameters.strOptionalModelPath })
    .add_pos_opt<std::string>({ "Name of the output file", &poParameters.strOutputFileName });


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
    if (poParameters.strType == "shape")
    {
      typedef itk::StandardMeshRepresenter<float, statismo::cli::Dimensionality3D> RepresenterType;
      typedef itk::MeshFileReader<statismo::cli::DataTypeShape>                    DataReaderType;
      buildAndSaveModel<statismo::cli::DataTypeShape,
                        RepresenterType,
                        DataReaderType,
                        true,
                        statismo::cli::Dimensionality3D>(poParameters);
    }
    else
    {
      if (poParameters.uNumberOfDimensions == 2)
      {
        typedef itk::StandardImageRepresenter<statismo::cli::VectorPixel2DType, statismo::cli::Dimensionality2D>
                                                                           RepresenterType;
        typedef itk::ImageFileReader<statismo::cli::DataType2DDeformation> DataReaderType;
        buildAndSaveModel<statismo::cli::DataType2DDeformation,
                          RepresenterType,
                          DataReaderType,
                          false,
                          statismo::cli::Dimensionality2D>(poParameters);
      }
      else
      {
        typedef itk::StandardImageRepresenter<statismo::cli::VectorPixel3DType, statismo::cli::Dimensionality3D>
                                                                           RepresenterType;
        typedef itk::ImageFileReader<statismo::cli::DataType3DDeformation> DataReaderType;
        buildAndSaveModel<statismo::cli::DataType3DDeformation,
                          RepresenterType,
                          DataReaderType,
                          false,
                          statismo::cli::Dimensionality3D>(poParameters);
      }
    }
  }
  catch (itk::ExceptionObject & e)
  {
    cerr << "Could not build the model:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  std::cout << "ok" << std::endl;

  return EXIT_SUCCESS;
}

bool
isOptionsConflictPresent(ProgramOptions & opt)
{
  statismo::utils::ToLower(opt.strKernel);
  statismo::utils::ToLower(opt.strType);

  if (opt.strType != "shape" && opt.strType != "deformation")
  {
    return true;
  }

  if ((opt.strOptionalModelPath == "" && opt.strReferenceFile == "") ||
      (opt.strOptionalModelPath != "" & opt.strReferenceFile != ""))
  {
    return true;
  }

  if (opt.strOutputFileName == opt.strReferenceFile)
  {
    return true;
  }

  if (opt.strType == "deformation" && opt.uNumberOfDimensions != 2 && opt.uNumberOfDimensions != 3)
  {
    return true;
  }

  if (opt.strType == "shape" && opt.uNumberOfDimensions != 3)
  {
    return true;
  }

  return false;
}

template <class DataType, class RepresenterType, class DataReaderType, bool isShapeModel, unsigned Dimenstionality>
void
buildAndSaveModel(const ProgramOptions & opt)
{
  auto it = statismo::cli::sKernelMap.find(opt.strKernel);
  if (it == std::end(statismo::cli::sKernelMap))
  {
    itkGenericExceptionMacro(<< "The kernel '" << opt.strKernel
                             << "' isn't available. Available kernels: " << getAvailableKernelsStr());
  }

  typedef typename DataType::PointType                                   PointType;
  typedef std::unique_ptr<const statismo::ScalarValuedKernel<PointType>> MatrixPointerType;
  MatrixPointerType                                                      pKernel;
  if (isShapeModel == true)
  {
    pKernel.reset((statismo::ScalarValuedKernel<PointType> *)it->second.createKernelShape(opt.vKernelParameters));
  }
  else
  {
    if (Dimenstionality == statismo::cli::Dimensionality2D)
    {
      pKernel.reset(
        (statismo::ScalarValuedKernel<PointType> *)it->second.createKernel2DDeformation(opt.vKernelParameters));
    }
    else
    {
      pKernel.reset(
        (statismo::ScalarValuedKernel<PointType> *)it->second.createKernel3DDeformation(opt.vKernelParameters));
    }
  }

  typedef std::shared_ptr<statismo::MatrixValuedKernel<PointType>> KernelPointerType;
  KernelPointerType                                                pUnscaledKernel(
    new statismo::UncorrelatedMatrixValuedKernel<PointType>(pKernel.get(), Dimenstionality));
  KernelPointerType pScaledKernel(new statismo::ScaledKernel<PointType>(pUnscaledKernel.get(), opt.fKernelScale));
  KernelPointerType pStatModelKernel;
  KernelPointerType pModelBuildingKernel;

  typedef statismo::StatisticalModel<DataType>         RawModelType;
  typedef statismo::UniquePtrType<RawModelType>        RawModelPointerType;
  typedef typename RepresenterType::DatasetPointerType DatasetPointerType;

  RawModelPointerType               pRawStatisticalModel;
  typename RepresenterType::Pointer pRepresenter = RepresenterType::New();
  DatasetPointerType                pMean;


  if (opt.strOptionalModelPath != "")
  {
    try
    {
      pRawStatisticalModel =
        statismo::IO<DataType>::LoadStatisticalModel(pRepresenter.GetPointer(), opt.strOptionalModelPath.c_str());
      pStatModelKernel.reset(new statismo::StatisticalModelKernel<DataType>(pRawStatisticalModel.get()));
      pModelBuildingKernel.reset(new statismo::SumKernel<PointType>(pStatModelKernel.get(), pScaledKernel.get()));
    }
    catch (statismo::StatisticalModelException & s)
    {
      itkGenericExceptionMacro(<< "Failed to read the optional model: " << s.what());
    }
    pMean = pRawStatisticalModel->DrawMean();
  }
  else
  {
    pModelBuildingKernel = pScaledKernel;
    typename DataReaderType::Pointer pReferenceReader = DataReaderType::New();
    pReferenceReader->SetFileName(opt.strReferenceFile);
    pReferenceReader->Update();
    pRepresenter->SetReference(pReferenceReader->GetOutput());
    pMean = pRepresenter->IdentitySample();
  }

  typedef itk::LowRankGPModelBuilder<DataType> ModelBuilderType;
  typename ModelBuilderType::Pointer           gpModelBuilder = ModelBuilderType::New();
  gpModelBuilder->SetRepresenter(pRepresenter);

  typedef itk::StatisticalModel<DataType> StatisticalModelType;
  typename StatisticalModelType::Pointer  pModel;
  pModel = gpModelBuilder->BuildNewModel(pMean, *pModelBuildingKernel.get(), opt.iNrOfBasisFunctions);

  itk::StatismoIO<DataType>::SaveStatisticalModel(pModel, opt.strOutputFileName.c_str());
}

string
getAvailableKernelsStr()
{
  string ret;

  for (const auto & p : statismo::cli::sKernelMap)
  {
    ret += p.first + ",";
  }

  ret.pop_back();
  return ret;
}
