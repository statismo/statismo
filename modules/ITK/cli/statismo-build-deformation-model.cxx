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

#include <statismo/ITK/itkDataManager.h>
#include <itkDirectory.h>
#include <itkImageFileReader.h>
#include <statismo/ITK/itkPCAModelBuilder.h>
#include <statismo/ITK/itkStandardImageRepresenter.h>
#include <statismo/ITK/itkIO.h>
#include <statismo/ITK/itkStatisticalModel.h>

#include "utils/statismo-build-models-utils.h"

namespace po = lpo;
using namespace std;

struct ProgramOptions
{
  bool     bComputeScores{ true };
  string   strDataListFile;
  string   strOutputFileName;
  float    fNoiseVariance;
  unsigned uNumberOfDimensions;
};

bool
isOptionsConflictPresent(const ProgramOptions & opt);
template <unsigned Dimensions>
void
buildAndSaveDeformationModel(const ProgramOptions & opt);

int
main(int argc, char ** argv)
{
  ProgramOptions                                           poParameters;
  lpo::program_options<std::string, float, unsigned, bool> parser{ argv[0], "Program help:" };

  parser
    .add_opt<std::string>({ "data-list",
                            "l",
                            "File containing a list of meshes to build the deformation model from",
                            &poParameters.strDataListFile },
                          true)
    .add_opt<unsigned>(
      { "dimensionality", "d", "Dimensionality of the input image", &poParameters.uNumberOfDimensions, 3, 2, 3 }, true)
    .add_opt<float>({ "noise", "n", "Noise variance of the PPCA model", &poParameters.fNoiseVariance, 0.0f, 0.0f })
    .add_opt<bool>({ "scores", "s", "Compute scores (default true)", &poParameters.bComputeScores, true })
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
    if (poParameters.uNumberOfDimensions == 2)
    {
      buildAndSaveDeformationModel<2>(poParameters);
    }
    else
    {
      buildAndSaveDeformationModel<3>(poParameters);
    }
  }
  catch (ifstream::failure & e)
  {
    cerr << "Could not read the data-list:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }
  catch (itk::ExceptionObject & e)
  {
    cerr << "Could not build the model:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

bool
isOptionsConflictPresent(const ProgramOptions & opt)
{
  if (opt.strDataListFile == "" || opt.strOutputFileName == "")
  {
    return true;
  }

  if (opt.strDataListFile == opt.strOutputFileName)
  {
    return true;
  }

  return false;
}

template <unsigned Dimensions>
void
buildAndSaveDeformationModel(const ProgramOptions & opt)
{
  typedef itk::Vector<float, Dimensions>                             VectorPixelType;
  typedef itk::Image<VectorPixelType, Dimensions>                    ImageType;
  typedef itk::StandardImageRepresenter<VectorPixelType, Dimensions> RepresenterType;
  typename RepresenterType::Pointer                                  representer = RepresenterType::New();

  typedef itk::DataManager<ImageType> DataManagerType;
  typename DataManagerType::Pointer   dataManager = DataManagerType::New();

  StringList fileNames = getFileList(opt.strDataListFile);

  typedef itk::ImageFileReader<ImageType>           ImageReaderType;
  typedef vector<typename ImageReaderType::Pointer> ImageReaderList;

  if (fileNames.size() == 0)
  {
    itkGenericExceptionMacro(<< "No Data was loaded and thus the model can't be built.");
  }
  bool firstPass = true;
  for (StringList::const_iterator it = fileNames.begin(); it != fileNames.end(); ++it)
  {

    typename ImageReaderType::Pointer reader = ImageReaderType::New();

    reader->SetFileName(it->c_str());
    reader->Update();
    if (firstPass)
    {
      representer->SetReference(reader->GetOutput());
      dataManager->SetRepresenter(representer);
      firstPass = false;
    }
    dataManager->AddDataset(reader->GetOutput(), reader->GetFileName().c_str());
  }

  typedef itk::StatisticalModel<ImageType> StatisticalModelType;
  typename StatisticalModelType::Pointer   model;

  typedef itk::PCAModelBuilder<ImageType> ModelBuilderType;
  typename ModelBuilderType::Pointer      pcaModelBuilder = ModelBuilderType::New();
  model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), opt.fNoiseVariance, opt.bComputeScores);
  itk::StatismoIO<ImageType>::SaveStatisticalModel(model, opt.strOutputFileName.c_str());
}
