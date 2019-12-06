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
#include "utils/statismo-build-models-utils.h"

#include <statismo/ITK/itkDataManager.h>
#include <itkDirectory.h>
#include <itkImage.h>
#include <itkLandmarkBasedTransformInitializer.h>
#include <itkMesh.h>
#include <itkMeshFileReader.h>
#include <itkMeshFileWriter.h>
#include <statismo/ITK/itkPCAModelBuilder.h>
#include <itkRigid3DTransform.h>
#include <statismo/ITK/itkStandardMeshRepresenter.h>
#include <statismo/ITK/itkIO.h>
#include <statismo/ITK/itkStatisticalModel.h>
#include <itkTransformMeshFilter.h>

namespace po = lpo;
using namespace std;

struct ProgramOptions
{
  string strDataListFile;
  string strProcrustesMode;
  string strProcrustesReferenceFile;
  string strOutputFileName;
  float  fNoiseVariance;
};

bool
isOptionsConflictPresent(ProgramOptions & opt);
void
buildAndSaveShapeModel(const ProgramOptions & opt);

int
main(int argc, char ** argv)
{
  ProgramOptions                           poParameters;
  lpo::program_options<std::string, float> parser{ argv[0], "Program help:" };

  parser
    .add_opt<std::string>(
      { "data-list", "l", "File containing a list of meshes to build shape model from", &poParameters.strDataListFile },
      true)
    .add_opt<std::string>({ "procrustes",
                            "p",
                            "Specify how the data is aligned: REFERENCE aligns all datasets rigidly to the reference "
                            "and GPA alignes all datasets to the population mean.",
                            &poParameters.strProcrustesMode,
                            "GPA" })
    .add_opt<std::string>(
      { "reference",
        "r",
        "Specify the reference used for model building. This is needed if --procrustes is REFERENCE",
        &poParameters.strProcrustesReferenceFile })
    .add_opt<float>({ "noise", "n", "Noise variance of the PPCA model", &poParameters.fNoiseVariance, 0.0f, 0.0f })
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
    buildAndSaveShapeModel(poParameters);
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
isOptionsConflictPresent(ProgramOptions & opt)
{
  statismo::utils::ToLower(opt.strProcrustesMode);

  if (opt.strProcrustesMode != "reference" && opt.strProcrustesMode != "gpa")
  {
    return true;
  }

  if (opt.strProcrustesMode == "reference" && opt.strProcrustesReferenceFile == "")
  {
    return true;
  }

  if (opt.strProcrustesMode == "gpa" && opt.strProcrustesReferenceFile != "")
  {
    return true;
  }

  if (opt.strDataListFile == "" || opt.strOutputFileName == "")
  {
    return true;
  }

  if (opt.strDataListFile == opt.strOutputFileName)
  {
    return true;
  }

  if (opt.strProcrustesMode == "reference")
  {
    if (opt.strDataListFile == opt.strProcrustesReferenceFile ||
        opt.strOutputFileName == opt.strProcrustesReferenceFile)
    {
      return true;
    }
  }

  return false;
}

void
buildAndSaveShapeModel(const ProgramOptions & opt)
{
  const unsigned Dimensions = 3;

  typedef itk::StandardMeshRepresenter<float, Dimensions> RepresenterType;
  RepresenterType::Pointer                                representer = RepresenterType::New();

  typedef itk::Mesh<float, Dimensions> MeshType;
  typedef itk::DataManager<MeshType>   DataManagerType;
  DataManagerType::Pointer             dataManager = DataManagerType::New();

  StringList fileNames = getFileList(opt.strDataListFile);

  typedef itk::MeshFileReader<MeshType>   MeshReaderType;
  typedef vector<MeshReaderType::Pointer> MeshReaderList;
  MeshReaderList                          meshes;
  meshes.reserve(fileNames.size());
  for (StringList::const_iterator it = fileNames.begin(); it != fileNames.end(); ++it)
  {
    MeshReaderType::Pointer reader = MeshReaderType::New();
    reader->SetFileName(it->c_str());
    reader->Update();
    // itk::PCAModelBuilder is not a Filter in the ITK world, so the pipeline would not get executed if its main method
    // is called. So the pipeline before calling itk::PCAModelBuilder must be executed by the means of calls to Update()
    // (at least for last elements needed by itk::PCAModelBuilder).
    meshes.push_back(reader);
  }

  if (meshes.size() == 0)
  {
    itkGenericExceptionMacro(<< "The specified data-list is empty.");
  }

  if (opt.strProcrustesMode == "reference")
  {
    MeshReaderType::Pointer refReader = MeshReaderType::New();
    refReader->SetFileName(opt.strProcrustesReferenceFile);
    refReader->Update();
    representer->SetReference(refReader->GetOutput());
  }
  else
  {
    vector<MeshType::Pointer> originalMeshes;
    for (MeshReaderList::iterator it = meshes.begin(); it != meshes.end(); ++it)
    {
      MeshReaderType::Pointer reader = *it;
      originalMeshes.push_back(reader->GetOutput());
    }

    const unsigned uMaxGPAIterations = 20;
    const unsigned uNumberOfPoints = 100;
    const float    fBreakIfChangeBelow = 0.001f;

    typedef itk::VersorRigid3DTransform<float> Rigid3DTransformType;
    typedef itk::Image<float, Dimensions>      ImageType;
    typedef itk::LandmarkBasedTransformInitializer<Rigid3DTransformType, ImageType, ImageType>
                                                                               LandmarkBasedTransformInitializerType;
    typedef itk::TransformMeshFilter<MeshType, MeshType, Rigid3DTransformType> FilterType;
    MeshType::Pointer                                                          referenceMesh =
      calculateProcrustesMeanMesh<MeshType, LandmarkBasedTransformInitializerType, Rigid3DTransformType, FilterType>(
        originalMeshes, uMaxGPAIterations, uNumberOfPoints, fBreakIfChangeBelow);
    representer->SetReference(referenceMesh);
  }

  dataManager->SetRepresenter(representer);

  for (MeshReaderList::const_iterator it = meshes.begin(); it != meshes.end(); ++it)
  {
    MeshReaderType::Pointer reader = *it;
    dataManager->AddDataset(reader->GetOutput(), reader->GetFileName());
  }

  typedef itk::StatisticalModel<MeshType> StatisticalModelType;
  StatisticalModelType::Pointer           model;
  typedef itk::PCAModelBuilder<MeshType>  PCAModelBuilder;
  PCAModelBuilder::Pointer                pcaModelBuilder = PCAModelBuilder::New();
  model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), opt.fNoiseVariance);
  itk::StatismoIO<MeshType>::SaveStatisticalModel(model, opt.strOutputFileName.c_str());
}
