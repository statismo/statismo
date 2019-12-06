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

#include <itkImage.h>
#include <itkMesh.h>
#include <statismo/ITK/itkReducedVarianceModelBuilder.h>
#include <statismo/ITK/itkStandardImageRepresenter.h>
#include <statismo/ITK/itkStandardMeshRepresenter.h>
#include <statismo/ITK/itkIO.h>
#include <statismo/ITK/itkStatisticalModel.h>

#include <string>


namespace po = lpo;
using namespace std;

struct ProgramOptions
{
  string   strInputFileName;
  string   strOutputFileName;
  unsigned uNumberOfComponents;
  unsigned uNumberOfDimensions;
  double   dTotalVariance;
  string   strType;
};

bool
isOptionsConflictPresent(ProgramOptions & opt);
template <class DataType, class RepresenterType>
void
reduceModel(const ProgramOptions & opt);

int
main(int argc, char ** argv)
{
  ProgramOptions                                      poParameters;
  lpo::program_options<std::string, unsigned, double> parser{ argv[0], "Program help:" };

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
    .add_opt<std::string>({ "input-file", "i", "The path to the model file.", &poParameters.strInputFileName }, true)
    .add_opt<unsigned>({ "numcomponents",
                         "n",
                         "Creates a new model with the specified number of components.",
                         &poParameters.uNumberOfComponents,
                         0 })
    .add_opt<double>({ "totalvariance",
                       "v",
                       "Creates a new Model that will have a fraction of the old models' variance. This parameter is "
                       "in percent and thus ranges from 0 to 100.",
                       &poParameters.dTotalVariance,
                       0.0f,
                       0.0f,
                       100.0f })
    .add_pos_opt<std::string>(
      { "Name of the output file where the reduced model will be written to.", &poParameters.strOutputFileName });

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
    const unsigned Dimensionality2D = 2;
    const unsigned Dimensionality3D = 3;
    if (poParameters.strType == "shape")
    {
      typedef itk::Mesh<float, Dimensionality3D>                    DataType;
      typedef itk::StandardMeshRepresenter<float, Dimensionality3D> RepresenterType;
      reduceModel<DataType, RepresenterType>(poParameters);
    }
    else
    {
      if (poParameters.uNumberOfDimensions == Dimensionality2D)
      {
        typedef itk::Vector<float, Dimensionality2D>                             VectorPixelType;
        typedef itk::Image<VectorPixelType, Dimensionality2D>                    DataType;
        typedef itk::StandardImageRepresenter<VectorPixelType, Dimensionality2D> RepresenterType;
        reduceModel<DataType, RepresenterType>(poParameters);
      }
      else
      {
        typedef itk::Vector<float, Dimensionality3D>                             VectorPixelType;
        typedef itk::Image<VectorPixelType, Dimensionality3D>                    DataType;
        typedef itk::StandardImageRepresenter<VectorPixelType, Dimensionality3D> RepresenterType;
        reduceModel<DataType, RepresenterType>(poParameters);
      }
    }
  }
  catch (itk::ExceptionObject & e)
  {
    cerr << "Could not reduce the model:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

bool
isOptionsConflictPresent(ProgramOptions & opt)
{
  statismo::utils::ToLower(opt.strType);

  if (opt.strType != "shape" && opt.strType != "deformation")
  {
    return true;
  }

  if (opt.dTotalVariance != 0 && opt.uNumberOfComponents != 0)
  {
    return true;
  }

  if (opt.dTotalVariance == 0 && opt.uNumberOfComponents == 0)
  {
    return true;
  }

  if (opt.strInputFileName == "" || opt.strOutputFileName == "")
  {
    return true;
  }


  if (opt.strInputFileName == opt.strOutputFileName)
  {
    return true;
  }

  return false;
}


template <class DataType, class RepresenterType>
void
reduceModel(const ProgramOptions & opt)
{
  typename RepresenterType::Pointer pRepresenter = RepresenterType::New();


  typedef typename itk::StatisticalModel<DataType> StatisticalModelType;
  typename StatisticalModelType::Pointer           pModel = StatisticalModelType::New();

  pModel = itk::StatismoIO<DataType>::LoadStatisticalModel(pRepresenter, opt.strInputFileName.c_str());

  typedef typename itk::ReducedVarianceModelBuilder<DataType> ReducedVarianceModelBuilderType;
  typename ReducedVarianceModelBuilderType::Pointer pReducedVarModelBuilder = ReducedVarianceModelBuilderType::New();
  typename StatisticalModelType::Pointer            pOutputModel;

  if (opt.uNumberOfComponents != 0)
  {
    if (opt.uNumberOfComponents > pModel->GetNumberOfPrincipalComponents())
    {
      itkGenericExceptionMacro(<< "The model has " << pModel->GetNumberOfPrincipalComponents()
                               << " components. Upscaling models to more componenets isn't possible with this tool.");
    }
    pOutputModel =
      pReducedVarModelBuilder->BuildNewModelWithLeadingComponents(pModel.GetPointer(), opt.uNumberOfComponents);
  }
  else
  {
    pOutputModel = pReducedVarModelBuilder->BuildNewModelWithVariance(pModel.GetPointer(), opt.dTotalVariance);
  }

  itk::StatismoIO<DataType>::SaveStatisticalModel(pOutputModel, opt.strOutputFileName.c_str());
}
