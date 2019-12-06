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
#include <itkImage.h>
#include <itkMesh.h>
#include <itkMeshFileReader.h>
#include <itkMeshFileWriter.h>
#include <statismo/ITK/itkStandardImageRepresenter.h>
#include <statismo/ITK/itkStandardMeshRepresenter.h>
#include <statismo/ITK/itkIO.h>
#include <statismo/ITK/itkStatisticalModel.h>

#include <iostream>
#include <set>
#include <string>


const unsigned Dimensionality3D = 3;
const unsigned Dimensionality2D = 2;

namespace po = lpo;
using namespace std;

typedef vector<string> StringList;

struct ProgramOptions
{
  string     strInputFileName;
  string     strOutputFileName;
  string     strType;
  StringList vParameters;
  bool       bSampleMean;
  bool       bSampleReference;
  unsigned   uNumberOfDimensions;
};

bool
isOptionsConflictPresent(ProgramOptions & opt);
template <class DataType, class RepresenterType, class DataWriterType>
void
drawSampleFromModel(const ProgramOptions & opt);
template <class VectorType>
void
populateVectorWithParameters(const StringList & vParams, VectorType & vParametersReturnVector);

int
main(int argc, char ** argv)
{

  ProgramOptions                                                poParameters;
  lpo::program_options<std::string, StringList, bool, unsigned> parser{ argv[0], "Program help:" };

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
    .add_flag({ "mean", "m", "Draws the mean from the model and saves it.", &poParameters.bSampleMean })
    .add_flag({ "reference", "r", "Draws the reference from the model and saves it.", &poParameters.bSampleReference })
    .add_opt<StringList>(
      { "parameters",
        "p",
        "Makes it possible to specify a list of parameters and their positions that will then be used to draw a "
        "sample. Parameters are speciefied in the following format: POSITION1:VALUE1 POSITIONn:VALUEn. Unspecified "
        "parameters will be set to 0. The first parameter is at position 1.",
        &poParameters.vParameters })
    .add_pos_opt<std::string>({ "Name of the output file/the sample.", &poParameters.strOutputFileName });

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
      typedef itk::Mesh<float, Dimensionality3D>                    DataType;
      typedef itk::StandardMeshRepresenter<float, Dimensionality3D> RepresenterType;
      typedef itk::MeshFileWriter<DataType>                         DataWriterType;
      drawSampleFromModel<DataType, RepresenterType, DataWriterType>(poParameters);
    }
    else
    {
      if (poParameters.uNumberOfDimensions == 2)
      {
        typedef itk::Vector<float, Dimensionality2D>                             VectorPixelType;
        typedef itk::Image<VectorPixelType, Dimensionality2D>                    DataType;
        typedef itk::StandardImageRepresenter<VectorPixelType, Dimensionality2D> RepresenterType;
        typedef itk::ImageFileWriter<DataType>                                   DataWriterType;
        drawSampleFromModel<DataType, RepresenterType, DataWriterType>(poParameters);
      }
      else
      {
        typedef itk::Vector<float, Dimensionality3D>                             VectorPixelType;
        typedef itk::Image<VectorPixelType, Dimensionality3D>                    DataType;
        typedef itk::StandardImageRepresenter<VectorPixelType, Dimensionality3D> RepresenterType;
        typedef itk::ImageFileWriter<DataType>                                   DataWriterType;
        drawSampleFromModel<DataType, RepresenterType, DataWriterType>(poParameters);
      }
    }
  }
  catch (itk::ExceptionObject & e)
  {
    cerr << "Could not get a sample:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

bool
isOptionsConflictPresent(ProgramOptions & opt)
{
  statismo::utils::ToLower(opt.strType);

  if (opt.bSampleMean + (opt.vParameters.size() > 0) + opt.bSampleReference > 1)
  {
    return true;
  }

  if (opt.strInputFileName == "" || opt.strOutputFileName == "")
  {
    return true;
  }

  if (opt.strType != "shape" && opt.strType != "deformation")
  {
    return true;
  }

  if (opt.strInputFileName == opt.strOutputFileName)
  {
    return true;
  }

  return false;
}


template <class VectorType>
void
populateVectorWithParameters(const StringList & vParams, VectorType & vParametersReturnVector)
{
  set<unsigned> sSeenIndices;

  for (StringList::const_iterator i = vParams.begin(); i != vParams.end(); ++i)
  {
    StringList vSplit;
    bool       bSuccess = true;
    vSplit = statismo::utils::Split<':'>(*i);
    if (vSplit.size() != 2)
    {
      bSuccess = false;
    }
    else
    {
      try
      {
        unsigned uIndex = statismo::utils::LexicalCast<unsigned>(vSplit[0]) - 1;
        double   dValue = statismo::utils::LexicalCast<double>(vSplit[1]);

        if (uIndex >= vParametersReturnVector.size())
        {
          itkGenericExceptionMacro(
            << "The parameter '" << *i
            << "' is has an index value that is not in the range of this model's available parameters (1 to "
            << vParametersReturnVector.size() << ").");
        }

        if (sSeenIndices.find(uIndex) == sSeenIndices.end())
        {
          sSeenIndices.insert(uIndex);
          vParametersReturnVector[uIndex] = dValue;
        }
        else
        {
          itkGenericExceptionMacro(<< "The index '" << (uIndex + 1)
                                   << "' occurs more than once in the parameter list.");
        }
      }
      catch (const std::bad_cast &)
      {
        bSuccess = false;
      }
    }

    if (bSuccess == false)
    {
      itkGenericExceptionMacro(
        << "The parameter '" << *i
        << "' is in an incorrect format. The correct format is index:value. Like for example 0:1.1 or 19:-2");
    }
  }
}

template <class DataType, class RepresenterType, class DataWriterType>
void
drawSampleFromModel(const ProgramOptions & opt)
{
  typename RepresenterType::Pointer pRepresenter = RepresenterType::New();

  typedef itk::StatisticalModel<DataType> StatisticalModelType;
  typename StatisticalModelType::Pointer  pModel = StatisticalModelType::New();

  pModel = itk::StatismoIO<DataType>::LoadStatisticalModel(pRepresenter, opt.strInputFileName.c_str());

  typename DataType::Pointer output;
  if (opt.bSampleMean == true)
  {
    output = pModel->DrawMean();
  }
  else if (opt.vParameters.size() > 0)
  {
    unsigned                                  uNrOfParameters = pModel->GetNumberOfPrincipalComponents();
    typename StatisticalModelType::VectorType vModelParameters(uNrOfParameters);
    vModelParameters.fill(0);

    populateVectorWithParameters<typename StatisticalModelType::VectorType>(opt.vParameters, vModelParameters);

    output = pModel->DrawSample(vModelParameters);
  }
  else if (opt.bSampleReference == true)
  {
    output = pModel->GetRepresenter()->GetReference();
  }
  else
  {
    output = pModel->DrawSample();
  }

  typename DataWriterType::Pointer pDataWriter = DataWriterType::New();
  pDataWriter->SetFileName(opt.strOutputFileName.c_str());
  pDataWriter->SetInput(output);
  pDataWriter->Update();
}
