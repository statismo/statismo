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
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkWarpImageFilter.h>

#include <string>

const unsigned Dimensionality3D = 3;
const unsigned Dimensionality2D = 2;

namespace po = lpo;
using namespace std;


struct ProgramOptions
{
  string   strInputImageFileName;
  string   strInputDeformFieldFileName;
  string   strOutputFileName;
  unsigned uNumberOfDimensions;
};

bool
isOptionsConflictPresent(const ProgramOptions & opt);
template <unsigned Dimensions>
void
applyDeformationFieldToImage(const ProgramOptions & opt);

int
main(int argc, char ** argv)
{

  ProgramOptions                              poParameters;
  lpo::program_options<std::string, unsigned> parser{ argv[0], "Program help:" };

  parser
    .add_opt<unsigned>({ "dimensionality",
                         "d",
                         "Dimensionality of the input image (only available if you're building a deformation model)",
                         &poParameters.uNumberOfDimensions,
                         3,
                         2,
                         3 },
                       true)
    .add_opt<std::string>(
      { "input-image", "i", "The path to the original image.", &poParameters.strInputImageFileName }, true)
    .add_opt<std::string>(
      { "input-deformation-field", "f", "The path to the original image.", &poParameters.strInputDeformFieldFileName },
      true)
    .add_pos_opt<std::string>({ "Name of the warped output image.", &poParameters.strOutputFileName });

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
      applyDeformationFieldToImage<Dimensionality2D>(poParameters);
    }
    else
    {
      applyDeformationFieldToImage<Dimensionality3D>(poParameters);
    }
  }
  catch (itk::ExceptionObject & e)
  {
    cerr << "Could not warp the image:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

bool
isOptionsConflictPresent(const ProgramOptions & opt)
{
  if (opt.strInputDeformFieldFileName == "" || opt.strInputImageFileName == "" || opt.strOutputFileName == "")
  {
    return true;
  }
  return false;
}

template <unsigned Dimensions>
void
applyDeformationFieldToImage(const ProgramOptions & opt)
{
  typedef itk::Image<float, Dimensions>   ImageType;
  typedef itk::ImageFileReader<ImageType> ImageReaderType;
  typename ImageReaderType::Pointer       pOriginalImageReader = ImageReaderType::New();
  pOriginalImageReader->SetFileName(opt.strInputImageFileName.c_str());
  pOriginalImageReader->Update();
  typename ImageType::Pointer pOriginalImage = pOriginalImageReader->GetOutput();

  typedef itk::Vector<float, Dimensions>          VectorPixelType;
  typedef itk::Image<VectorPixelType, Dimensions> VectorImageType;
  typedef itk::ImageFileReader<VectorImageType>   VectorImageReaderType;
  typename VectorImageReaderType::Pointer         pDeformationFieldReader = VectorImageReaderType::New();
  pDeformationFieldReader->SetFileName(opt.strInputDeformFieldFileName.c_str());
  pDeformationFieldReader->Update();
  typename VectorImageType::Pointer pDeformationField = pDeformationFieldReader->GetOutput();

  typedef itk::LinearInterpolateImageFunction<ImageType, double> InterpolatorType;
  typename InterpolatorType::Pointer                             pInterpolator = InterpolatorType::New();

  typedef itk::WarpImageFilter<ImageType, ImageType, VectorImageType> WarpFilterType;
  typename WarpFilterType::Pointer                                    pWarper = WarpFilterType::New();
  pWarper->SetInput(pOriginalImage);
  pWarper->SetInterpolator(pInterpolator);
  pWarper->SetOutputSpacing(pOriginalImage->GetSpacing());
  pWarper->SetOutputOrigin(pOriginalImage->GetOrigin());
  pWarper->SetOutputDirection(pOriginalImage->GetDirection());
  pWarper->SetDisplacementField(pDeformationField);
  pWarper->Update();
  typename ImageType::Pointer pWarpedImage = pWarper->GetOutput();

  typedef itk::ImageFileWriter<ImageType> ImageWriterType;
  typename ImageWriterType::Pointer       pImageWriter = ImageWriterType::New();
  pImageWriter->SetInput(pWarpedImage);
  pImageWriter->SetFileName(opt.strOutputFileName.c_str());
  pImageWriter->Update();
}