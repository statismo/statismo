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

#include <string>

#include <boost/program_options.hpp>

#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkWarpImageFilter.h>


const unsigned Dimensionality3D = 3;
const unsigned Dimensionality2D = 2;

namespace po = boost::program_options;
using namespace std;


struct programOptions {
    bool bDisplayHelp;
    string strInputImageFileName;
    string strInputDeformFieldFileName;
    string strOutputFileName;
    unsigned uNumberOfDimensions;
};

po::options_description initializeProgramOptions(programOptions& poParameters);
bool isOptionsConflictPresent(programOptions& opt);
template <unsigned Dimensions>
void applyDeformationFieldToImage(programOptions& opt);



int main(int argc, char** argv) {
    programOptions poParameters;

    po::positional_options_description optPositional;
    optPositional.add("output-file", 1);
    po::options_description optAllOptions = initializeProgramOptions(poParameters);


    po::variables_map vm;
    try {
        po::parsed_options parsedOptions = po::command_line_parser(argc, argv).options(optAllOptions).positional(optPositional).run();
        po::store(parsedOptions, vm);
        po::notify(vm);
    } catch (po::error& e) {
        cerr << "An exception occurred while parsing the Command line:"<<endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    if (poParameters.bDisplayHelp == true) {
        cout << optAllOptions << endl;
        return EXIT_SUCCESS;
    }
    if (isOptionsConflictPresent(poParameters) == true)	{
        cerr << "A conflict in the options exists or insufficient options were set." << endl;
        cout << optAllOptions << endl;
        return EXIT_FAILURE;
    }

    try {
        if (poParameters.uNumberOfDimensions == 2) {
            applyDeformationFieldToImage<Dimensionality2D>(poParameters);
        } else {
            applyDeformationFieldToImage<Dimensionality3D>(poParameters);
        }
    } catch (itk::ExceptionObject & e) {
        cerr << "Could not warp the image:" << endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

bool isOptionsConflictPresent(programOptions& opt) {
    if (opt.strInputDeformFieldFileName == "" || opt.strInputImageFileName == "" || opt.strOutputFileName == "") {
        return true;
    }
    return false;
}


template <unsigned Dimensions>
void applyDeformationFieldToImage(programOptions& opt) {
    typedef itk::Image<float, Dimensions> ImageType;
    typedef itk::ImageFileReader<ImageType> ImageReaderType;
    typename ImageReaderType::Pointer pOriginalImageReader = ImageReaderType::New();
    pOriginalImageReader->SetFileName(opt.strInputImageFileName.c_str());
    pOriginalImageReader->Update();
    typename ImageType::Pointer pOriginalImage = pOriginalImageReader->GetOutput();

    typedef itk::Vector<float, Dimensions> VectorPixelType;
    typedef itk::Image<VectorPixelType, Dimensions> VectorImageType;
    typedef itk::ImageFileReader<VectorImageType> VectorImageReaderType;
    typename VectorImageReaderType::Pointer pDeformationFieldReader = VectorImageReaderType::New();
    pDeformationFieldReader->SetFileName(opt.strInputDeformFieldFileName.c_str());
    pDeformationFieldReader->Update();
    typename VectorImageType::Pointer pDeformationField = pDeformationFieldReader->GetOutput();

    typedef itk::LinearInterpolateImageFunction<ImageType, double> InterpolatorType;
    typename InterpolatorType::Pointer pInterpolator = InterpolatorType::New();

    typedef itk::WarpImageFilter<ImageType, ImageType, VectorImageType> WarpFilterType;
    typename WarpFilterType::Pointer pWarper = WarpFilterType::New();
    pWarper->SetInput(pOriginalImage);
    pWarper->SetInterpolator(pInterpolator);
    pWarper->SetOutputSpacing(pOriginalImage->GetSpacing());
    pWarper->SetOutputOrigin(pOriginalImage->GetOrigin());
    pWarper->SetOutputDirection(pOriginalImage->GetDirection());
    pWarper->SetDisplacementField(pDeformationField);
    pWarper->Update();
    typename ImageType::Pointer pWarpedImage = pWarper->GetOutput();

    typedef itk::ImageFileWriter<ImageType>  ImageWriterType;
    typename ImageWriterType::Pointer pImageWriter = ImageWriterType::New();
    pImageWriter->SetInput(pWarpedImage);
    pImageWriter->SetFileName(opt.strOutputFileName.c_str());
    pImageWriter->Update();
}


po::options_description initializeProgramOptions(programOptions& poParameters) {
    po::options_description optMandatory("Mandatory options");
    optMandatory.add_options()
    ("dimensionality,d", po::value<unsigned>(&poParameters.uNumberOfDimensions)->default_value(3), "Dimensionality of the input image and deformation field.")
    ("input-image,i", po::value<string>(&poParameters.strInputImageFileName), "The path to the original image.")
    ("input-deformation-field,f", po::value<string>(&poParameters.strInputDeformFieldFileName), "The path to the deformation field that will be applied ot the original image.")
    ("output-file,o", po::value<string>(&poParameters.strOutputFileName), "Name of the warped output image.")
    ;
    po::options_description optAdditional("Optional options");
    optAdditional.add_options()
    ("help,h", po::bool_switch(&poParameters.bDisplayHelp), "Display this help message")
    ;

    po::options_description optAllOptions;
    optAllOptions.add(optMandatory).add(optAdditional);
    return optAllOptions;
}