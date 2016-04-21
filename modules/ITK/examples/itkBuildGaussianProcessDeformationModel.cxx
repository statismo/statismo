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
#include <sys/types.h>
#include <errno.h>
#include <iostream>

#include <itkDirectory.h>
#include <itkImageFileReader.h>

#include "itkDataManager.h"
#include "itkLowRankGPModelBuilder.h"
#include "itkStandardImageRepresenter.h"
#include "itkStatismoIO.h"
#include "itkStatisticalModel.h"

#include "Kernels.h"
#include "KernelCombinators.h"

/*
 * This example shows the ITK Wrapping of statismo can be used to build a deformation model.
 */


typedef itk::Image<float, 3> ImageType3D;
typedef itk::Image< itk::Vector<float, 3> ,3 > VectorImageType3D;
typedef itk::StandardImageRepresenter<itk::Vector<float, 3>, 3> RepresenterType3D;

typedef itk::Image<float, 2> ImageType2D;
typedef itk::Image< itk::Vector<float, 2> ,2 > VectorImageType2D;
typedef itk::StandardImageRepresenter<itk::Vector<float, 2>, 2> RepresenterType2D;




/**
 * A scalar valued gaussian kernel.
 */
template <class TPoint>
class GaussianKernel: public statismo::ScalarValuedKernel<TPoint> {
  public:
    typedef typename  TPoint::CoordRepType CoordRepType;
    typedef vnl_vector<CoordRepType> VectorType;

    GaussianKernel(double sigma) : m_sigma(sigma), m_sigma2(sigma * sigma) {}

    inline double operator()(const TPoint& x, const TPoint& y) const {
        VectorType xv = x.GetVnlVector();
        VectorType yv = y.GetVnlVector();

        VectorType r = yv - xv;
        return exp(-dot_product(r, r) / m_sigma2);
    }

    std::string GetKernelInfo() const {
        std::ostringstream os;
        os << "GaussianKernel(" << m_sigma << ")";
        return os.str();
    }

  private:

    double m_sigma;
    double m_sigma2;
};




template <class RepresenterType, class ImageType>
void itkExample(const char* referenceFilename, double gaussianKernelSigma, const char* modelname) {



    typedef itk::LowRankGPModelBuilder<ImageType> ModelBuilderType;
    typedef itk::StatisticalModel<ImageType> StatisticalModelType;
    typedef std::vector<std::string> StringVectorType;
    typedef itk::ImageFileReader<ImageType> ImageFileReaderType;
    typedef typename ImageType::PointType PointType;

    // we take an arbitrary dataset as the reference, as they have all the same resolution anyway
    typename ImageFileReaderType::Pointer refReader = ImageFileReaderType::New();
    refReader->SetFileName(referenceFilename);
    refReader->Update();

    typename RepresenterType::Pointer representer = RepresenterType::New();
    representer->SetReference(refReader->GetOutput());


    const GaussianKernel<PointType> gk = GaussianKernel<PointType>(gaussianKernelSigma); // a gk with sigma 100
    // make the kernel matrix valued and scale it by a factor of 100
    const statismo::MatrixValuedKernel<PointType>& mvGk = statismo::UncorrelatedMatrixValuedKernel<PointType>(&gk, representer->GetDimensions());
    const statismo::MatrixValuedKernel<PointType>& scaledGk = statismo::ScaledKernel<PointType>(&mvGk, 100.0);


    typename ModelBuilderType::Pointer gpModelBuilder = ModelBuilderType::New();
    gpModelBuilder->SetRepresenter(representer);
    typename StatisticalModelType::Pointer model = gpModelBuilder->BuildNewZeroMeanModel(scaledGk, 100);
    itk::StatismoIO<ImageType>::SaveStatisticalModel(model, modelname);
}

int main(int argc, char* argv[]) {

    if (argc < 5) {
        std::cout << "usage " << argv[0] << " dimension referenceFilename gaussianKernelSigma modelname" << std::endl;
        exit(-1);
    }

    unsigned int dimension = atoi(argv[1]);
    const char* referenceFilename = argv[2];
    double gaussianKernelSigma = atof(argv[3]);
    const char* modelname = argv[4];

    if (dimension==2) {
        itkExample<RepresenterType2D, VectorImageType2D>(referenceFilename, gaussianKernelSigma,  modelname);
    } else if (dimension==3) {
        itkExample<RepresenterType3D, VectorImageType3D>(referenceFilename, gaussianKernelSigma, modelname);
    } else {
        assert(0);
    }

    std::cout << "Model building is completed successfully." << std::endl;
}

