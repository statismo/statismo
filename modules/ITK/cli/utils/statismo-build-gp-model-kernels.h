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

#include <map>
#include <string>

#include <boost/algorithm/string.hpp>

#include <itkImage.h>
#include <itkMesh.h>

#include <KernelCombinators.h>
#include <Kernels.h>

/*
You can add your kernels here:
	1. Either #include your kernel or put the class in here
	2. Write a function in this file that creates your kernel (create it on the heap and NOT on the stack; it will be deleted, don't worry)
	   Your function will receive a vector containing the kernel arguments (vector<string>). You have to parse them or thrown an itk::ExceptionObject if something goes wrong.
	3. Add your kernel to the kernel map (do this in the function createKernelMap(); your kernel name has to be lowercase)
	4. Document your kernel in the statismo-build-gp-model.md file. (Examples for kernels with multiple parameters are in there, but currently commented out)
*/


const unsigned Dimensionality3D = 3;
typedef itk::Mesh<float, Dimensionality3D> DataTypeShape;
typedef itk::Vector<float, Dimensionality3D> VectorPixel3DType;
typedef itk::Image<VectorPixel3DType, Dimensionality3D> DataType3DDeformation;
const unsigned Dimensionality2D = 2;
typedef itk::Vector<float, Dimensionality2D> VectorPixel2DType;
typedef itk::Image<VectorPixel2DType, Dimensionality2D> DataType2DDeformation;

struct KernelContainer {
    const statismo::ScalarValuedKernel<DataTypeShape::PointType>* (*createKernelShape)(std::vector<std::string> kernelArgs);
    const statismo::ScalarValuedKernel<DataType3DDeformation::PointType>* (*createKernel3DDeformation)(std::vector<std::string> kernelArgs);
    const statismo::ScalarValuedKernel<DataType2DDeformation::PointType>* (*createKernel2DDeformation)(std::vector<std::string> kernelArgs);
};
typedef std::map<std::string, KernelContainer> KernelMapType;
KernelMapType kernelMap;


template <class TPoint>
class GaussianKernel : public statismo::ScalarValuedKernel<TPoint> {
  public:
    typedef typename  TPoint::CoordRepType CoordRepType;
    typedef vnl_vector<CoordRepType> VectorType;


    GaussianKernel(double sigma) : m_sigma(sigma), m_sigma2(-1.0 / (sigma * sigma)) {

    }

    inline double operator()(const TPoint& x, const TPoint& y) const {
        VectorType xv = x.GetVnlVector();
        VectorType yv = y.GetVnlVector();

        VectorType r = yv - xv;

        return exp((double)dot_product(r, r) * m_sigma2);
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


template <class TPoint>
const statismo::ScalarValuedKernel<TPoint>* createGaussianKernel(std::vector<std::string> kernelArgs) {
    if (kernelArgs.size() == 1) {
        try {
            double sigma = boost::lexical_cast<double>(kernelArgs[0]);
            if (sigma <= 0) {
                itkGenericExceptionMacro( << "Error: sigma has to be > 0");
            }
            //The kernel will be deleted automatically once it's no longer being used
            return new GaussianKernel<TPoint>(sigma);
        } catch (boost::bad_lexical_cast &) {
            itkGenericExceptionMacro( << "Error: could not parse the kernel argument (expected a floating point number)");
        }
    } else {
        itkGenericExceptionMacro( <<"The Gaussian Kernel takes one and only one argument: sigma. You provided " <<kernelArgs.size()<< " Arguments.");
    }
}

#define addKernelToKernelMap(kernelName, functionName){ \
    KernelContainer kernel; \
    kernel.createKernel3DDeformation = &functionName < DataType3DDeformation::PointType >; \
	kernel.createKernel2DDeformation = &functionName < DataType2DDeformation::PointType >; \
    kernel.createKernelShape = &functionName < DataTypeShape::PointType >; \
    kernelMap.insert(std::make_pair(boost::algorithm::to_lower_copy(std::string(kernelName)), kernel)); \
}

void createKernelMap() {
    addKernelToKernelMap("gaussian", createGaussianKernel)
}