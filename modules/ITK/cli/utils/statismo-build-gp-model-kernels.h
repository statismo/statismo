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
#include <boost/ptr_container/ptr_vector.hpp>
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
class BSplineKernel : public statismo::ScalarValuedKernel<TPoint> {
  public:
    typedef typename  TPoint::CoordRepType CoordRepType;
    typedef vnl_vector<CoordRepType> VectorType;


    BSplineKernel(double support) : m_support(support) {}


    // the 3rd order b-spline basis function
    double bspline3(const double& x) const {
        const double absX = std::fabs(x);
        const double absXSquared = absX * absX;
        const double absXCube = absXSquared * absX;
        const double twoMinAbsX = 2.0 - absX;
        const double twoMinAbsXCube = twoMinAbsX * twoMinAbsX * twoMinAbsX;
        const double twoByThree = 2.0 / 3.0;

        double splineValue = 0;
        if (absX >= 0 && absX < 1) {
            splineValue = twoByThree - absXSquared + 0.5 * absXCube;
        } else if (absX >= 1 && absX < 2) {
            splineValue = twoMinAbsXCube / 6.0;
        } else {
            splineValue = 0;
        }
        return splineValue;
    }

    double tensorProductSpline(const VectorType& x) const {
        double prod = 1;
        for (unsigned d = 0; d < x.size(); ++d) {
            prod *= bspline3(x[d]);
        }
        return prod;
    }


    inline double operator()(const TPoint& x, const TPoint& y) const {

        assert(x.Size() == y.Size());
        const unsigned dim = x.Size();

        if (dim < 1 || dim > 3) {
            std::ostringstream os;
            os << "Currently only dimensions 1 - 3 are supported.  ( Received" << dim << ")";
            throw statismo::StatisticalModelException(os.str().c_str());
        }

        const double supportBasisFunction = 4.0;
        const double scale = -1.0 * std::log(m_support / supportBasisFunction) / std::log(2.0);

        VectorType xScaled = x.GetVnlVector() * std::pow(2.0, scale);
        VectorType yScaled = y.GetVnlVector() * std::pow(2.0, scale);

        std::vector<int> kLower(dim);
        std::vector<int> kUpper(dim);
        for (unsigned d = 0; d < dim; ++d) {
            kLower[d] = static_cast<int>(std::ceil(std::max(xScaled[d], yScaled[d]) - 0.5 * supportBasisFunction));
            kUpper[d] = static_cast<int>(std::floor(std::min(xScaled[d], yScaled[d]) + 0.5 * supportBasisFunction));
        }


        // We need to generate the cartesian product k_1 x ... x k_d, where k_i goes through all the integers
        // within the given bounds. A non-recursive solution requires d loops. Here we just write down the cases
        // for 1 2 and 3D
        double sum = 0.0;
        double kx = kLower[0];
        while (kx <= kUpper[0]) {
            if (dim == 1) {
                VectorType k(1);
                k[0] = kx;
                sum += (tensorProductSpline(xScaled - k) * tensorProductSpline(yScaled - k));

            } else {
                double ky = kLower[1];
                while (ky <= kUpper[1]) {
                    if (dim == 2) {
                        VectorType k(2);
                        k[0] = kx;
                        k[1] = ky;
                        sum += (tensorProductSpline(xScaled - k) * tensorProductSpline(yScaled - k));
                    } else {
                        double kz = kLower[2];
                        while (kz <= kUpper[2]) {
                            VectorType k(3);
                            k[0] = kx;
                            k[1] = ky;
                            k[2] = kz;
                            sum += (tensorProductSpline(xScaled - k) * tensorProductSpline(yScaled - k));
                            kz += 1;
                        }
                    }
                    ky += 1;
                }
            }

            kx += 1;
        }

        return sum;
    }

    std::string GetKernelInfo() const {
        std::ostringstream os;
        os << "BSplineKernel(" << m_support << ")";
        return os.str();
    }

  private:
    double m_support;
};


//
// A kernel that represents deformations on different scale levels.
// Each scale level is modeled as a b-spline kernel, which leads to a "wavelet style"
// decomposition of the deformation. See
// Opfer, Roland. "Multiscale kernels." Advances in computational mathematics 25.4 (2006): 357-380.
//
template <class TPoint>
class MultiscaleKernel : public statismo::ScalarValuedKernel<TPoint> {
  public:
    typedef typename  TPoint::CoordRepType CoordRepType;
    typedef vnl_vector<CoordRepType> VectorType;


    MultiscaleKernel(double supportBaseLevel, unsigned numberOfLevels) :
        m_supportBaseLevel(supportBaseLevel),
        m_numberOfLevels(numberOfLevels) {
        double support = supportBaseLevel;
        for (unsigned i = 0; i < numberOfLevels; ++i) {
            m_kernels.push_back(new BSplineKernel<TPoint>(m_supportBaseLevel * std::pow(2, -1.0 * i)));
            m_kernelWeights.push_back(std::pow(2, -1.0 * i));
        }
    }



    inline double operator()(const TPoint& x, const TPoint& y) const {

        assert(x.Size() == y.Size());
        const unsigned dim = x.Size();
        double sum = 0;
        for (unsigned i = 0; i < m_kernels.size(); ++i) {
            sum += m_kernels[i](x, y) * m_kernelWeights[i];
        }

        return sum;
    }

    std::string GetKernelInfo() const {
        std::ostringstream os;
        os << "MultiscaleKernel(" << m_supportBaseLevel << ", " << m_numberOfLevels << ")";
        return os.str();
    }

  private:
    double m_supportBaseLevel;
    unsigned m_numberOfLevels;
    boost::ptr_vector<BSplineKernel<TPoint> >  m_kernels;
    std::vector<double> m_kernelWeights;
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

template <class TPoint>
const statismo::ScalarValuedKernel<TPoint>* createMultiscaleKernel(std::vector<std::string> kernelArgs) {
    if (kernelArgs.size() == 2) {
        try {
            double baseLevel = boost::lexical_cast<double>(kernelArgs[0]);
            unsigned numberOfLevels = boost::lexical_cast<unsigned>(kernelArgs[1]);

            if (baseLevel <= 0) {
                itkGenericExceptionMacro( << "Error: baselevel has to be > 0");
            }
            //The kernel will be deleted automatically once it's no longer being used
            return new MultiscaleKernel<TPoint>(baseLevel, numberOfLevels);
        } catch (boost::bad_lexical_cast &) {
            itkGenericExceptionMacro( << "Error: could not parse the kernel argument");
        }
    } else {
        itkGenericExceptionMacro( <<"The Multiscale Kernel takes two arguments: the support of the base level and the number of levels. You provided" <<kernelArgs.size()<< " Arguments.");
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
    addKernelToKernelMap("gaussian", createGaussianKernel);
    addKernelToKernelMap("multiscale", createMultiscaleKernel);
}