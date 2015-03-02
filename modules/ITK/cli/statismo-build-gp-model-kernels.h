#include <string>
#include <vector>
#include <map>

#include <itkImage.h>
#include <itkMesh.h>

#include <boost/algorithm/string.hpp>

#include "Kernels.h"
#include "KernelCombinators.h"

using namespace std;

/*
You can add your kernels here:
	1. Either #include your kernel or put the class in here
	2. Write a function in this file that creates your kernel (create it on the heap and NOT on the stack; it will be deleted, don't worry)
	   Your function will receive a vector containing the kernel arguments (vector<string>). You have to parse them or thrown an itk::ExceptionObject if something goes wrong.
	3. Add your kernel to the kernel map (do this in the function createKernelMap(); your kernel name has to be lowercase)
	4. Document your kernel in the statismo-build-gp-model.md file. (Examples for kernels with multiple parameters are in there, but currently commented out)
*/


const unsigned Dimensions = 3;
typedef itk::Mesh<float, Dimensions> DataTypeShape;
typedef itk::Image< itk::Vector<float, Dimensions>, Dimensions > DataTypeDeformation;
struct KernelContainer {
    const statismo::ScalarValuedKernel<DataTypeShape::PointType>* (*createKernelShape)(vector<string> kernelArgs);
    const statismo::ScalarValuedKernel<DataTypeDeformation::PointType>* (*createKernelDeformation)(vector<string> kernelArgs);
};
typedef map<string, KernelContainer> KernelMapType;
KernelMapType kernelMap;


template <class TPoint>
class GaussianKernel : public statismo::ScalarValuedKernel<TPoint> {
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


template <class TPoint>
const statismo::ScalarValuedKernel<TPoint>* createGaussianKernel(vector<string> kernelArgs) {
    if (kernelArgs.size() == 1) {
        try {
            double sigma = boost::lexical_cast<double>(kernelArgs[0]);
            if (sigma <= 0) {
                itk::ExceptionObject e(__FILE__, __LINE__, "Error: sigma has to be > 0", ITK_LOCATION);
                throw e;
            }
            //The kernel will be deleted automatically once it's no longer being used
            return new GaussianKernel<TPoint>(sigma);
        } catch (boost::bad_lexical_cast &) {
            itk::ExceptionObject e(__FILE__, __LINE__, "Error: could not parse the kernel argument (expected a floating point number)", ITK_LOCATION);
            throw e;
        }
    } else {
        itk::ExceptionObject e(__FILE__, __LINE__, "The Gaussian Kernel takes one and only one argument: sigma. You provided " + boost::lexical_cast<string>(kernelArgs.size()) + " Arguments.", ITK_LOCATION);
        throw e;
    }
}

#define addKernelToKernelMap(kernelName, functionName){KernelContainer kernel;kernel.createKernelDeformation = &functionName < DataTypeDeformation::PointType >;kernel.createKernelShape = &functionName < DataTypeShape::PointType >;kernelMap.insert(make_pair(boost::algorithm::to_lower_copy(string(kernelName)), kernel)); }
void createKernelMap() {
    addKernelToKernelMap("gauss", createGaussianKernel)
}