/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *
 * Copyright (c) 2011 University of Basel
 * All rights reserved.
 *
 * Statismo is licensed under the BSD licence (3 clause) license
 */



#include "LowRankGPModelBuilder.h"
#include "StatisticalModel.h"
#include "vtkPolyDataReader.h"
#include "vtkStandardMeshRepresenter.h"
#include "Kernels.h"
#include "KernelCombinators.h"

#include <iostream>
#include <memory>


using namespace statismo;


/*
 * We use a sum of gaussian kernels as our main model.
 */
class MultiscaleGaussianKernel: public MatrixValuedKernel<vtkPoint> {
public:

    MultiscaleGaussianKernel(float baseWidth, float baseScale, unsigned nLevels) : m_baseWidth(baseWidth), m_baseScale(baseScale), m_nLevels(nLevels), MatrixValuedKernel<vtkPoint>(3){
	}

    inline statismo::MatrixType operator()(const vtkPoint& x, const vtkPoint& y) const {
		VectorType r(3);
		r << x[0] - y[0], x[1] - y[1], x[2] - y[2];

        float kernelValue = 0;
        for (unsigned l = 1 ; l <= m_nLevels; ++l) {
            float widthOnLevel = m_baseWidth / l;
            float scaleOnLevel = m_baseScale / l;
            kernelValue += scaleOnLevel * std::exp(-r.dot(r) / (widthOnLevel * widthOnLevel));
        }
        return statismo::MatrixType::Identity(3,3) * kernelValue;
	}

	std::string GetKernelInfo() const {
		std::ostringstream os;
        os << "Multiscale GaussianKernel";
		return os.str();
	}

private:

    float m_baseWidth;
    float m_baseScale;
    unsigned m_nLevels;
};


vtkPolyData* loadVTKPolyData(const std::string& filename)
{
	vtkPolyDataReader* reader = vtkPolyDataReader::New();
	reader->SetFileName(filename.c_str());
	reader->Update();
	vtkPolyData* pd = vtkPolyData::New();
	pd->ShallowCopy(reader->GetOutput());
    reader->Delete();
    return pd;
}

// As an example of a tempering function, we use the sigmoid function in x direction. This implies that
// any point whose x coordinate is smaller than 0 will be unchanged, while values larger than 0 will be tempered (with value 2).
// How smooth the transition between the values is, is defined by the variable a.
struct SigmoidFunction : public TemperingFunction<vtkPoint> {
  static const double a = 0.5;
  double operator() (const vtkPoint& pt) const {return  (1.0 / ( 1 + std::exp(-pt[0] * a)) + 1.0) ; }
};




//
// Computes a multi-scale gaussian model and uses spatial tempering to make the smoothness spatially varying.
//
// For mathematical details of this procedure, refer to the paper:
// T. Gerig, K. Shahim, M. Reyes, T. Vetter, M. Luethi, Spatially-varying registration using Gaussian Processes.
//
int main(int argc, char** argv) {

    if (argc < 7) {
        std::cout << "Usage " << argv[0] << " referenceMesh baseKernelWidth baseScale numLevels numberOfComponents outputmodelName" << std::endl;
		exit(-1);
	}
    std::string refFilename(argv[1]);
    double baseKernelWidth = std::atof(argv[2]);
    double baseScale = std::atof(argv[3]);
    unsigned numLevels = std::atoi(argv[4]);
    int numberOfComponents = std::atoi(argv[5]);
    std::string outputModelFilename(argv[6]);


	// All the statismo classes have to be parameterized with the RepresenterType.

	typedef vtkStandardMeshRepresenter RepresenterType;
	typedef LowRankGPModelBuilder<vtkPolyData> ModelBuilderType;
	typedef StatisticalModel<vtkPolyData> StatisticalModelType;
    typedef MultiscaleGaussianKernel GaussianKernelType;
	typedef MatrixValuedKernel<vtkPoint> MatrixValuedKernelType;

	try {

        vtkPolyData* referenceMesh = loadVTKPolyData(refFilename);
        vtkStandardMeshRepresenter* representer = vtkStandardMeshRepresenter::Create(referenceMesh);

        MultiscaleGaussianKernel gk = MultiscaleGaussianKernel(baseKernelWidth, baseScale, numLevels);

        SigmoidFunction temperingFun;

        const MatrixValuedKernelType& temperedKernel = SpatiallyVaryingKernel<RepresenterType::DatasetType>(representer, gk, temperingFun , numberOfComponents,  numberOfComponents * 2, true);

        // We create a new model using the combined kernel. The new model will be more flexible than the original statistical model.
        ModelBuilderType* modelBuilder = ModelBuilderType::Create(representer);

        StatisticalModelType* newModel = modelBuilder->BuildNewModel(referenceMesh, temperedKernel, numberOfComponents);

        // Once we have built the model, we can save it to disk.
        newModel->Save(outputModelFilename);
		std::cout << "Successfully saved shape model as " << outputModelFilename << std::endl;

        referenceMesh->Delete();
        representer->Delete();
        modelBuilder->Delete();
        newModel->Delete();

	}
	catch (StatisticalModelException& e) {
		std::cout << "Exception occured while building the shape model" << std::endl;
		std::cout << e.what() << std::endl;
	}
}
