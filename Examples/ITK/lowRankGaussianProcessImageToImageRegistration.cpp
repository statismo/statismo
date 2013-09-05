/*
 * This file is part of the statismo library.
 *
 * Author: Christoph Jud (christoph.jud@unibas.ch)
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
 * 
 * Usage:
 * 
 * (without landmarks)
 * ./bin/itkLowRankGaussianProcessImageToImageRegistration share/data/hand_images/hand-1.vtk share/data/hand_images/hand-2.vtk /tmp/registered.vtk /tmp/deformationfield.vtk MeanSquares 70 100 100 100
 *
 * (including landmarks)
 * ./bin/itkLowRankGaussianProcessImageToImageRegistration share/data/hand_images/hand-1.vtk share/data/hand_landmarks/hand-1.fcsv share/data/hand_images/hand-2.vtk share/data/hand_landmarks/hand-2.fcsv /tmp/registered.vtk /tmp/deformationfield.vtk MeanSquares 70 100 0.1 100 100
 * 
 */


#include "itkStandardImageRepresenter.h"
#include "statismo_ITK/itkStatisticalModel.h"
#include "statismo_ITK/itkLowRankGPModelBuilder.h"
#include "statismo_ITK/itkPartiallyFixedModelBuilder.h"
#include "statismo_ITK/itkDataManager.h"
#include "statismo_ITK/itkInterpolatingStatisticalDeformationModelTransform.h"
#include "itkMeanSquaresImageToImageMetric.h"
#include "itkNormalizedCorrelationImageToImageMetric.h"
#include "itkLBFGSOptimizer.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkImageRegistrationMethod.h"
#include "itkWarpImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCommand.h"
#include "itkImage.h"

#include "itkDirectory.h"
#include <sys/types.h>
#include <errno.h>
#include <iostream>
#include <iomanip>
#include <string>


/*
 * Build a low-rank Gaussian process model using a Gaussian kernel function
 * 
 * Input:
 *  - const char* referenceFilename		-> Filename to reference image
 *  - double gaussianKernelSigma		-> Kernel parameter sigma of Gaussian kernel
 *  - double gaussianKernelScale		-> Kernel parameter scale of Gaussian kernel
 *  - unsigned numberOfBasisFunctions	-> Number of basis functions to be approximated
 *  
 * Output:
 *  - TStatisticalModelType::Pointer	-> Smartpointer on a (statismo) statistical model.
 *  
 * Comment:
 *  - The composition of different kernel functions can be even more complicated. For example
 *    a linear combination of different kernel functions is again a kernel function and thus
 *    can be handled by the LowRankGPModelBuilder.
 */
template <class TRepresenterType, class TImageType, class TStatisticalModelType>
typename TStatisticalModelType::Pointer buildLowRankGPModel(const char* referenceFilename, double gaussianKernelSigma, double gaussianKernelScale, unsigned numberOfBasisFunctions) {

	typedef itk::LowRankGPModelBuilder<TImageType> ModelBuilderType;
    typedef std::vector<std::string> StringVectorType;
	typedef itk::ImageFileReader<TImageType> ImageFileReaderType;

	std::cout << "Building low-rank Gaussian process deformation model... " << std::flush;

    // we take an arbitrary dataset as the reference, as they have all the same resolution anyway
	typename ImageFileReaderType::Pointer referenceReader = ImageFileReaderType::New();
	referenceReader->SetFileName(referenceFilename);
	referenceReader->Update();

    typename TRepresenterType::Pointer representer = TRepresenterType::New();
    representer->SetReference(referenceReader->GetOutput());

	const statismo::GaussianKernel gk = statismo::GaussianKernel(gaussianKernelSigma); // a Gaussian kernel with sigma=gaussianKernelSigma
	// make the kernel matrix valued and scale it by a factor of 100
	const statismo::MatrixValuedKernel& mvGk = statismo::UncorrelatedMatrixValuedKernel(&gk, representer->GetDimensions());
	const statismo::MatrixValuedKernel& scaledGk = statismo::ScaledKernel(&mvGk, gaussianKernelScale); // apply Gaussian scale parameter

    typename ModelBuilderType::Pointer gpModelBuilder = ModelBuilderType::New();
    gpModelBuilder->SetRepresenter(representer);
    typename TStatisticalModelType::Pointer model = gpModelBuilder->BuildNewZeroMeanModel(scaledGk, numberOfBasisFunctions); // number of basis functions
    
    std::cout << "[done]" << std::endl;
    return model;
}


/*
 * Landmark file reader function.
 * 
 * Format for n landmark points
 * x0 y0 z0\n
 * x1 y1 z1\n
 * ...
 * xi yi zi\n
 * ...
 * xn yn zn\n
 * 
 * Input:
 *  - std::string landmarkFilename		-> Filename of landmark file.
 *  
 * Output:
 *  - std::vector<TPointType>			-> A standard vector of points.
 *  
 * Comment:
 * 	- Template parameter TPointType is a statismo StatisticalModelType::PointType e.g. itk::Point<float, 3>
 */
template<class TPointType, unsigned int VImageDimension>
typename std::vector<TPointType> ReadLandmarkFile(std::string landmarkFilename){
	std::vector<TPointType> pointVector;
	
	const unsigned int x=0, y=1, z=2;
	double p[3];
	
	std::ifstream infile;
	infile.open( landmarkFilename.c_str() ); // ASCII mode
	std::string line;
	while(std::getline(infile, line)){	
		std::stringstream line_stream(line);
		if(!(line_stream >> p[x] && line_stream >> p[y] && line_stream >> p[z])){
			std::stringstream error_message;
			error_message << "ReadLandmarkFile: landmark file is corrupt (filename " << landmarkFilename << ")." << std::endl;
			throw error_message.str();
		}

		TPointType point;
		for(unsigned i=0; i<VImageDimension; i++){
			point[i] = p[i]; 
		}
		pointVector.push_back(point);
	}
	infile.close();

	return pointVector;
}


/*
 * Build a partially fixed model.
 * 
 * The original model is constrainted on the landmarks displacements.
 * 
 * Input:
 *  - TStatisticalModelType::Pointer model		-> Original model.
 *  - std::string referenceLandmarkFilename		-> Filename of the reference landmark file.
 *  - std::string targetLandmarkFilename		-> Filename of the target landmark file.
 *  - double landmarkUncertainty				-> Landmark uncertainty.
 * 
 * Output:
 *  - TStatisticalModelType::Pointer			-> Smartpointer on the constrainted model.
 *  
 * Comment:
 *  - The noise on the landmark measurements is modeled as mean free isotropic Gaussian where
 *    variance parameter is set by  landmarkUncertainty.
 */
template <class TImageType, class TStatisticalModelType, unsigned int VImageDimension>
typename TStatisticalModelType::Pointer constrainModel(typename TStatisticalModelType::Pointer model, std::string referenceLandmarkFilename, std::string targetLandmarkFilename, double landmarkUncertainty) {
	typedef typename itk::PartiallyFixedModelBuilder<TImageType> PartiallyFixedModelBuilderType;
	
	typename TStatisticalModelType::PointValueListType constraints;
	
	std::vector<typename TStatisticalModelType::PointType> referencePointVector = ReadLandmarkFile<typename TStatisticalModelType::PointType, VImageDimension>(referenceLandmarkFilename);
	std::vector<typename TStatisticalModelType::PointType> targetPointVector = ReadLandmarkFile<typename TStatisticalModelType::PointType, VImageDimension>(targetLandmarkFilename);
	assert(referencePointVector.size()!=targetPointVector.size());
	
	for(unsigned i=0; i<referencePointVector.size(); i++){
		typename TImageType::PixelType displacement;
		for(unsigned d=0; d<VImageDimension; d++){
			displacement[d] = referencePointVector[i][d] - targetPointVector[i][d];
		}
		
		typename TStatisticalModelType::PointValuePairType pointValue(targetPointVector[i], displacement);
		constraints.push_back(pointValue);
	}

	typename PartiallyFixedModelBuilderType::Pointer pfmb = PartiallyFixedModelBuilderType::New();
	typename TStatisticalModelType::Pointer constraintModel = pfmb->BuildNewModelFromModel(model.GetPointer(), constraints, landmarkUncertainty);
	
	return constraintModel;
}

/*
 * Iteration observer of the registration
 */
class IterationStatusObserver : public itk::Command{
public:
	typedef  IterationStatusObserver   Self;
	typedef  itk::Command             Superclass;
	typedef  itk::SmartPointer<Self>  Pointer;

	itkNewMacro( Self );

	typedef itk::LBFGSOptimizer    OptimizerType;
	typedef const OptimizerType                     *OptimizerPointer;

	void Execute(itk::Object *caller, const itk::EventObject & event){
		Execute( (const itk::Object *)caller, event);
	}

	void Execute(const itk::Object * object, const itk::EventObject & event){
		OptimizerPointer optimizer = dynamic_cast< OptimizerPointer >( object );
		if( ! itk::IterationEvent().CheckEvent( &event ) ){
			return;
		}

		std::cout << "Iteration: " << ++m_iter_no ;
		std::cout << "; Value: " << optimizer->GetCachedValue();
		std::cout << "; Current Parameters: " << optimizer->GetCachedCurrentPosition() << std::endl;
	}

protected:
	IterationStatusObserver(): m_iter_no(0) {};
	virtual ~IterationStatusObserver(){};

private:
	int m_iter_no;
};


/*
 * Image to image registration method using a statismo statistical model.
 * 
 * The standard parametric registration framework of ITK is used for registration, where
 * the transform is a InterpolatingStatisticalDeformationModelTransform.
 * 
 * Input:
 *  - std::string referenceFilename			-> Filename of reference image.
 *  - std::string targetFilename			-> Filename of target image.
 *  - TStatisticalModelType::Pointer model	-> Smartpointer to the statistical model.
 *  - std::string outputDfFilename			-> Filename where the resulting deformation field is written ("" = disabled).
 *  - unsigned numberOfIterations			-> Maximum number of iteration performed by the optimizer.
 *  
 * Output:
 *  - TImageType::Pointer		-> The registered image (reference image warped by the deformation field).
 */
template<class TImageType, class TVectorImageType, class TStatisticalModelType, class TMetricType, unsigned int VImageDimension>
typename TImageType::Pointer modelBasedImageToImageRegistration(std::string referenceFilename, 
																					std::string targetFilename, 
																					typename TStatisticalModelType::Pointer model,
																					std::string outputDfFilename,
																					unsigned numberOfIterations){

	typedef itk::ImageFileReader<TImageType> ImageReaderType;
	typedef itk::InterpolatingStatisticalDeformationModelTransform<TVectorImageType, double, VImageDimension> TransformType;
	typedef itk::LBFGSOptimizer OptimizerType;
	typedef itk::ImageRegistrationMethod<TImageType, TImageType> RegistrationFilterType;
	typedef itk::WarpImageFilter< TImageType, TImageType, TVectorImageType > WarperType;
	typedef itk::LinearInterpolateImageFunction< TImageType, double > InterpolatorType;
	
	typename ImageReaderType::Pointer referenceReader = ImageReaderType::New();
	referenceReader->SetFileName(referenceFilename.c_str());
	referenceReader->Update();
	typename TImageType::Pointer referenceImage = referenceReader->GetOutput();
	referenceImage->Update();

	typename ImageReaderType::Pointer targetReader = ImageReaderType::New();
	targetReader->SetFileName(targetFilename.c_str());
	targetReader->Update();
	typename TImageType::Pointer targetImage = targetReader->GetOutput();
	targetImage->Update();

	// do the fitting
	typename TransformType::Pointer transform = TransformType::New();
	transform->SetStatisticalModel(model);
	transform->SetIdentity();

	// Setting up the fitting
	OptimizerType::Pointer optimizer = OptimizerType::New();
	optimizer->MinimizeOn();
	optimizer->SetMaximumNumberOfFunctionEvaluations(numberOfIterations);

	typedef  IterationStatusObserver ObserverType;
	ObserverType::Pointer observer = ObserverType::New();
	optimizer->AddObserver( itk::IterationEvent(), observer );

	typename TMetricType::Pointer metric = TMetricType::New();
	typename InterpolatorType::Pointer interpolator = InterpolatorType::New();

	typename RegistrationFilterType::Pointer registration = RegistrationFilterType::New();
	registration->SetInitialTransformParameters(transform->GetParameters());
	registration->SetMetric(metric);
	registration->SetOptimizer(   optimizer   );
	registration->SetTransform(   transform );
	registration->SetInterpolator( interpolator );
	registration->SetFixedImage( targetImage );
	registration->SetFixedImageRegion(targetImage->GetBufferedRegion() );
	registration->SetMovingImage( referenceImage );

	try {
		std::cout << "Performing registration... " << std::flush;
		registration->Update();
		std::cout << "[done]" << std::endl;

	} catch ( itk::ExceptionObject& o ) {
		std::cout << "caught exception " << o << std::endl;
	}

	typename TVectorImageType::Pointer df = model->DrawSample(transform->GetCoefficients());

	// write deformation field
	if(outputDfFilename.size()>0){
		typename itk::ImageFileWriter<TVectorImageType>::Pointer df_writer = itk::ImageFileWriter<TVectorImageType>::New();
		df_writer->SetFileName(outputDfFilename);
		df_writer->SetInput(df);
		df_writer->Update();
	}

	
	// warp reference
	std::cout << "Warping reference... " << std::flush;
	typename WarperType::Pointer warper = WarperType::New();
	warper->SetInput(referenceImage  );
	warper->SetInterpolator( interpolator );
	warper->SetOutputSpacing( targetImage->GetSpacing() );
	warper->SetOutputOrigin( targetImage->GetOrigin() );
	warper->SetOutputDirection( targetImage->GetDirection() );
	warper->SetDisplacementField( df );
	warper->Update();
	std::cout << "[done]" << std::endl;

	return warper->GetOutput();
}

/*
 * Performes the image to image registration:
 *  1.	low-rank approximation of the Gaussian process prior
 *  2.	constraining the low-rank model on landmark displacements (if landmarks are defined)
 *  3.	performe the model based image to image registration
 *  4.	write the result on the hdd
 *  
 * Input:
 *  - std::string referenceFilename			-> Filename of reference image.
 *  - std::string referenceLandmarkFilename	-> Filename of reference landmark file ("" = disabled).
 *  - std::string targetFilename			-> Filename of target image.
 *  - std::string targetLandmarkFilename	-> Filename of target landmark file ("" = disabled).
 *  - double gaussianKernelSigma			-> Kernel parameter of Gaussian kernel.
 *  - double gaussianKernelScale			-> Kernel parameter of Gaussian kernel.
 *  - double landmarkUncertainty			-> Noise parameter to model uncertainty on the landmarks.
 *  - std::string similarityMetric			-> Similarity metric for the performance measure in the optimization.
 *  - std::string outputFilename			-> Filename of the resulting registration to be written to ("" = disabled).
 *  - std::string outputDfFilename			-> Filename of the resulting deformation field to be written to.
 *  - unsigned numberOfBasisFunctions		-> Number of basis function used in the low-rank approximation.
 *  - unsigned numberOfIterations			-> Maximum number of iterations to perform in the optimization.
 */
template<class TPixelType, unsigned int VImageDimension>
void runImageToImageRegistration(std::string referenceFilename, 
									std::string referenceLandmarkFilename, 
									std::string targetFilename, 
									std::string targetLandmarkFilename, 
									double gaussianKernelSigma, 
									double gaussianKernelScale,
									double landmarkUncertainty,
									std::string similarityMetric, 
									std::string outputFilename, 
									std::string outputDfFilename,
									unsigned numberOfBasisFunctions,
									unsigned numberOfIterations){
	
	
	typedef itk::Image<TPixelType, VImageDimension> ImageType;
	typedef itk::Image< itk::Vector<float, VImageDimension> ,VImageDimension > VectorImageType;
	typedef itk::StandardImageRepresenter<typename VectorImageType::PixelType, VImageDimension> RepresenterType;
	typedef itk::StatisticalModel<VectorImageType> StatisticalModelType;
	typedef itk::MeanSquaresImageToImageMetric<ImageType, ImageType> MeanSquaresMetricType;
	typedef itk::NormalizedCorrelationImageToImageMetric<ImageType, ImageType> NormalizedCorrelationMetricType;
	
	// build deformation model
	typename StatisticalModelType::Pointer model = buildLowRankGPModel<RepresenterType, VectorImageType, StatisticalModelType>(referenceFilename.c_str(), gaussianKernelSigma, gaussianKernelScale, numberOfBasisFunctions);
	if(referenceLandmarkFilename.size()>0 && targetLandmarkFilename.size()>0){
		model = constrainModel< VectorImageType, StatisticalModelType, VImageDimension>(model, referenceLandmarkFilename, targetLandmarkFilename, landmarkUncertainty);
	}

	// image to image registration with this model
	typename ImageType::Pointer registeredImage;
	if(similarityMetric=="MeanSquares"){
		registeredImage =  modelBasedImageToImageRegistration<ImageType, VectorImageType, StatisticalModelType, MeanSquaresMetricType, VImageDimension>(referenceFilename, targetFilename, model, outputDfFilename, numberOfIterations);
	}
	if(similarityMetric=="NormalizedCorrelation"){
		registeredImage =  modelBasedImageToImageRegistration<ImageType, VectorImageType, StatisticalModelType, NormalizedCorrelationMetricType, VImageDimension>(referenceFilename, targetFilename, model, outputDfFilename, numberOfIterations);
	}

	// write registered image
	if(outputFilename.size()>0){
		typename itk::ImageFileWriter<ImageType>::Pointer writer = itk::ImageFileWriter<ImageType>::New();
		writer->SetFileName(outputFilename);
		writer->SetInput(registeredImage);
		writer->Update();
	}
}


/*
 * Main routine:
 *  1.	parsing parameters (replace this with our favorite options parser)
 *  2.	run registration
 */
int main(int argc, char* argv[]) {

	std::string referenceFilename("");
	std::string targetFilename("");
	std::string referenceLandmarkFilename("");
	std::string targetLandmarkFilename("");
	std::string similarityMetric("");
	std::string outputFilename("");
	std::string outputDfFilename("");
	
	double gaussianKernelSigma = 70;
	double gaussianKernelScale = 100;
	double landmarkUncertainty = 0.1;
	
	unsigned numberOfBasisFunctions = 100;
	unsigned numberOfIterations = 100;
	
	
	// parse command line parameters
	if (argc != 10 && argc != 13) {
		std::cout << "***********************************************************" << std::endl;
		std::cout << "usage\t" << argv[0] << std::endl;
									std::cout << "referenceFilename:\t\t Filename of reference image." << std::endl;
									std::cout << "targetFilename:\t\t\t Filename of target image." << std::endl;
									std::cout << "outputFilename:\t\t\t Filename of the resulting registered image." << std::endl;
									std::cout << "outputDfFilename:\t\t Filename of the resulting deformation field." << std::endl;
									std::cout << "similarityMetric:\t\t Similarity metric: MeanSquares and NormalizedCorrelation are supported." << std::endl;
									std::cout << "gaussianKernelSigma:\t\t Sigma of the Gaussian kernel. (e.g. 70)" << std::endl;
									std::cout << "gaussianKernelScale:\t\t Scale of the Gaussian kernel. (e.g. 100)" << std::endl;
									std::cout << "numBasisFunctions:\t\t Number of basis functions to approximate. (e.g. 100)" << std::endl;
									std::cout << "numIterations:\t\t\t Number of iterations to perform in the optimization. (e.g. 100)" << std::endl << std::endl;

		std::cout << "or (including landmarks)\t" << argv[0] << std::endl;
									std::cout << "referenceFilename:\t\t Filename of reference image." << std::endl;
									std::cout << "referenceLandmarkFilename:\t Filename of reference landmark file." << std::endl;
									std::cout << "targetFilename:\t\t\t Filename of target image." << std::endl;
									std::cout << "targetLandmarkFilename:\t\t Filename of target landmark file." << std::endl;
									std::cout << "outputFilename:\t\t\t Filename of the resulting registered image." << std::endl;
									std::cout << "outputDfFilename:\t\t Filename of the resulting deformation field." << std::endl;
									std::cout << "similarityMetric:\t\t Similarity metric: MeanSquares and NormalizedCorrelation are supported." << std::endl;
									std::cout << "gaussianKernelSigma:\t\t Sigma of the Gaussian kernel. (e.g. 70)" << std::endl;
									std::cout << "gaussianKernelScale:\t\t Scale of the Gaussian kernel. (e.g. 100)" << std::endl;
									std::cout << "landmarkUncertainty:\t\t Uncertainty of the landmarks. (e.g. 0.1)" << std::endl;
									std::cout << "numBasisFunctions:\t\t Number of basis functions to approximate. (e.g. 100)" << std::endl;
									std::cout << "numIterations:\t\t\t Number of iterations to perform in the optimization. (e.g. 100)" << std::endl;							
		exit(-1);
	}

	if(argc == 10){
		referenceFilename = argv[1];
		targetFilename = argv[2];
		outputFilename = argv[3];
		outputDfFilename = argv[4];
		similarityMetric = argv[5];
		
		std::stringstream ss;
		ss << argv[6];
		ss >> gaussianKernelSigma;
		ss.str("");
		ss.clear();
		ss << argv[7];
		ss >> gaussianKernelScale;
		ss.str("");
		ss.clear();
		ss << argv[8];
		ss >> numberOfBasisFunctions;
		ss.str("");
		ss.clear();
		ss << argv[9];
		ss >> numberOfIterations;
	}
	if(argc == 13){
		referenceFilename = argv[1];
		referenceLandmarkFilename = argv[2];
		targetFilename = argv[3];
		targetLandmarkFilename = argv[4];
		outputFilename = argv[5];
		outputDfFilename = argv[6];
		similarityMetric = argv[7];
		
		std::stringstream ss;
		ss << argv[8];
		ss >> gaussianKernelSigma;
		ss.str("");
		ss.clear();
		ss << argv[9];
		ss >> gaussianKernelScale;
		ss.str("");
		ss.clear();
		ss << argv[10];
		ss >> landmarkUncertainty;
		ss.str("");
		ss.clear();
		ss << argv[11];
		ss >> numberOfBasisFunctions;
		ss.str("");
		ss.clear();
		ss << argv[12];
		ss >> numberOfIterations;
	}
	
	
	
	// derive number of space dimensions
	typedef itk::ImageIOBase::IOComponentType ScalarPixelType;
	itk::ImageIOBase::Pointer imageIO = itk::ImageIOFactory::CreateImageIO(referenceFilename.c_str(), itk::ImageIOFactory::ReadMode);

	imageIO->SetFileName(referenceFilename);
	imageIO->ReadImageInformation();
	const unsigned numDimensions =  imageIO->GetNumberOfDimensions();

	
	// print out parameters
	std::cout << "************************************************" << std::endl;
	std::cout << "Low-rank Gaussian process image registration:" << std::endl;
	std::cout << " - space dimensions\t\t" << numDimensions << std::endl;
	std::cout << " - reference\t\t\t" << referenceFilename << std::endl;
	if(referenceLandmarkFilename.size()>0) std::cout << " - reference landmarks\t\t" << referenceLandmarkFilename << std::endl;
	std::cout << " - target\t\t\t" << targetFilename << std::endl;
	if(targetLandmarkFilename.size()>0) std::cout << " - target landmarks\t\t" << targetLandmarkFilename << std::endl;
	std::cout << " - output\t\t\t" << outputFilename << std::endl;
	std::cout << " - output deformation field\t" << outputDfFilename << std::endl << std::endl;
	std::cout << " - similarity metric\t\t" << similarityMetric << std::endl;
	std::cout << " - Gaussian sigma\t\t" << gaussianKernelSigma << std::endl;
	std::cout << " - Gaussian scale\t\t" << gaussianKernelScale << std::endl;
	if(targetLandmarkFilename.size()>0 && referenceLandmarkFilename.size()>0) std::cout << " - Landmark uncertainty\t\t" << landmarkUncertainty << std::endl;
	std::cout << " - #basis functions\t\t" << numberOfBasisFunctions << std::endl;
	std::cout << " - #iterations\t\t\t" << numberOfIterations << std::endl << std::endl;
	
	
	if(!(similarityMetric=="NormalizedCorrelation" || similarityMetric=="MeanSquares")){
		std::cout << "Error: only MeanSquares or NormalizedCorrelation supported as metric." << std::endl;
		return -1;
	}
	
	if(landmarkUncertainty == 0){
		std::cout << "Warning: landmark uncertainty sould be greater than zero." << std::endl;
	}
	if(landmarkUncertainty < 0){
		std::cout << "Error: landmark uncertainty has to be positive." << std::endl;
		return -1;
	}
	
	if (numDimensions==2){ // run the image to image registration in 2D
		runImageToImageRegistration<float, 2>(referenceFilename, 
												referenceLandmarkFilename, 
												targetFilename, 
												targetLandmarkFilename, 
												gaussianKernelSigma, 
												gaussianKernelScale, 
												landmarkUncertainty, 
												similarityMetric, 
												outputFilename, 
												outputDfFilename,
												numberOfBasisFunctions,
												numberOfIterations);
	}
	else if (numDimensions==3){ // run the image to image registration in 2D
		runImageToImageRegistration<float, 3>(referenceFilename, 
												referenceLandmarkFilename, 
												targetFilename, 
												targetLandmarkFilename, 
												gaussianKernelSigma, 
												gaussianKernelScale, 
												landmarkUncertainty, 
												similarityMetric, 
												outputFilename, 
												outputDfFilename,
												numberOfBasisFunctions,
												numberOfIterations);
	}
	else{
	  assert(0);
	}

	std::cout << "Low-rank Gaussian process image to image registration has been successfully finished." << std::endl;
}

