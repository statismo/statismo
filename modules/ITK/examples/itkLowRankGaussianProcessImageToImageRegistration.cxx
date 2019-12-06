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
 *
 *  This example is to illustrate the full functionality of the Gaussian Process registration using statismo.
 *  It can perform a registration of 2D and 3D images. Optionally, landmarks can be provided, which should be matched in
 * the final registration result.
 *
 * Usage:
 *
 * (without landmarks)
 * ./bin/itkLowRankGaussianProcessImageToImageRegistration share/data/hand_images/hand-1.vtk
 * share/data/hand_images/hand-2.vtk /tmp/registered.vtk /tmp/deformationfield.vtk MeanSquares 70 100 100 100
 *
 * (including landmarks)
 * ./bin/itkLowRankGaussianProcessImageToImageRegistration share/data/hand_images/hand-1.vtk
 * share/data/hand_landmarks/hand-1.fcsv share/data/hand_images/hand-2.vtk share/data/hand_landmarks/hand-2.fcsv
 * /tmp/registered.vtk /tmp/deformationfield.vtk MeanSquares 70 100 0.1 100 100
 *
 */

#include <sys/types.h>
#include <errno.h>
#include <iostream>
#include <iomanip>
#include <string>

#include <itkCommand.h>
#include <itkDirectory.h>
#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkImageRegistrationMethod.h>
#include <itkLBFGSOptimizer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkMeanSquaresImageToImageMetric.h>
#include <itkNormalizedCorrelationImageToImageMetric.h>
#include <itkWarpImageFilter.h>

#include "statismo/core/Kernels.h"
#include "statismo/core/KernelCombinators.h"

#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkInterpolatingStatisticalDeformationModelTransform.h"
#include "statismo/ITK/itkLowRankGPModelBuilder.h"
#include "statismo/ITK/itkPosteriorModelBuilder.h"
#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkStatisticalModel.h"

/**
 * A scalar valued gaussian kernel.
 */
template <class TPoint>
class GaussianKernel : public statismo::ScalarValuedKernel<TPoint>
{
public:
  typedef typename TPoint::CoordRepType CoordRepType;
  typedef vnl_vector<CoordRepType>      VectorType;

  GaussianKernel(double sigma)
    : m_sigma(sigma)
    , m_sigma2(sigma * sigma)
  {}

  inline double
  operator()(const TPoint & x, const TPoint & y) const
  {
    VectorType xv = x.GetVnlVector();
    VectorType yv = y.GetVnlVector();

    VectorType r = yv - xv;
    return exp(-dot_product(r, r) / m_sigma2);
  }

  std::string
  GetKernelInfo() const
  {
    std::ostringstream os;
    os << "GaussianKernel(" << m_sigma << ")";
    return os.str();
  }

private:
  double m_sigma;
  double m_sigma2;
};


/*
 * Build a low-rank Gaussian process model using a Gaussian kernel function
 *
 * Input:
 *  - Filename to reference image
 *  - Kernel parameter sigma of Gaussian kernel
 *  - Kernel parameter scale of Gaussian kernel
 *  - Number of basis functions to be approximated
 *
 * Output:
 *  - Smartpointer on a (statismo) statistical model.
 *
 *  - The composition of different kernel functions can be even more complicated. For example
 *    a linear combination of different kernel functions is again a kernel function and thus
 *    can be handled by the LowRankGPModelBuilder.
 */
template <class TRepresenter, class TImage, class TStatisticalModel>
typename TStatisticalModel::Pointer
buildLowRankGPModel(const char * referenceFilename,
                    double       gaussianKernelSigma,
                    double       gaussianKernelScale,
                    unsigned     numberOfBasisFunctions)
{

  typedef itk::LowRankGPModelBuilder<TImage> ModelBuilderType;
  typedef itk::ImageFileReader<TImage>       ImageFileReaderType;
  typedef typename TImage::PointType         PointType;


  typename TRepresenter::Pointer representer = TRepresenter::New();

  std::cout << "Building low-rank Gaussian process deformation model... " << std::flush;


  // we take an arbitrary dataset as the reference, as they have all the same resolution anyway
  typename ImageFileReaderType::Pointer referenceReader = ImageFileReaderType::New();
  referenceReader->SetFileName(referenceFilename);
  referenceReader->Update();

  representer->SetReference(referenceReader->GetOutput());

  const GaussianKernel<PointType> gk =
    GaussianKernel<PointType>(gaussianKernelSigma); // a Gaussian kernel with sigma=gaussianKernelSigma
  // make the kernel matrix valued and scale it by a factor of 100
  const statismo::MatrixValuedKernel<PointType> & mvGk =
    statismo::UncorrelatedMatrixValuedKernel<PointType>(&gk, representer->GetDimensions());
  const statismo::MatrixValuedKernel<PointType> & scaledGk =
    statismo::ScaledKernel<PointType>(&mvGk, gaussianKernelScale); // apply Gaussian scale parameter

  typename ModelBuilderType::Pointer gpModelBuilder = ModelBuilderType::New();
  gpModelBuilder->SetRepresenter(representer);
  typename TStatisticalModel::Pointer model =
    gpModelBuilder->BuildNewZeroMeanModel(scaledGk, numberOfBasisFunctions); // number of basis functions

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
template <class TPointType, unsigned int VImageDimension>
typename std::vector<TPointType>
ReadLandmarkFile(std::string landmarkFilename)
{
  std::vector<TPointType> pointVector;

  const unsigned int x = 0, y = 1, z = 2;
  double             p[3];

  std::ifstream infile;
  infile.open(landmarkFilename.c_str()); // ASCII mode
  std::string line;
  while (std::getline(infile, line))
  {
    std::stringstream line_stream(line);
    if (!(line_stream >> p[x] && line_stream >> p[y] && line_stream >> p[z]))
    {
      std::stringstream error_message;
      error_message << "ReadLandmarkFile: landmark file is corrupt (filename " << landmarkFilename << ")." << std::endl;
      throw error_message.str();
    }

    TPointType point;
    for (unsigned i = 0; i < VImageDimension; i++)
    {
      point[i] = p[i];
    }
    pointVector.push_back(point);
  }

  return pointVector;
}


/*
 * Build a partially fixed model.
 *
 * The original model is constrainted on the landmarks displacements.
 *
 * Input:
 *  - Original model.
 *  - Filename of the reference landmark file.
 *  - Filename of the target landmark file.
 *  - Landmark uncertainty.
 *
 * Output:
 *  - Smartpointer on the constrainted model.
 *
 * Comment:
 *  - The noise on the landmark measurements is modeled as mean free isotropic Gaussian where
 *    variance parameter is set by  landmarkUncertainty.
 */

template <class TImage, class TStatisticalModel, unsigned int VImageDimension>
typename TStatisticalModel::Pointer
constrainModel(typename TStatisticalModel::Pointer model,
               std::string                         referenceLandmarkFilename,
               std::string                         targetLandmarkFilename,
               double                              landmarkUncertainty)
{
  typedef typename itk::PosteriorModelBuilder<TImage> PosteriorModelBuilderType;

  typename TStatisticalModel::PointValueListType constraints;

  std::vector<typename TStatisticalModel::PointType> referencePointVector =
    ReadLandmarkFile<typename TStatisticalModel::PointType, VImageDimension>(referenceLandmarkFilename);
  std::vector<typename TStatisticalModel::PointType> targetPointVector =
    ReadLandmarkFile<typename TStatisticalModel::PointType, VImageDimension>(targetLandmarkFilename);
  assert(referencePointVector.size() != targetPointVector.size());

  for (unsigned i = 0; i < referencePointVector.size(); i++)
  {
    typename TImage::PixelType displacement;
    for (unsigned d = 0; d < VImageDimension; d++)
    {
      displacement[d] = referencePointVector[i][d] - targetPointVector[i][d];
    }

    typename TStatisticalModel::PointValuePairType pointValue(targetPointVector[i], displacement);
    constraints.push_back(pointValue);
  }

  typename PosteriorModelBuilderType::Pointer pfmb = PosteriorModelBuilderType::New();
  typename TStatisticalModel::Pointer         constraintModel =
    pfmb->BuildNewModelFromModel(model.GetPointer(), constraints, landmarkUncertainty);

  return constraintModel;
}

/*
 * Iteration observer of the registration
 */
class IterationStatusObserver : public itk::Command
{
public:
  typedef IterationStatusObserver Self;
  typedef itk::Command            Superclass;
  typedef itk::SmartPointer<Self> Pointer;

  itkNewMacro(Self);

  typedef itk::LBFGSOptimizer   OptimizerType;
  typedef const OptimizerType * OptimizerPointer;

  void
  Execute(itk::Object * caller, const itk::EventObject & event)
  {
    Execute((const itk::Object *)caller, event);
  }

  void
  Execute(const itk::Object * object, const itk::EventObject & event)
  {
    OptimizerPointer optimizer = dynamic_cast<OptimizerPointer>(object);
    if (!itk::IterationEvent().CheckEvent(&event))
    {
      return;
    }

    std::cout << "Iteration: " << ++m_iter_no;
    std::cout << "; Value: " << optimizer->GetCachedValue();
    std::cout << "; Current Parameters: " << optimizer->GetCachedCurrentPosition() << std::endl;
  }

protected:
  IterationStatusObserver()
    : m_iter_no(0){};
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
 *  - Filename of reference image.
 *  - Filename of target image.
 *  - Smartpointer to the statistical model.
 *  - Filename where the resulting deformation field is written ("" = disabled).
 *  - Maximum number of iteration performed by the optimizer.
 *
 * Output:
 *  - TImage::Pointer		-> The registered image (reference image warped by the deformation field).
 */

template <class TImage, class TVectorImage, class TStatisticalModel, class TMetric, unsigned int VImageDimension>
typename TImage::Pointer
modelBasedImageToImageRegistration(std::string                         referenceFilename,
                                   std::string                         targetFilename,
                                   typename TStatisticalModel::Pointer model,
                                   std::string                         outputDfFilename,
                                   unsigned                            numberOfIterations)
{


  typedef itk::ImageFileReader<TImage>                                                                  ImageReaderType;
  typedef itk::InterpolatingStatisticalDeformationModelTransform<TVectorImage, double, VImageDimension> TransformType;
  typedef itk::LBFGSOptimizer                                                                           OptimizerType;
  typedef itk::ImageRegistrationMethod<TImage, TImage>        RegistrationFilterType;
  typedef itk::WarpImageFilter<TImage, TImage, TVectorImage>  WarperType;
  typedef itk::LinearInterpolateImageFunction<TImage, double> InterpolatorType;

  typename ImageReaderType::Pointer referenceReader = ImageReaderType::New();
  referenceReader->SetFileName(referenceFilename.c_str());
  referenceReader->Update();
  typename TImage::Pointer referenceImage = referenceReader->GetOutput();
  referenceImage->Update();

  typename ImageReaderType::Pointer targetReader = ImageReaderType::New();
  targetReader->SetFileName(targetFilename.c_str());
  targetReader->Update();
  typename TImage::Pointer targetImage = targetReader->GetOutput();
  targetImage->Update();

  // do the fitting
  typename TransformType::Pointer transform = TransformType::New();
  transform->SetStatisticalModel(model);
  transform->SetIdentity();

  // Setting up the fitting
  OptimizerType::Pointer optimizer = OptimizerType::New();
  optimizer->MinimizeOn();
  optimizer->SetMaximumNumberOfFunctionEvaluations(numberOfIterations);

  typedef IterationStatusObserver ObserverType;
  ObserverType::Pointer           observer = ObserverType::New();
  optimizer->AddObserver(itk::IterationEvent(), observer);

  typename TMetric::Pointer          metric = TMetric::New();
  typename InterpolatorType::Pointer interpolator = InterpolatorType::New();

  typename RegistrationFilterType::Pointer registration = RegistrationFilterType::New();
  registration->SetInitialTransformParameters(transform->GetParameters());
  registration->SetMetric(metric);
  registration->SetOptimizer(optimizer);
  registration->SetTransform(transform);
  registration->SetInterpolator(interpolator);
  registration->SetFixedImage(targetImage);
  registration->SetFixedImageRegion(targetImage->GetBufferedRegion());
  registration->SetMovingImage(referenceImage);

  try
  {
    std::cout << "Performing registration... " << std::flush;
    registration->Update();
    std::cout << "[done]" << std::endl;
  }
  catch (itk::ExceptionObject & o)
  {
    std::cout << "caught exception " << o << std::endl;
  }

  typename TVectorImage::Pointer df = model->DrawSample(transform->GetCoefficients());

  // write deformation field
  if (outputDfFilename.size() > 0)
  {
    typename itk::ImageFileWriter<TVectorImage>::Pointer df_writer = itk::ImageFileWriter<TVectorImage>::New();
    df_writer->SetFileName(outputDfFilename);
    df_writer->SetInput(df);
    df_writer->Update();
  }


  // warp reference
  std::cout << "Warping reference... " << std::flush;
  typename WarperType::Pointer warper = WarperType::New();
  warper->SetInput(referenceImage);
  warper->SetInterpolator(interpolator);
  warper->SetOutputSpacing(targetImage->GetSpacing());
  warper->SetOutputOrigin(targetImage->GetOrigin());
  warper->SetOutputDirection(targetImage->GetDirection());
  warper->SetDisplacementField(df);
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
 *  referenceFilename			-> Filename of reference image.
 *  referenceLandmarkFilename	-> Filename of reference landmark file ("" = disabled).
 *  targetFilename			-> Filename of target image.
 *  targetLandmarkFilename	-> Filename of target landmark file ("" = disabled).
 *  gaussianKernelSigma			-> Kernel parameter of Gaussian kernel.
 *  gaussianKernelScale			-> Kernel parameter of Gaussian kernel.
 *  landmarkUncertainty			-> Noise parameter to model uncertainty on the landmarks.
 *  similarityMetric			-> Similarity metric for the performance measure in the optimization.
 *  outputFilename			-> Filename of the resulting registration to be written to ("" = disabled).
 *  outputDfFilename			-> Filename of the resulting deformation field to be written to.
 *  numberOfBasisFunctions		-> Number of basis function used in the low-rank approximation.
 *  numberOfIterations			-> Maximum number of iterations to perform in the optimization.
 */
template <class PixelType, unsigned int VImageDimension>
void
runImageToImageRegistration(std::string referenceFilename,
                            std::string referenceLandmarkFilename,
                            std::string targetFilename,
                            std::string targetLandmarkFilename,
                            double      gaussianKernelSigma,
                            double      gaussianKernelScale,
                            double      landmarkUncertainty,
                            std::string similarityMetric,
                            std::string outputFilename,
                            std::string outputDfFilename,
                            unsigned    numberOfBasisFunctions,
                            unsigned    numberOfIterations)
{


  typedef itk::Image<PixelType, VImageDimension>                                              ImageType;
  typedef itk::Image<itk::Vector<float, VImageDimension>, VImageDimension>                    VectorImageType;
  typedef itk::StandardImageRepresenter<typename VectorImageType::PixelType, VImageDimension> RepresenterType;
  typedef itk::StatisticalModel<VectorImageType>                                              StatisticalModelType;
  typedef itk::MeanSquaresImageToImageMetric<ImageType, ImageType>                            MeanSquaresMetricType;
  typedef itk::NormalizedCorrelationImageToImageMetric<ImageType, ImageType> NormalizedCorrelationMetricType;

  // build deformation model
  typename StatisticalModelType::Pointer model =
    buildLowRankGPModel<RepresenterType, VectorImageType, StatisticalModelType>(
      referenceFilename.c_str(), gaussianKernelSigma, gaussianKernelScale, numberOfBasisFunctions);
  if (referenceLandmarkFilename.size() > 0 && targetLandmarkFilename.size() > 0)
  {
    model = constrainModel<VectorImageType, StatisticalModelType, VImageDimension>(
      model, referenceLandmarkFilename, targetLandmarkFilename, landmarkUncertainty);
  }

  // image to image registration with this model
  typename ImageType::Pointer registeredImage;
  if (similarityMetric == "MeanSquares")
  {
    registeredImage = modelBasedImageToImageRegistration<ImageType,
                                                         VectorImageType,
                                                         StatisticalModelType,
                                                         MeanSquaresMetricType,
                                                         VImageDimension>(
      referenceFilename, targetFilename, model, outputDfFilename, numberOfIterations);
  }
  if (similarityMetric == "NormalizedCorrelation")
  {
    registeredImage = modelBasedImageToImageRegistration<ImageType,
                                                         VectorImageType,
                                                         StatisticalModelType,
                                                         NormalizedCorrelationMetricType,
                                                         VImageDimension>(
      referenceFilename, targetFilename, model, outputDfFilename, numberOfIterations);
  }

  // write registered image
  if (outputFilename.size() > 0)
  {
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
int
main(int argc, char * argv[])
{

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
  if (argc != 10 && argc != 13)
  {
    std::cout << "***********************************************************" << std::endl;
    std::cout << "usage\t" << argv[0] << std::endl;
    std::cout << "referenceFilename:\t\t Filename of reference image." << std::endl;
    std::cout << "targetFilename:\t\t\t Filename of target image." << std::endl;
    std::cout << "outputFilename:\t\t\t Filename of the resulting registered image." << std::endl;
    std::cout << "outputDfFilename:\t\t Filename of the resulting deformation field." << std::endl;
    std::cout << "similarityMetric:\t\t Similarity metric: MeanSquares and NormalizedCorrelation are supported."
              << std::endl;
    std::cout << "gaussianKernelSigma:\t\t Sigma of the Gaussian kernel. (e.g. 70)" << std::endl;
    std::cout << "gaussianKernelScale:\t\t Scale of the Gaussian kernel. (e.g. 100)" << std::endl;
    std::cout << "numBasisFunctions:\t\t Number of basis functions to approximate. (e.g. 100)" << std::endl;
    std::cout << "numIterations:\t\t\t Number of iterations to perform in the optimization. (e.g. 100)" << std::endl
              << std::endl;

    std::cout << "or (including landmarks)\t" << argv[0] << std::endl;
    std::cout << "referenceFilename:\t\t Filename of reference image." << std::endl;
    std::cout << "referenceLandmarkFilename:\t Filename of reference landmark file." << std::endl;
    std::cout << "targetFilename:\t\t\t Filename of target image." << std::endl;
    std::cout << "targetLandmarkFilename:\t\t Filename of target landmark file." << std::endl;
    std::cout << "outputFilename:\t\t\t Filename of the resulting registered image." << std::endl;
    std::cout << "outputDfFilename:\t\t Filename of the resulting deformation field." << std::endl;
    std::cout << "similarityMetric:\t\t Similarity metric: MeanSquares and NormalizedCorrelation are supported."
              << std::endl;
    std::cout << "gaussianKernelSigma:\t\t Sigma of the Gaussian kernel. (e.g. 70)" << std::endl;
    std::cout << "gaussianKernelScale:\t\t Scale of the Gaussian kernel. (e.g. 100)" << std::endl;
    std::cout << "landmarkUncertainty:\t\t Uncertainty of the landmarks. (e.g. 0.1)" << std::endl;
    std::cout << "numBasisFunctions:\t\t Number of basis functions to approximate. (e.g. 100)" << std::endl;
    std::cout << "numIterations:\t\t\t Number of iterations to perform in the optimization. (e.g. 100)" << std::endl;
    exit(-1);
  }

  if (argc == 10)
  {
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
  if (argc == 13)
  {
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
  itk::ImageIOBase::Pointer imageIO =
    itk::ImageIOFactory::CreateImageIO(referenceFilename.c_str(), itk::ImageIOFactory::ReadMode);

  imageIO->SetFileName(referenceFilename);
  imageIO->ReadImageInformation();
  const unsigned numDimensions = imageIO->GetNumberOfDimensions();


  // print out parameters
  std::cout << "************************************************" << std::endl;
  std::cout << "Low-rank Gaussian process image registration:" << std::endl;
  std::cout << " - space dimensions\t\t" << numDimensions << std::endl;
  std::cout << " - reference\t\t\t" << referenceFilename << std::endl;
  if (referenceLandmarkFilename.size() > 0)
    std::cout << " - reference landmarks\t\t" << referenceLandmarkFilename << std::endl;
  std::cout << " - target\t\t\t" << targetFilename << std::endl;
  if (targetLandmarkFilename.size() > 0)
    std::cout << " - target landmarks\t\t" << targetLandmarkFilename << std::endl;
  std::cout << " - output\t\t\t" << outputFilename << std::endl;
  std::cout << " - output deformation field\t" << outputDfFilename << std::endl << std::endl;
  std::cout << " - similarity metric\t\t" << similarityMetric << std::endl;
  std::cout << " - Gaussian sigma\t\t" << gaussianKernelSigma << std::endl;
  std::cout << " - Gaussian scale\t\t" << gaussianKernelScale << std::endl;
  if (targetLandmarkFilename.size() > 0 && referenceLandmarkFilename.size() > 0)
    std::cout << " - Landmark uncertainty\t\t" << landmarkUncertainty << std::endl;
  std::cout << " - #basis functions\t\t" << numberOfBasisFunctions << std::endl;
  std::cout << " - #iterations\t\t\t" << numberOfIterations << std::endl << std::endl;


  if (!(similarityMetric == "NormalizedCorrelation" || similarityMetric == "MeanSquares"))
  {
    std::cout << "Error: only MeanSquares or NormalizedCorrelation supported as metric." << std::endl;
    return -1;
  }

  if (landmarkUncertainty == 0)
  {
    std::cout << "Warning: landmark uncertainty sould be greater than zero." << std::endl;
  }
  if (landmarkUncertainty < 0)
  {
    std::cout << "Error: landmark uncertainty has to be positive." << std::endl;
    return -1;
  }

  if (numDimensions == 2)
  { // run the image to image registration in 2D
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
  else if (numDimensions == 3)
  { // run the image to image registration in 2D
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
  else
  {
    assert(0);
  }

  std::cout << "Low-rank Gaussian process image to image registration has successfully finished." << std::endl;
}
