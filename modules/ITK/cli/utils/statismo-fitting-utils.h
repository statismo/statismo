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

#include <iostream>
#include <vector>

#include <statismo/ITK/itkPosteriorModelBuilder.h>

class ConsoleOutputSilencer;
template <class DataType>
std::vector<typename DataType::PointType>
readLandmarksFile(std::string path);
template <class DataType, class StatisticalModelType, class PointsLocatorType>
typename StatisticalModelType::Pointer
buildPosteriorShapeModel(typename StatisticalModelType::Pointer pModel,
                         const std::string &                    strFixedLandmarksFileName,
                         const std::string &                    strMovingLandmarksFileName,
                         const double                           dLandmarksVariance);
template <class DataType, class StatisticalModelType>
typename StatisticalModelType::Pointer
buildPosteriorShapeModel(typename StatisticalModelType::Pointer pModel,
                         typename DataType::Pointer             pMesh,
                         const double                           dVariance);
template <class DataType, class StatisticalModelType>
typename StatisticalModelType::Pointer
buildPosteriorDeformationModel(typename StatisticalModelType::Pointer pModel,
                               const std::string &                    strFixedLandmarksFileName,
                               const std::string &                    strMovingLandmarksFileName,
                               const double                           dLandmarksVariance);
template <class OptimizerType>
void
initializeOptimizer(typename OptimizerType::Pointer pOptimizer,
                    const unsigned                  uNumberOfIterations,
                    const unsigned                  uNumberOfModelComponents,
                    const unsigned                  uTotalNumberOfOptimizationParameters,
                    const bool                      bPrintFittingInformation,
                    ConsoleOutputSilencer *         pCOSilencer);

#ifdef _WIN32
#  include <io.h>
#  define DUP(f) _dup(f)
#  define DUP2(f, newf) _dup2(f, newf)
#  define FILENO(f) _fileno(f)
#  define FOPEN(f, mode) fopen_s(f, mode)
#  define CLOSE(f) _close(f)
#else
#  include <unistd.h>
#  define DUP(fd) dup(fd)
#  define DUP2(fd, newfd) dup2(fd, newfd)
#  define FILENO(f) fileno(f)
#  define FOPEN(f, mode) fopen(f, mode)
#  define CLOSE(f) close(f)
#endif

// The optimizer may print something despite me not wanting it to print anything. It is thus that I manually silence the
// console output when it's not me printing.
class ConsoleOutputSilencer
{
private:
  int    iOldStdOutDescriptor;
  int    iOldStdErrDescriptor;
  int    iNullDescriptor;
  FILE * fNullFile;
  bool   bIsOutputEnabled;

  void
  flushAll()
  {
    fflush(stdout);
    fflush(stderr);
    fflush(fNullFile);
  }

public:
  ~ConsoleOutputSilencer()
  {
    if (bIsOutputEnabled == false)
    {
      enableOutput();
    }
    CLOSE(iNullDescriptor);
  }

  ConsoleOutputSilencer()
  {
#ifdef _WIN32
    fopen_s(&fNullFile, "NUL", "w");
#else
    fNullFile = fopen("/dev/null", "w");
#endif
    iOldStdOutDescriptor = DUP(FILENO(stdout));
    iOldStdErrDescriptor = DUP(FILENO(stderr));
    iNullDescriptor = FILENO(fNullFile);
    bIsOutputEnabled = true;
  }

  void
  disableOutput()
  {
    if (bIsOutputEnabled == false)
    {
      return;
    }
    bIsOutputEnabled = false;
    flushAll();
    DUP2(iNullDescriptor, FILENO(stdout));
    DUP2(iNullDescriptor, FILENO(stderr));
  }

  void
  enableOutput()
  {
    if (bIsOutputEnabled == true)
    {
      return;
    }
    bIsOutputEnabled = true;
    flushAll();
    DUP2(iOldStdOutDescriptor, FILENO(stdout));
    DUP2(iOldStdErrDescriptor, FILENO(stderr));
  }
};

template <class OptimizerType>
class IterationStatusObserver : public itk::Command
{
public:
  typedef IterationStatusObserver<OptimizerType> Self;
  typedef itk::Command                           Superclass;
  typedef itk::SmartPointer<Self>                Pointer;
  typedef const OptimizerType *                  OptimizerPointer;

  itkNewMacro(Self);

  void
  Execute(itk::Object * caller, const itk::EventObject & event)
  {
    Execute((const itk::Object *)caller, event);
  }

  void
  Execute(const itk::Object * object, const itk::EventObject & event)
  {
    OptimizerPointer optimizer = dynamic_cast<OptimizerPointer>(object);

    if (itk::IterationEvent().CheckEvent(&event) == false || optimizer == NULL)
    {
      return;
    }

    if (coSilencer != NULL)
    {
      coSilencer->enableOutput();
    }
    std::cout << "Iteration: " << ++m_iter_no;
    std::cout << "; Value: " << optimizer->GetCachedValue();
    std::cout << "; Current Parameters: " << optimizer->GetCachedCurrentPosition() << std::endl;
    if (coSilencer != NULL)
    {
      coSilencer->disableOutput();
    }
  }

  void
  SetConsoleSilencer(ConsoleOutputSilencer * pCOSilencer)
  {
    coSilencer = pCOSilencer;
  }

protected:
  IterationStatusObserver()
    : m_iter_no(0)
    , coSilencer(NULL){};

  virtual ~IterationStatusObserver(){};

private:
  int                     m_iter_no;
  ConsoleOutputSilencer * coSilencer;
};


template <class OptimizerType>
void
initializeOptimizer(typename OptimizerType::Pointer pOptimizer,
                    const unsigned                  uNumberOfIterations,
                    const unsigned                  uNumberOfModelComponents,
                    const unsigned                  uTotalNumberOfOptimizationParameters,
                    const bool                      bPrintFittingInformation,
                    ConsoleOutputSilencer *         pCOSilencer)
{
  const unsigned uNumberOfRigid2DtransformComponents = 3;
  const unsigned uNumberOfRigid3DtransformComponents = 6;

  pOptimizer->SetMaximumNumberOfFunctionEvaluations(uNumberOfIterations);
  pOptimizer->MinimizeOn();

  if (bPrintFittingInformation == true)
  {
    typedef IterationStatusObserver<OptimizerType> ObserverType;
    typename ObserverType::Pointer                 pObserver = ObserverType::New();
    pObserver->SetConsoleSilencer(pCOSilencer);
    pOptimizer->AddObserver(itk::IterationEvent(), pObserver);
  }

  unsigned uNrOfRotationComponents;
  unsigned uNrOfTranslationComponents;
  switch (uTotalNumberOfOptimizationParameters - uNumberOfModelComponents)
  {
    case uNumberOfRigid3DtransformComponents:
      uNrOfRotationComponents = 3;
      uNrOfTranslationComponents = 3;
      break;
    case uNumberOfRigid2DtransformComponents:
      uNrOfRotationComponents = 1;
      uNrOfTranslationComponents = 2;
      break;
    case 0:
      break;
    default:
      itkGenericExceptionMacro(<< "Unknown Transform. Can't scale for that one.")
  }


  if (uNumberOfModelComponents != uTotalNumberOfOptimizationParameters)
  {
    const double dModelParamScale = 3;
    const double dRotationScale = 0.1;
    const double dTranslationScale = 1;
    // set the scales of the optimizer, to compensate for potentially different scales of translation, rotation and
    // shape parameters
    typename OptimizerType::ScalesType scales(uTotalNumberOfOptimizationParameters);
    unsigned                           count = 0;

    for (unsigned i = 0; i < uNrOfRotationComponents; ++i, ++count)
    {
      scales[count] = 1.0 / (dRotationScale);
    }
    for (unsigned i = 0; i < uNrOfTranslationComponents; ++i, ++count)
    {
      scales[count] = 1.0 / (dTranslationScale);
    }
    for (unsigned i = 0; i < uNumberOfModelComponents; ++i, ++count)
    {
      scales[count] = 1.0 / (dModelParamScale);
    }
    pOptimizer->SetScales(scales);
  }
}

template <class DataType, class StatisticalModelType>
typename StatisticalModelType::Pointer
buildPosteriorDeformationModel(typename StatisticalModelType::Pointer pModel,
                               const std::string &                    strFixedLandmarksFileName,
                               const std::string &                    strMovingLandmarksFileName,
                               const double                           dLandmarksVariance)
{
  typedef std::vector<typename DataType::PointType> PointVector;
  PointVector vFixedLandmarks = readLandmarksFile<DataType>(strFixedLandmarksFileName);
  PointVector vMovingLandmarks = readLandmarksFile<DataType>(strMovingLandmarksFileName);

  if (vFixedLandmarks.size() != vMovingLandmarks.size())
  {
    itkGenericExceptionMacro(<< "There have to be an equal number of fixed and moving Landmarks.")
  }

  typename StatisticalModelType::PointValueListType vConstraints;

  typename PointVector::iterator pFixed = vFixedLandmarks.begin();
  for (typename PointVector::iterator pMoving = vMovingLandmarks.begin(); pMoving != vMovingLandmarks.end();
       ++pMoving, ++pFixed)
  {
    typename DataType::PixelType pxDisplacement;
    for (unsigned i = 0; i < pFixed->Size(); ++i)
    {
      pxDisplacement[i] = (*pMoving)[i] - (*pFixed)[i];
    }
    typename StatisticalModelType::PointValuePairType pointValue(*pFixed, pxDisplacement);
    vConstraints.push_back(pointValue);
  }

  typedef itk::PosteriorModelBuilder<DataType> PosteriorModelBuilderType;
  typename PosteriorModelBuilderType::Pointer  pPosteriorModelBuilder = PosteriorModelBuilderType::New();
  return pPosteriorModelBuilder->BuildNewModelFromModel(pModel, vConstraints, dLandmarksVariance, false);
}


template <class DataType, class StatisticalModelType>
typename StatisticalModelType::Pointer
buildPosteriorShapeModel(typename StatisticalModelType::Pointer pModel,
                         typename DataType::Pointer             pMesh,
                         const double                           dVariance)
{
  if (pMesh->GetNumberOfPoints() != pModel->GetRepresenter()->GetReference()->GetNumberOfPoints())
  {
    itkGenericExceptionMacro(<< "The provided Mesh is not in correspondence.")
  }

  typename StatisticalModelType::PointValueListType vConstraints;

  typename DataType::PointsContainer::Iterator pFixed = pModel->GetRepresenter()->GetReference()->GetPoints()->Begin();
  for (typename DataType::PointsContainer::Iterator pMoving = pMesh->GetPoints()->Begin();
       pMoving != pMesh->GetPoints()->End();
       ++pMoving, ++pFixed)
  {
    typename StatisticalModelType::PointValuePairType pointValue(pFixed->Value(), pMoving->Value());
    vConstraints.push_back(pointValue);
  }

  typedef itk::PosteriorModelBuilder<DataType> PosteriorModelBuilderType;
  typename PosteriorModelBuilderType::Pointer  pPosteriorModelBuilder = PosteriorModelBuilderType::New();
  return pPosteriorModelBuilder->BuildNewModelFromModel(pModel, vConstraints, dVariance, false);
}

template <class DataType, class StatisticalModelType, class PointsLocatorType>
typename StatisticalModelType::Pointer
buildPosteriorShapeModel(typename StatisticalModelType::Pointer pModel,
                         const std::string &                    strFixedLandmarksFileName,
                         const std::string &                    strMovingLandmarksFileName,
                         const double                           dLandmarksVariance)
{
  std::vector<typename DataType::PointType> vFixedLandmarks = readLandmarksFile<DataType>(strFixedLandmarksFileName);
  std::vector<typename DataType::PointType> vMovingLandmarks = readLandmarksFile<DataType>(strMovingLandmarksFileName);

  if (vFixedLandmarks.size() != vMovingLandmarks.size())
  {
    itkGenericExceptionMacro(<< "There have to be an equal number of fixed and moving Landmarks.")
  }

  typename DataType::Pointer          pReference = pModel->GetRepresenter()->GetReference();
  typename PointsLocatorType::Pointer pPointLocator = PointsLocatorType::New();
  pPointLocator->SetPoints(pReference->GetPoints());
  pPointLocator->Initialize();

  const typename DataType::PointsContainer *        pcReferenceMeshPoints = pReference->GetPoints();
  typename StatisticalModelType::PointValueListType vConstraints;

  for (unsigned i = 0; i < vFixedLandmarks.size(); ++i)
  {

    unsigned                     iClosestPointId = pPointLocator->FindClosestPoint(vFixedLandmarks[i]);
    typename DataType::PointType refPoint = pcReferenceMeshPoints->at(iClosestPointId);

    // compensate for the rigid transformation that was applied to the model
    typename StatisticalModelType::PointValuePairType pointValue(refPoint, vMovingLandmarks[i]);
    vConstraints.push_back(pointValue);
  }

  typedef itk::PosteriorModelBuilder<DataType> PosteriorModelBuilderType;
  typename PosteriorModelBuilderType::Pointer  pPosteriorModelBuilder = PosteriorModelBuilderType::New();
  return pPosteriorModelBuilder->BuildNewModelFromModel(pModel, vConstraints, dLandmarksVariance, false);
}

template <class DataType>
std::vector<typename DataType::PointType>
readLandmarksFile(std::string path)
{
  std::vector<typename DataType::PointType> vLandmarks;

  std::ifstream file;
  try
  {
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    file.open(path.c_str(), std::ifstream::in);

    std::string line;
    while (getline(file, line))
    {
      if (line != "")
      {
        // reading files with windows EOL on linux results in the \r not being removed from the line ending
        if (*line.rbegin() == '\r')
        {
          line.erase(line.length() - 1, 1);
        }
        // typedef boost::tokenizer<boost::escaped_list_separator<char> > TokenizerType;
        // TokenizerType t(line);
        // TODO: Replace with a real csv tokenizer that can handle coma in escaped string
        auto                                   t = statismo::utils::Split<','>(line);
        typename DataType::PointType           p;
        typename DataType::PointType::Iterator pointIter = p.Begin();
        // The first element is the description/name and will be ignored
        for (auto i = ++t.begin(); i != t.end(); ++i, ++pointIter)
        {
          try
          {
            float fCoordValue = statismo::utils::LexicalCast<float>(*i);
            if (pointIter == p.End())
            {
              // ignore the last point if it is equal to 0 in the 2D case (and it really is the last point)
              if (p.Size() == 2 && ++i == t.end())
              {
                if (fCoordValue == 0)
                {
                  break;
                }
                else
                {
                  itkGenericExceptionMacro(<< "The last point in the 2D case has to be 0 in the following line: '"
                                           << line << "' (file: " << path << ")");
                }
              }
              else
              {
                itkGenericExceptionMacro(<< "Too many point components were found in this line: '" << line
                                         << "' (file: " << path << ")");
              }
            }
            *pointIter = fCoordValue;
          }
          catch (const std::bad_cast & e)
          {
            itkGenericExceptionMacro(<< "Could not parse '" << (*i) << "' to a float in this line: '" << line
                                     << "' in the file '" << path << "'");
          }
        }
        if (pointIter != p.End())
        {
          itkGenericExceptionMacro(<< "Not enough point components were found in this line: '" << line
                                   << "' (file: " << path << ")");
        }
        vLandmarks.push_back(p);
      }
    }
  }
  catch (std::ifstream::failure e)
  {
    if (file.eof() == false)
    {
      throw std::ifstream::failure("Failed to read a file: '" + path + "'");
    }
  }

  return vLandmarks;
}
