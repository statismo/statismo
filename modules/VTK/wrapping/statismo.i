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

%module statismo_VTK

%include "typemaps.i"
%include "std_string.i"
%include "std_list.i"
%include "std_vector.i"
%include "std_pair.i"
%include "std_vector.i"
%include "carrays.i"

%include "statismoTypemaps.i"
//%include "vtkPolyData.i"
%include "vtkStandardMeshRepresenter.i"
//%include "vtkUnstructuredGridRepresenter.i"
//%include "vtkStructuredPointsRepresenter.i"
//%include "TrivialVectorialRepresenter.i"


%{
#include "Representer.h"
#include "DataManager.h"
#include "DataManagerWithSurrogates.h"
#include "StatisticalModel.h"
#include "PosteriorModelBuilder.h"
#include "ReducedVarianceModelBuilder.h"
#include "PCAModelBuilder.h"
#include "statismo/core/Exceptions.h"
#include "statismo/core/CommonTypes.h"
#include "StatismoIO.h"
#include <list>
#include <string>
%}


namespace statismo {
class StatisticalModelException {
public:
    StatisticalModelException(const char* message);
};
}

%exception {
   try {
      $action
   } catch (statismo::StatisticalModelException &e) {
      PyErr_SetString(PyExc_RuntimeError, const_cast<char*>(e.what()));
      return NULL;
   }
}

//////////////////////////////////////////////////////
// DataManager
//////////////////////////////////////////////////////

namespace statismo {
template <typename T>
struct DataItem {


    typedef Representer<T> RepresenterType;
    typedef typename RepresenterType::DatasetConstPointerType DatasetConstPointerType;
    typedef typename RepresenterType::DatasetPointerType DatasetPointerType;


    std::string GetDatasetURI() const;
    %newobject GetSample;
    const DatasetPointerType GetSample() const;

    const  statismo::VectorType& GetSampleVector() const;
    virtual ~DataItem();
private:
    DataItem(const Representer<T>* representer, const std::string& filename, const statismo::VectorType& sampleVector);
};

template <typename T>
struct DataItemWithSurrogates : public DataItem<T> {
        const statismo::VectorType& GetSurrogateVector() const;
    const std::string& GetSurrogateFilename() const;
};
}

%template(DataItem_vtkPD) statismo::DataItem<vtkPolyData>;
%template(DataItemWithSurrogates_vtkPD) statismo::DataItemWithSurrogates<vtkPolyData>;


%traits_swigtype(statismo::DataItem<vtkPolyData>); // workaround for a swig bug with const ptr
%fragment(SWIG_Traits_frag(statismo::DataItem<vtkPolyData>));  // workaround for a swig bug with const ptr
%template(DataItemList_vtkPD) std::list<const statismo::DataItem<vtkPolyData> *>;


template <typename Representer>
class CrossValidationFold {
public:
    typedef DataItem<Representer> DataItemType;
    typedef std::list<const DataItemType*> DataItemListType;

    CrossValidationFold();
    DataItemListType GetTrainingData() const;
    DataItemListType GetTestingData() const;

};
%template(CrossValidationFold_vtkPD) statismo::CrossValidationFold<vtkPolyData>;

%template(CrossValidationFoldList_vtkPD) std::list<statismo::CrossValidationFold<vtkPolyData> >;


//////////////////////////////////////////////////////
// DataManager
//////////////////////////////////////////////////////

namespace statismo {


template <typename T>
class DataManager {
public:

    typedef statismo::Representer<T> RepresenterType;
    typedef typename RepresenterType::DatasetConstPointerType DatasetConstPointerType;
    typedef typename RepresenterType::DatasetPointerType DatasetPointerType;


    typedef DataItem<T> DataItemType;
    typedef std::list<const DataItemType *> DataItemListType;

    typedef CrossValidationFold<T> CrossValidationFoldType;
    typedef std::list<CrossValidationFoldType> CrossValidationFoldListType;




    %newobject Create;
    static DataManager* Create(const RepresenterType*);
    %newobject Load;
    static DataManager* Load(RepresenterType* representer, const char* filename);

    void AddDataset(DatasetConstPointerType dataset, const std::string& URI);
    unsigned GetNumberOfSamples() const;

    void Save(const char* filename);


    /** cross validation functionality */
    CrossValidationFoldListType GetCrossValidationFolds(unsigned nFolds, bool randomize = true) const;
    DataItemListType GetData() const;

private:
    DataManager();
};
}
%template(DataManager_vtkPD) statismo::DataManager<vtkPolyData>;


namespace statismo {
template <typename T>
class DataManagerWithSurrogates : public DataManager<T> {
public:
    typedef statismo::Representer<T> RepresenterType;
    typedef RepresenterType::DatasetConstPointerType DatasetConstPointerType;

    static DataManagerWithSurrogates<T>* Create(const RepresenterType* representer, const std::string& surrogTypeFilename);
    void AddDatasetWithSurrogates(DatasetConstPointerType datasetFilename, const std::string& surrogateFilename,  const std::string& surrogateFilename);

private:
    DataManagerWithSurrogates();
};
}

%template(DataManagerWithSurrogates_vtkPD) statismo::DataManagerWithSurrogates<vtkPolyData>;


//////////////////////////////////////////////////////
// PointValuePair
//////////////////////////////////////////////////////



%template(PointValuePair_vtkPD) std::pair<statismo::RepresenterTraits<vtkPolyData>::PointType, statismo::RepresenterTraits<vtkPolyData>::ValueType>;
%template(PointValueList_vtkPD) std::list<std::pair<statismo::RepresenterTraits<vtkPolyData>::PointType, statismo::RepresenterTraits<vtkPolyData>::ValueType> >;
%template(PointIdValuePair_vtkPD) std::pair<unsigned, statismo::RepresenterTraits<vtkPolyData>::ValueType>;
%template(PointIdValueList_vtkPD) std::list<std::pair<unsigned, statismo::RepresenterTraits<vtkPolyData>::ValueType> >;

//////////////////////////////////////////////////////
// PointValueWithCovariancePair
//////////////////////////////////////////////////////

// Never got this to work properly. The wrapping of statismo::MatrixType is not sophisticated enough to work with the std::pair and list wrappers.

//%template(PointValuePair_tvr) std::pair<TrivialVectorialRepresenter::PointType, TrivialVectorialRepresenter::ValueType>;
//%template(PointValueWithCovariancePair_vtkPD) std::pair< std::pair< vtkPoint,vtkPoint >,statismo::MatrixType >;
//%template(PointValueWithCovarianceList_vtkPD) std::list< std::pair< std::pair< vtkPoint,vtkPoint >,statismo::MatrixType > >;
//%template(MatrixList) std::list<statismo::MatrixType>;
//%template(MatrixMatrixPair) std::pair< statismo::MatrixType, statismo::MatrixType >;
//%template(MatrixMatrixList) std::list< std::pair< statismo::MatrixType, statismo::MatrixType > >;
//%template(PointMatrixPair) std::pair< vtkPoint, statismo::MatrixType >;
//%template(PointMatrixList) std::list< std::pair< vtkPoint, statismo::MatrixType > >;


//////////////////////////////////////////////////////
// ModelInfo
//////////////////////////////////////////////////////

namespace statismo {

class BuilderInfo {
public:
    typedef std::pair<std::string, std::string> KeyValuePair;
    typedef std::list<KeyValuePair> KeyValueList;

    const KeyValueList& GetDataInfo() const;
    const KeyValueList& GetParameterInfo() const;
};

class ModelInfo {
public:
    typedef std::vector<BuilderInfo> BuilderInfoList;
    const statismo::MatrixType& GetScoresMatrix() const;

    virtual void Save(const H5::CommonFG& publicFg) const;
    virtual void Load(const H5::CommonFG& publicFg);
    BuilderInfoList GetBuilderInfoList() const ;
};
}
%template (KeyValuePair) std::pair<std::string, std::string>;
%template(KeyValueList) std::list<std::pair<std::string, std::string> >;
%template(BuilderInfoList) std::vector<statismo::BuilderInfo>;

/////////////////////////////////////////////////////////////////
// Domain
/////////////////////////////////////////////////////////////////
namespace statismo {
template <typename PointType>
class Domain {
public:
    typedef std::vector<PointType> DomainPointsListType;
    const DomainPointsListType& GetDomainPoints() const;
    const unsigned GetNumberOfPoints() const;
};
}
%template(DomainVtkPoint) statismo::Domain<statismo::vtkPoint>;
%template(DomainId) statismo::Domain<unsigned int>;
%template(DomainPointsListVtkPoint) std::vector<statismo::vtkPoint>;
%template(DomainPointsListId) std::vector<unsigned int>;

//////////////////////////////////////////////////////
// StatisticalModel
//////////////////////////////////////////////////////

namespace statismo {
template <class T>
class StatisticalModel {
public:

    typedef Representer<T> RepresenterType ;
    typedef typename RepresenterType::DatasetPointerType DatasetPointerType;
    typedef typename RepresenterType::DatasetConstPointerType DatasetConstPointerType;
    typedef typename RepresenterType::ValueType ValueType;
    typedef typename RepresenterType::PointType PointType;


    typedef Domain<PointType> DomainType;

    typedef unsigned PointIdType;


    //typedef  PointValuePair<Representer>  PointValuePairType;
    typedef std::pair<PointType, ValueType> PointValuePairType;
    typedef std::pair<unsigned, ValueType> PointIdValuePairType;
    typedef std::list<PointValuePairType> PointValueListType;
    typedef std::list<PointIdValuePairType> PointIdValueListType;

    %newobject Create;
     static StatisticalModel* Create(const RepresenterType* representer,
                                     const statismo::VectorType& m,
                                     const statismo::MatrixType& orthonormalPCABasis,
                                     const statismo::VectorType& pcaVariance,
                                     double noiseVariance);
     virtual ~StatisticalModel();

    const RepresenterType* GetRepresenter() const;
    const DomainType& GetDomain() const;



     ValueType EvaluateSampleAtPoint(DatasetConstPointerType sample, const PointType& point) const;
     DatasetPointerType DrawMean() const;
     ValueType DrawMeanAtPoint(const PointType& pt) const;
     %rename("DrawMeanAtPointId") DrawMeanAtPoint(unsigned) const;
     ValueType DrawMeanAtPoint(unsigned ptId) const;

     %rename("DrawRandomSample") DrawSample() const;
     DatasetPointerType DrawSample() const;
     DatasetPointerType DrawSample(const statismo::VectorType& coeffs) const;
     ValueType DrawSampleAtPoint(const statismo::VectorType& coeffs, const PointType& pt) const;
     %rename("DrawSampleAtPointId") DrawSampleAtPoint(const statismo::VectorType&, unsigned) const;
     ValueType DrawSampleAtPoint(const statismo::VectorType& coeffs, unsigned ptId) const;

       DatasetPointerType DrawPCABasisSample(unsigned componentNumber) const;


     statismo::VectorType ComputeCoefficients(DatasetConstPointerType ds) const;
     statismo::VectorType ComputeCoefficientsForSampleVector(const statismo::VectorType& sample) const;
     statismo::VectorType ComputeCoefficientsForPointValues(const PointValueListType&  pointValues) const;
     statismo::VectorType ComputeCoefficientsForPointIDValues(const PointIdValueListType&  pointValues) const;

     double ComputeLogProbability(DatasetConstPointerType ds) const;
     double ComputeProbability(DatasetConstPointerType ds) const;
         double ComputeMahalanobisDistance(DatasetConstPointerType ds) const;

     statismo::MatrixType GetCovarianceAtPoint(const PointType& pt1, const PointType& pt2) const;
     statismo::MatrixType GetCovarianceAtPoint(unsigned ptId1, unsigned ptId2) const;

     unsigned GetNumberOfPrincipalComponents();
     statismo::VectorType DrawSampleVector(const statismo::VectorType& coefficients) const;
     const statismo::MatrixType& GetPCABasisMatrix() const ;
     const statismo::MatrixType GetOrthonormalPCABasisMatrix() const ;
     const statismo::VectorType& GetPCAVarianceVector() const;
     const statismo::VectorType& GetMeanVector() const;

      const ModelInfo& GetModelInfo() const;
      void SetModelInfo(const ModelInfo& modelInfo);
    double GetNoiseVariance() const;

    private:
    StatisticalModel();

};
}

%template(StatisticalModel_vtkPD) statismo::StatisticalModel<vtkPolyData>;

//////////////////////////////////////////////////////
// PCAModelBuilder
//////////////////////////////////////////////////////


namespace statismo {
%newobject *::BuildNewModel;
template <typename T>
class PCAModelBuilder {
public:

    typedef ModelBuilder<T> Superclass;
    typedef typename DataManager<T> DataManagerType;
    typedef typename DataManagerType::DataItemListType DataItemListType;

        typedef enum { JacobiSVD, SelfAdjointEigenSolver } EigenValueMethod;

    %newobject Create;
    static PCAModelBuilder* Create();

         StatisticalModel<T>* BuildNewModel(const DataItemListType& sampleList, double noiseVariance, bool computeScores=true, EigenValueMethod method = JacobiSVD) const;
private:
    PCAModelBuilder();

};
}

%template(PCAModelBuilder_vtkPD) statismo::PCAModelBuilder<vtkPolyData>;


//////////////////////////////////////////////////////
// PosteriorModelBuilder
//////////////////////////////////////////////////////



namespace statismo {
%newobject *::BuildNewModelFromModel;
%newobject *::BuildNewModel;

template <typename T>
class PosteriorModelBuilder {
    typedef ModelBuilder<T> Superclass;
public:
    typedef  DataManager<T>                 DataManagerType;
    typedef typename DataManagerType::DataItemListType DataItemListType;
    typedef  StatisticalModel<T>     StatisticalModelType;
    typedef typename StatisticalModelType::PointValueType PointValueType;
    typedef  typename StatisticalModelType::PointValueListType PointValueListType;

    %newobject Create;
    static PosteriorModelBuilder* Create();
    virtual ~PosteriorModelBuilder();

    StatisticalModelType* BuildNewModelFromModel(const StatisticalModelType* model,    const PointValueListType& pointValues,  double pointValuesNoiseVariance, bool computeScores=true) const;
    StatisticalModelType* BuildNewModel(const DataItemListType& sampleList, const PointValueListType& pointValues,  double pointValuesNoiseVariance,    double noiseVariance) const;

    private:
        PosteriorModelBuilder();
};
}

%template(PosteriorModelBuilder_vtkPD) statismo::PosteriorModelBuilder<vtkPolyData>;




//////////////////////////////////////////////////////
// ReducedVarianceModelBuilder
//////////////////////////////////////////////////////

namespace statismo {
%newobject *::BuildNewModelFromModel;

template <typename Representer>
class ReducedVarianceModelBuilder {
    typedef ModelBuilder<Representer> Superclass;
public:
    typedef  StatisticalModel<Representer>     StatisticalModelType;

    %newobject Create;
    static ReducedVarianceModelBuilder* Create();
    virtual ~ReducedVarianceModelBuilder();

        StatisticalModelType* BuildNewModelWithLeadingComponents(const StatisticalModelType* model, unsigned numberOfLeadingComponents) const;
        StatisticalModelType* BuildNewModelWithVariance(const StatisticalModelType* model,    double totalVariance) const;
        // StatisticalModelType* BuildNewModelFromModel(const StatisticalModelType* model,    double totalVariance) const;

    private:
        ReducedVarianceModelBuilder();
};
}

%template(ReducedVarianceModelBuilder_vtkPD) statismo::ReducedVarianceModelBuilder<vtkPolyData>;


//////////////////////////////////////////////////////
// IO
//////////////////////////////////////////////////////
namespace statismo {
template <typename T>
class IO {
private:
    typedef StatisticalModel<T>  StatisticalModelType;
    IO();
public:

    static StatisticalModelType* LoadStatisticalModel(
        typename StatisticalModelType::RepresenterType *representer,
        const std::string &filename,
        unsigned maxNumberOfPCAComponents = std::numeric_limits<unsigned>::max());
    static StatisticalModelType* LoadStatisticalModel(
        typename StatisticalModelType::RepresenterType *representer,
        const H5::Group &modelRoot,
        unsigned maxNumberOfPCAComponents = std::numeric_limits<unsigned>::max());
    static void SaveStatisticalModel(const StatisticalModelType *const model, const std::string &filename);
};
}
%template(IO_vtkPD) statismo::IO<vtkPolyData>;
