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

#ifndef __StatisticalModel_hxx
#define __StatisticalModel_hxx

#include <cmath>

#include <fstream>
#include <iostream>
#include <string>

#include "Exceptions.h"
#include "HDF5Utils.h"
#include "ModelBuilder.h"
#include "StatisticalModel.h"

namespace statismo {

template <typename T>
StatisticalModel<T>::StatisticalModel(const RepresenterType* representer, const VectorType& m, const MatrixType& orthonormalPCABasis, const VectorType& pcaVariance, double noiseVariance)
    : m_representer(representer->Clone()),
      m_mean(m),
      m_pcaVariance(pcaVariance),
      m_noiseVariance(noiseVariance),
      m_cachedValuesValid(false),
      m_modelLoaded(false) {
    VectorType D = pcaVariance.array().sqrt();
    m_pcaBasisMatrix = orthonormalPCABasis * DiagMatrixType(D);
}

template <typename T>
StatisticalModel<T>::StatisticalModel(const RepresenterType* representer)
    : m_representer(representer->Clone()),  m_noiseVariance(0), m_cachedValuesValid(0) {
}


template <typename T>
StatisticalModel<T>::~StatisticalModel() {

    if (m_representer != 0) {
//		 not all representers can implement a const correct version of delete.
//		 We therefore simply const cast it. This is save here.
        const_cast<RepresenterType*>(m_representer)->Delete();
        m_representer = 0;
    }

}



template <typename T>
typename StatisticalModel<T>::DatasetPointerType
StatisticalModel<T>::DatasetToSample(DatasetConstPointerType ds) const {
    DatasetPointerType sample;
    sample = m_representer->CloneDataset(ds);
    return sample;
}


template <typename T>
typename StatisticalModel<T>::ValueType
StatisticalModel<T>::EvaluateSampleAtPoint(const DatasetConstPointerType sample, const PointType& point) const {
    unsigned ptid = this->m_representer->GetPointIdForPoint(point);
    return EvaluateSampleAtPoint(sample, ptid);
}


template <typename T>
typename StatisticalModel<T>::ValueType
StatisticalModel<T>::EvaluateSampleAtPoint(const DatasetConstPointerType sample, unsigned ptid) const {
    return this->m_representer->PointSampleFromSample(sample, ptid);
}


template <typename T>
typename StatisticalModel<T>::DatasetPointerType
StatisticalModel<T>::DrawMean() const {
    VectorType coeffs = VectorType::Zero(this->GetNumberOfPrincipalComponents());
    return DrawSample(coeffs, false);
}


template <typename T>
typename StatisticalModel<T>::ValueType
StatisticalModel<T>::DrawMeanAtPoint(const PointType& point) const {
    VectorType coeffs = VectorType::Zero(this->GetNumberOfPrincipalComponents());
    return DrawSampleAtPoint(coeffs, point);

}

template <typename T>
typename StatisticalModel<T>::ValueType
StatisticalModel<T>::DrawMeanAtPoint(unsigned pointId) const {
    VectorType coeffs = VectorType::Zero(this->GetNumberOfPrincipalComponents());
    return DrawSampleAtPoint(coeffs, pointId, false);

}




template <typename T>
typename StatisticalModel<T>::DatasetPointerType
StatisticalModel<T>::DrawSample(bool addNoise) const {

    // we create random coefficients and draw a random sample from the model
    VectorType coeffs = Utils::generateNormalVector(GetNumberOfPrincipalComponents());

    return DrawSample(coeffs, addNoise);
}


template <typename T>
typename StatisticalModel<T>::DatasetPointerType
StatisticalModel<T>::DrawSample(const VectorType& coefficients, bool addNoise) const {
    return m_representer->SampleVectorToSample(DrawSampleVector(coefficients, addNoise));
}


template <typename T>
typename StatisticalModel<T>::DatasetPointerType
StatisticalModel<T>::DrawPCABasisSample(const unsigned pcaComponent) const {
    if (pcaComponent >= this->GetNumberOfPrincipalComponents()) {
        throw StatisticalModelException("Wrong pcaComponent index provided to DrawPCABasisSample!");
    }


    return m_representer->SampleVectorToSample( m_pcaBasisMatrix.col(pcaComponent));
}



template <typename T>
VectorType
StatisticalModel<T>::DrawSampleVector(const VectorType& coefficients, bool addNoise) const {

    if (coefficients.size() != this->GetNumberOfPrincipalComponents()) {
        throw StatisticalModelException("Incorrect number of coefficients provided !");
    }

    unsigned vectorSize = this->m_mean.size();
    assert (vectorSize != 0);

    VectorType epsilon = VectorType::Zero(vectorSize);
    if (addNoise) {
        epsilon = Utils::generateNormalVector(vectorSize) * sqrt(m_noiseVariance);
    }


    return m_mean+ m_pcaBasisMatrix * coefficients + epsilon;
}


template <typename T>
typename StatisticalModel<T>::ValueType
StatisticalModel<T>::DrawSampleAtPoint(const VectorType& coefficients, const PointType& point, bool addNoise) const {

    unsigned ptId = this->m_representer->GetPointIdForPoint(point);

    return DrawSampleAtPoint(coefficients, ptId, addNoise);

}

template <typename T>
typename StatisticalModel<T>::ValueType
StatisticalModel<T>::DrawSampleAtPoint(const VectorType& coefficients, const unsigned ptId, bool addNoise) const {

    unsigned dim = m_representer->GetDimensions();

    VectorType v(dim);
    VectorType epsilon = VectorType::Zero(dim);
    if (addNoise) {
        epsilon = Utils::generateNormalVector(dim) * sqrt(m_noiseVariance);
    }
    for (unsigned d = 0; d < dim; d++) {
        unsigned idx =m_representer->MapPointIdToInternalIdx(ptId, d);

        if (idx >= m_mean.rows()) {
            std::ostringstream os;
            os << "Invalid idx computed in DrawSampleAtPoint. ";
            os << " The most likely cause of this error is that you provided an invalid point id (" << ptId <<")";
            throw StatisticalModelException(os.str().c_str());
        }

        v[d] = m_mean[idx] + m_pcaBasisMatrix.row(idx).dot(coefficients) + epsilon[d];
    }

    return this->m_representer->PointSampleVectorToPointSample(v);
}



template <typename T>
MatrixType
StatisticalModel<T>::GetCovarianceAtPoint(const PointType& pt1, const PointType& pt2) const {
    unsigned ptId1 = this->m_representer->GetPointIdForPoint(pt1);
    unsigned ptId2 = this->m_representer->GetPointIdForPoint(pt2);

    return GetCovarianceAtPoint(ptId1, ptId2);
}

template <typename T>
MatrixType
StatisticalModel<T>::GetCovarianceAtPoint(unsigned ptId1, unsigned ptId2) const {
    unsigned dim = m_representer->GetDimensions();
    MatrixType cov(dim, dim);

    for (unsigned i = 0; i < dim; i++) {
        unsigned idxi = m_representer->MapPointIdToInternalIdx(ptId1, i);
        VectorType vi = m_pcaBasisMatrix.row(idxi);
        for (unsigned j = 0; j < dim; j++) {
            unsigned idxj = m_representer->MapPointIdToInternalIdx(ptId2, j);
            VectorType vj = m_pcaBasisMatrix.row(idxj);
            cov(i,j) = vi.dot(vj);
            if (i == j) cov(i,j) += m_noiseVariance;
        }
    }
    return cov;
}

template <typename T>
MatrixType
StatisticalModel<T>::GetCovarianceMatrix() const {
    MatrixType M = m_pcaBasisMatrix * m_pcaBasisMatrix.transpose();
    M.diagonal() += m_noiseVariance * VectorType::Ones(m_pcaBasisMatrix.rows());
    return M;
}


template <typename T>
VectorType
StatisticalModel<T>::ComputeCoefficientsForDataset(DatasetConstPointerType dataset) const {
    DatasetPointerType sample;
    sample = m_representer->CloneDataset(dataset);
    VectorType v = ComputeCoefficientsForSample(sample);
    m_representer->DeleteDataset(sample);
    return v;
}

template <typename T>
VectorType
StatisticalModel<T>::ComputeCoefficientsForSample(DatasetConstPointerType sample) const {
    return ComputeCoefficientsForSampleVector(m_representer->SampleToSampleVector(sample));
}

template <typename T>
VectorType
StatisticalModel<T>::ComputeCoefficientsForSampleVector(const VectorType& sample) const {

    CheckAndUpdateCachedParameters();

    const MatrixType& WT = m_pcaBasisMatrix.transpose();

    VectorType coeffs = m_MInverseMatrix * (WT * (sample - m_mean));
    return coeffs;
}



template <typename T>
VectorType
StatisticalModel<T>::ComputeCoefficientsForPointValues(const PointValueListType&  pointValueList, double pointValueNoiseVariance) const {
    PointIdValueListType ptIdValueList;

    for (typename PointValueListType::const_iterator it  = pointValueList.begin();
            it != pointValueList.end();
            ++it) {
        ptIdValueList.push_back(PointIdValuePairType(m_representer->GetPointIdForPoint(it->first), it->second));
    }
    return ComputeCoefficientsForPointIDValues(ptIdValueList, pointValueNoiseVariance);
}

template <typename T>
VectorType
StatisticalModel<T>::ComputeCoefficientsForPointIDValues(const PointIdValueListType&  pointIdValueList, double pointValueNoiseVariance) const {

    unsigned dim = m_representer->GetDimensions();

    double noiseVariance = std::max(pointValueNoiseVariance, (double) m_noiseVariance);

    // build the part matrices with , considering only the points that are fixed
    MatrixType PCABasisPart(pointIdValueList.size()* dim, this->GetNumberOfPrincipalComponents());
    VectorType muPart(pointIdValueList.size() * dim);
    VectorType sample(pointIdValueList.size() * dim);

    unsigned i = 0;
    for (typename PointIdValueListType::const_iterator it = pointIdValueList.begin(); it != pointIdValueList.end(); ++it) {
        VectorType val = this->m_representer->PointSampleToPointSampleVector(it->second);
        unsigned pt_id = it->first;
        for (unsigned d = 0; d < dim; d++) {
            PCABasisPart.row(i * dim + d) = this->GetPCABasisMatrix().row(m_representer->MapPointIdToInternalIdx(pt_id, d));
            muPart[i * dim + d] = this->GetMeanVector()[m_representer->MapPointIdToInternalIdx(pt_id, d)];
            sample[i * dim + d] = val[d];
        }
        i++;
    }

    MatrixType M = PCABasisPart.transpose() * PCABasisPart;
    M.diagonal() += noiseVariance * VectorType::Ones(PCABasisPart.cols());
    VectorType coeffs = M.inverse() * PCABasisPart.transpose() * (sample - muPart);

    return coeffs;
}


template <typename T>
double
StatisticalModel<T>::ComputeLogProbabilityOfDataset(DatasetConstPointerType ds) const {
    VectorType alpha = ComputeCoefficientsForDataset(ds);
    return ComputeLogProbabilityOfCoefficients(alpha);
}

template <typename T>
double
StatisticalModel<T>::ComputeProbabilityOfDataset(DatasetConstPointerType ds) const {
    VectorType alpha = ComputeCoefficientsForDataset(ds);
    return ComputeProbabilityOfCoefficients(alpha);
}


template <typename T>
double
StatisticalModel<T>::ComputeLogProbabilityOfCoefficients(const VectorType& coefficents) const {
    return log(pow(2 * PI, -0.5 * this->GetNumberOfPrincipalComponents())) - 0.5 * coefficents.squaredNorm();
}

template <typename T>
double
StatisticalModel<T>::ComputeProbabilityOfCoefficients(const VectorType& coefficients) const {
    return pow(2 * PI, - 0.5 * this->GetNumberOfPrincipalComponents()) * exp(- 0.5 * coefficients.squaredNorm());
}


template <typename T>
double
StatisticalModel<T>::ComputeMahalanobisDistanceForDataset(DatasetConstPointerType ds) const {
    VectorType alpha = ComputeCoefficientsForDataset(ds);
    return std::sqrt(alpha.squaredNorm());
}


template <typename T>
VectorType
StatisticalModel<T>::RobustlyComputeCoefficientsForDataset(DatasetConstPointerType ds, unsigned nIterations, unsigned nu, double sigma2) const {
    throw NotImplementedException("StatisticalModel", "RobustlyComputeCoefficientsForDataset");
    /*
    unsigned dim = Representer::GetDimensions();

    // we use double to improve the stability
    MatrixType U = Utils::toDouble(this->m_pcaBasisMatrix);
    VectorType yfloat;
    m_representer->DatasetToSample(ds,&yfloat);

    MatrixTypeDoublePrecision y = Eigen::Map::toDouble(yfloat);
    VectorTypeDoublePrecision mu = Utils::toDouble(this->m_mean);


    const MatrixTypeDoublePrecision& UT = U.transpose();

    unsigned nPCA = GetNumberOfPrincipalComponents();

    typedef typename Representer::DatasetTraitsType DatasetTraitsType;


    Eigen::DiagonalWrapper<VectorTypeDoublePrecision> Vinv(VectorTypeDoublePrecision::Zero(y.size());
    Eigen::DiagonalWrapper<VectorTypeDoublePrecision> VinvSqrt(VectorTypeDoublePrecision::Zero(y.size()));

    y -= mu;
    VectorTypeDoublePrecision f = VectorTypeDoublePrecistion::Zero(y.size());

    Eigen::DiagonalWrapper<VectorTypeDoublePrecision> D(Utils::toDouble(m_pcaVariance.topLeftCorner(nPCA)));
    Eigen::DiagonalWrapper<VectorTypeDoublePrecision> Dinv = D.inverse();
    Eigen::DiagonalWrapper<VectorTypeDoublePrecision>  D2 = D * D;
    Eigen::DiagonalWrapper<VectorTypeDoublePrecision>  D2inv = D2.inverse();

    for (unsigned i = 0; i < nIterations; i++) {

    	// E step
    	for (unsigned j = 0; j < Vinv.size(); j++) {
    		// student-t case
    		Vinv(j) = (nu + 1.0) / (nu * sigma2 + pow(y(j) -  f(j), 2));

    		// for later use
    		VinvSqrt(j) = sqrt(Vinv(j));
    	}

    	// M step
    	const MatrixTypeDoublePrecision W = VinvSqrt * U * D;
    	const MatrixTypeDoublePrecision WT = W.transpose();

    	const VectorTypeDoublePrecision outer_term = (Vinv * y);
    	const MatrixTypeDoublePrecision IWTWInv = vnl_matrix_inverse<double>(vnl_diag_matrix<double>(nPCA, 1) + WT * W);
    	const VectorTypeDoublePrecision fst_term = (U * (D2 * (UT * outer_term)));
    	const VectorTypeDoublePrecision snd_term = (U * (D2 * (UT * (Vinv * (U * (D2 * (UT * outer_term)))))));
    	const VectorTypeDoublePrecision trd_term = (U * (D2 * (UT * (VinvSqrt * (W * (IWTWInv * (WT * (VinvSqrt * (U * (D2 * (UT * outer_term)))))))))));
    	f = fst_term - snd_term + trd_term;
    }

    // the latent variable f is now a robust approximation for the data set. We return its coefficients.
    DatasetConstPointerType newds = m_representer->sampleToDataset(f);
    VectorType coeffs = GetCoefficientsForData(newds);
    Representer::DeleteDataset(newds);
    return coeffs;
    */
}




template <typename T>
float
StatisticalModel<T>::GetNoiseVariance() const {
    return m_noiseVariance;
}


template <typename T>
const VectorType&
StatisticalModel<T>::GetMeanVector() const {
    return m_mean;
}

template <typename T>
const VectorType&
StatisticalModel<T>::GetPCAVarianceVector() const {
    return m_pcaVariance;
}


template <typename T>
const MatrixType&
StatisticalModel<T>::GetPCABasisMatrix() const {
    return m_pcaBasisMatrix;
}

template <typename T>
MatrixType
StatisticalModel<T>::GetOrthonormalPCABasisMatrix() const {
    // we can recover the orthonormal matrix by undoing the scaling with the pcaVariance
    // (c.f. the method SetParameters)

    assert(m_pcaVariance.maxCoeff() > 1e-8);
    VectorType D = m_pcaVariance.array().sqrt();
    return m_pcaBasisMatrix * DiagMatrixType(D).inverse();
}



template <typename T>
void
StatisticalModel<T>::SetModelInfo(const ModelInfo& modelInfo) {
    m_modelInfo = modelInfo;
}


template <typename T>
const ModelInfo&
StatisticalModel<T>::GetModelInfo() const {
    return m_modelInfo;
}



template <typename T>
unsigned int
StatisticalModel<T>::GetNumberOfPrincipalComponents() const {
    return m_pcaBasisMatrix.cols();
}

template <typename T>
MatrixType
StatisticalModel<T>::GetJacobian(const PointType& pt) const {

    unsigned ptId = m_representer->GetPointIdForPoint(pt);

    return GetJacobian(ptId);
}

template <typename T>
MatrixType
StatisticalModel<T>::GetJacobian(unsigned ptId) const {

    unsigned Dimensions = m_representer->GetDimensions();
    MatrixType J = MatrixType::Zero(Dimensions, GetNumberOfPrincipalComponents());

    for(unsigned i = 0; i < Dimensions; i++) {
        unsigned idx = m_representer->MapPointIdToInternalIdx(ptId, i);
        for(unsigned j = 0; j < GetNumberOfPrincipalComponents(); j++) {
            J(i,j) += m_pcaBasisMatrix(idx,j) ;
        }
    }
    return J;
}


template <typename T>
StatisticalModel<T>*
StatisticalModel<T>::Load(Representer<T>* representer, const std::string& filename, unsigned maxNumberOfPCAComponents) {

    StatisticalModel* newModel = 0;

    H5::H5File file;
    try {
        file = H5::H5File(filename.c_str(), H5F_ACC_RDONLY);
    } catch (H5::Exception& e) {
        std::string msg(std::string("could not open HDF5 file \n") + e.getCDetailMsg());
        throw StatisticalModelException(msg.c_str());
    }

    H5::Group modelRoot = file.openGroup("/");

    newModel =  Load(representer, modelRoot, maxNumberOfPCAComponents);

    modelRoot.close();
    file.close();
    return newModel;

}


template <typename T>
StatisticalModel<T>*
StatisticalModel<T>::Load(Representer<T>* representer, const H5::Group& modelRoot, unsigned maxNumberOfPCAComponents) {

    StatisticalModel* newModel = 0;

    try {
        H5::Group representerGroup = modelRoot.openGroup("./representer");

        representer->Load(representerGroup);
        representerGroup.close();

        newModel = new StatisticalModel(representer);

        int minorVersion = 0;
        int majorVersion = 0;

        if (HDF5Utils::existsObjectWithName(modelRoot, "version") == false) {
            // this is an old statismo format, that had not been versioned. We set the version to 0.8 as this is the last version
            // that stores the old format
            std::cout << "Warning: version attribute does not exist in hdf5 file. Assuming version 0.8" <<std::endl;
            minorVersion = 8;
            majorVersion = 0;
        } else {
            H5::Group versionGroup = modelRoot.openGroup("./version");
            minorVersion = HDF5Utils::readInt(versionGroup, "./minorVersion");
            majorVersion = HDF5Utils::readInt(versionGroup, "./majorVersion");
        }

        H5::Group modelGroup = modelRoot.openGroup("./model");
        HDF5Utils::readVector(modelGroup, "./mean", newModel->m_mean);
        statismo::VectorType pcaVariance;
        HDF5Utils::readVector(modelGroup, "./pcaVariance", maxNumberOfPCAComponents, pcaVariance);
        newModel->m_pcaVariance = pcaVariance;

        // Depending on the statismo version, the pcaBasis matrix was stored as U*D or U (where U are the orthonormal PCA Basis functions and D the standard deviations).
        // Here we make sure that we fill the pcaBasisMatrix (which statismo stores as U*D) with the right values.
        if (majorVersion == 0 && minorVersion == 8) {
            HDF5Utils::readMatrix(modelGroup, "./pcaBasis", maxNumberOfPCAComponents, newModel->m_pcaBasisMatrix);
        } else if (majorVersion ==0 && minorVersion == 9) {
            MatrixType orthonormalPCABasis;
            HDF5Utils::readMatrix(modelGroup, "./pcaBasis", maxNumberOfPCAComponents, orthonormalPCABasis);

            VectorType D = pcaVariance.array().sqrt();
            newModel->m_pcaBasisMatrix = orthonormalPCABasis * DiagMatrixType(D);
        } else {
            std::ostringstream os;
            os << "an invalid statismo version was provided (" << majorVersion << "." << minorVersion << ")";
            throw StatisticalModelException(os.str().c_str());
        }
        newModel->m_noiseVariance = HDF5Utils::readFloat(modelGroup, "./noiseVariance");

        modelGroup.close();
        newModel->m_modelInfo.Load(modelRoot);

    } catch (H5::Exception& e) {
        std::string msg(std::string("an exeption occured while reading HDF5 file") +
                        "The most likely cause is that the hdf5 file does not contain the required objects. \n" + e.getCDetailMsg());
        throw StatisticalModelException(msg.c_str());
    }

    assert(newModel != 0);
    newModel->m_cachedValuesValid = false;

    newModel->m_modelLoaded = true;

    return newModel;
}

template <typename T>
void
StatisticalModel<T>::Save(const std::string& filename) const {
    using namespace H5;

    if (m_modelLoaded == true) {
        throw StatisticalModelException("Cannot save the model: Note, to prevent inconsistencies in the model's history, only models that have been newly created can be saved, and not those loaded from an hdf5 file.");
    }



    H5File file;
    std::ifstream ifile(filename.c_str());

    try {
        file = H5::H5File( filename.c_str(), H5F_ACC_TRUNC);
    } catch (H5::FileIException& e) {
        std::string msg(std::string("Could not open HDF5 file for writing \n") + e.getCDetailMsg());
        throw StatisticalModelException(msg.c_str());
    }


    H5::Group modelRoot = file.openGroup("/");

    H5::Group versionGroup = modelRoot.createGroup("version");
    HDF5Utils::writeInt(versionGroup, "majorVersion", 0);
    HDF5Utils::writeInt(versionGroup, "minorVersion", 9);
    versionGroup.close();

    Save(modelRoot);
    modelRoot.close();
    file.close();
}

template <typename T>
void
StatisticalModel<T>::Save(const H5::Group& modelRoot) const {

    try {
        // create the group structure

        std::string dataTypeStr = RepresenterType::TypeToString(m_representer->GetType());

        H5::Group representerGroup = modelRoot.createGroup("./representer");
        HDF5Utils::writeStringAttribute(representerGroup, "name", m_representer->GetName());
        HDF5Utils::writeStringAttribute(representerGroup, "version", m_representer->GetVersion());
        HDF5Utils::writeStringAttribute(representerGroup, "datasetType", dataTypeStr);

        this->m_representer->Save(representerGroup);
        representerGroup.close();

        H5::Group modelGroup = modelRoot.createGroup( "./model" );
        HDF5Utils::writeMatrix(modelGroup, "./pcaBasis", GetOrthonormalPCABasisMatrix());
        HDF5Utils::writeVector(modelGroup, "./pcaVariance", m_pcaVariance);
        HDF5Utils::writeVector(modelGroup, "./mean", m_mean);
        HDF5Utils::writeFloat(modelGroup, "./noiseVariance", m_noiseVariance);
        modelGroup.close();

        m_modelInfo.Save(modelRoot);


    } catch (H5::Exception& e) {
        std::string msg(std::string("an exception occurred while writing HDF5 file \n") + e.getCDetailMsg());
        throw StatisticalModelException(msg.c_str());
    }
}

template <typename T>
void
StatisticalModel<T>::CheckAndUpdateCachedParameters() const {

    if (m_cachedValuesValid == false) {
        VectorType I = VectorType::Ones(m_pcaBasisMatrix.cols());
        MatrixType Mmatrix = m_pcaBasisMatrix.transpose() * m_pcaBasisMatrix;
        Mmatrix.diagonal() += m_noiseVariance * I;

        m_MInverseMatrix = Mmatrix.inverse();

    }
    m_cachedValuesValid = true;
}

} // namespace statismo

#endif
