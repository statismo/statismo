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

#ifndef VTKSTANDARDIMAGERESENTER_H_
#define VTKSTANDARDIMAGERESENTER_H_

#include <itk_H5Cpp.h>

#include <vtkSmartPointer.h>
#include <vtkStructuredPoints.h>

#include "CommonTypes.h"
#include "Domain.h"
#include "Representer.h"
#include "vtkHelper.h"


/**
 * \brief Representer class for vtkStructuredPoints of arbitrary scalar type and PixelDimension
 *
 * The pixel values used for the shape model are stored in the vtkPointData object of the vtkStructuredPoints.
 * They can be either scalars (cf. sp->GetPointData()->GetScalars()) or vectors (cf. sp->GetPointData()->GetVectors()).
 *
 * If you supply vtkStructuredPoints images with several arrays (e.g. scalars, vectors, tensors etc.), you need to ensure that
 * the array relevant to the shape model is the first array, i.e. the one returned by sp->GetPointData()->GetArray(0).
 *
 * \sa Representer
 */

namespace statismo {
template <>
struct RepresenterTraits<vtkStructuredPoints> {
    typedef vtkStructuredPoints* DatasetPointerType;
    typedef const vtkStructuredPoints* DatasetConstPointerType;

    typedef vtkPoint PointType;
    typedef vtkNDPixel ValueType;
    ///@}


};



template <class TScalar, unsigned PixelDimensions>
class vtkStandardImageRepresenter  : public Representer<vtkStructuredPoints> {
  public:



    static vtkStandardImageRepresenter* Create(const vtkStructuredPoints* reference) {
        return new vtkStandardImageRepresenter(reference);
    }
    static vtkStandardImageRepresenter* Create() {
        return new vtkStandardImageRepresenter();
    }

    void Load(const H5::Group& fg);
    vtkStandardImageRepresenter* Clone() const;

    virtual ~vtkStandardImageRepresenter();
    void Delete() const {
        delete this;
    }

    void DeleteDataset(DatasetPointerType d) const {
        d->Delete();
    };

    DatasetPointerType CloneDataset(DatasetConstPointerType d) const {
        vtkStructuredPoints* clone = vtkStructuredPoints::New();
        clone->DeepCopy(const_cast<vtkStructuredPoints*>(d));
        return clone;
    }


    unsigned GetDimensions() const {
        return  PixelDimensions;
    }
    std::string GetVersion() const {
        return "1.0" ;
    }
    RepresenterDataType GetType() const {
        return IMAGE;
    }
    const DomainType& GetDomain() const  {
        return m_domain;
    }

    std::string GetName() const {
        return "vtkStandardImageRepresenter";
    }

    const vtkStructuredPoints* GetReference() const {
        return m_reference;
    }

    statismo::VectorType PointToVector(const PointType& pt) const;
    statismo::VectorType SampleToSampleVector(DatasetConstPointerType sample) const;
    DatasetPointerType SampleVectorToSample(const statismo::VectorType& sample) const;


    ValueType PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const;
    statismo::VectorType PointSampleToPointSampleVector(const ValueType& v) const;
    ValueType PointSampleVectorToPointSample(const statismo::VectorType& samplePoint) const;

    unsigned GetPointIdForPoint(const PointType& pt) const;

    unsigned GetNumberOfPoints() const;
    void Save(const H5::Group& fg) const;

    static unsigned GetNumberOfPoints(DatasetPointerType  reference);


  private:

    vtkStructuredPoints* LoadRefLegacy(const H5::Group& fg) const;
    vtkStructuredPoints* LoadRef(const H5::Group& fg) const;

    vtkStandardImageRepresenter(DatasetConstPointerType reference);
    vtkStandardImageRepresenter() : m_reference(0) {}
    void SetReference(DatasetConstPointerType reference);

    vtkStructuredPoints* m_reference;
    DomainType m_domain;
};

} // namespace statismo

#include "vtkStandardImageRepresenter.hxx"

#endif /* VTKStandardImageRepRESENTER_H_ */
