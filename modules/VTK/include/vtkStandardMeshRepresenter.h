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


#ifndef VTK_STANDARD_MESH_REPRESENTER_H_
#define VTK_STANDARD_MESH_REPRESENTER_H_

#include <itk_H5Cpp.h>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "CommonTypes.h"
#include "Domain.h"
#include "Representer.h"
#include "vtkHelper.h"

namespace statismo {


template <>
struct RepresenterTraits<vtkPolyData> {
    typedef vtkPolyData* DatasetPointerType;
    typedef const vtkPolyData* DatasetConstPointerType;

    typedef vtkPoint PointType;
    typedef vtkPoint ValueType;

    ///@}


};



/**
 * \brief A representer for vtkPolyData, which stores the represnter data in the standard
 * mesh format defined for statismo.
 *
 * See Representer for more details about representer classes
 * \sa Representer
 */
class vtkStandardMeshRepresenter : public Representer<vtkPolyData> {
  public:


    static vtkStandardMeshRepresenter* Create() {
        return new vtkStandardMeshRepresenter();
    }

    static vtkStandardMeshRepresenter* Create(const vtkPolyData* reference) {
        return new vtkStandardMeshRepresenter(reference);
    }


    void Load(const H5::Group& fg);

    vtkStandardMeshRepresenter* Clone() const;
    void Delete() const {
        delete this;
    }

    virtual ~vtkStandardMeshRepresenter();


    std::string GetName() const {
        return "vtkStandardMeshRepresenter";
    }
    unsigned GetDimensions() const {
        return 3;
    }
    std::string GetVersion() const {
        return "1.0" ;
    }
    RepresenterDataType GetType() const {
        return POLYGON_MESH;
    }
    const DomainType& GetDomain() const {
        return m_domain;
    }

    void DeleteDataset(DatasetPointerType d) const {
        d->Delete();
    };

    DatasetPointerType CloneDataset(DatasetConstPointerType d) const {
        vtkPolyData* clone = vtkPolyData::New();
        clone->DeepCopy(const_cast<vtkPolyData*>(d));
        return clone;
    }

    DatasetConstPointerType GetReference() const {
        return m_reference;
    }
    statismo::VectorType PointToVector(const PointType& pt) const;
    statismo::VectorType SampleToSampleVector(DatasetConstPointerType sample) const;
    DatasetPointerType SampleVectorToSample(const statismo::VectorType& sample) const;

    ValueType PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const;
    statismo::VectorType PointSampleToPointSampleVector(const ValueType& v) const;
    ValueType PointSampleVectorToPointSample(const statismo::VectorType& pointSample) const;


    void Save(const H5::Group& fg) const;
    unsigned GetNumberOfPoints() const;
    unsigned GetPointIdForPoint(const PointType& point) const;


  private:

    vtkStandardMeshRepresenter() : m_reference(0) {}
    vtkStandardMeshRepresenter(const std::string& reference);
    vtkStandardMeshRepresenter(const DatasetConstPointerType reference);
    vtkStandardMeshRepresenter(const vtkStandardMeshRepresenter& orig);
    vtkStandardMeshRepresenter& operator=(const vtkStandardMeshRepresenter& rhs);

    void SetReference(const vtkPolyData* reference);

    vtkPolyData* LoadRefLegacy(const H5::Group& fg) const;
    vtkPolyData* LoadRef(const H5::Group& fg) const;

    void WriteDataArray(const H5::CommonFG& group,  const std::string& name, const vtkDataArray* ds) const;
    static vtkDataArray* GetAsDataArray(const H5::Group& group,  const std::string& name);
    static void FillDataArray(const statismo::GenericEigenType<double>::MatrixType& m, vtkDataArray* dataArray);
    DatasetPointerType m_reference;

    DomainType m_domain;
};

} // namespace statismo

#endif /* VTK_STANDARD_MESH_REPRESENTER_H_ */
