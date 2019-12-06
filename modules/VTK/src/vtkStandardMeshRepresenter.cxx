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

#include "statismo/VTK/vtkStandardMeshRepresenter.h"

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCharArray.h>
#include <vtkDataArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkLongArray.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyDataReader.h>
#include <vtkShortArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedLongArray.h>
#include <vtkUnsignedShortArray.h>

#include "statismo/core/HDF5Utils.h"
#include "statismo/core/Utils.h"

namespace statismo
{

vtkStandardMeshRepresenter::vtkStandardMeshRepresenter(DatasetConstPointerType reference)
  : vtkStandardMeshRepresenter()
{
  this->SetReference(reference);
}

vtkStandardMeshRepresenter::~vtkStandardMeshRepresenter() {}

vtkStandardMeshRepresenter *
vtkStandardMeshRepresenter::CloneImpl() const
{
  // this works since Create deep copies the reference
  return Create(m_reference);
}


void
vtkStandardMeshRepresenter::Load(const H5::Group & fg)
{
  vtkSmartPointer<vtkPolyData> ref; 

  std::string repName = hdf5utils::ReadStringAttribute(fg, "name");
  if (repName == "vtkPolyDataRepresenter" || repName == "itkMeshRepresenter")
  {
    ref = LoadRefLegacy(fg);
  }
  else
  {
    ref = LoadRef(fg);
  }

  this->SetReference(ref);
}


vtkStandardMeshRepresenter::DatasetPointerType
vtkStandardMeshRepresenter::LoadRef(const H5::Group & fg) const
{

  statismo::MatrixType vertexMat;
  hdf5utils::ReadMatrix(fg, "./points", vertexMat);

  typedef statismo::GenericEigenTraits<unsigned int>::MatrixType UIntMatrixType;
  UIntMatrixType                                                 cellsMat;
  hdf5utils::ReadMatrixOfType<unsigned int>(fg, "./cells", cellsMat);

  // create the reference from this information
  auto ref = DatasetPointerType::New();

  unsigned nVertices = vertexMat.cols();
  unsigned nCells = cellsMat.cols();

  auto pcoords = vtkSmartPointer<vtkFloatArray>::New();
  pcoords->SetNumberOfComponents(3);
  pcoords->SetNumberOfTuples(nVertices);
  for (unsigned i = 0; i < nVertices; i++)
  {
    pcoords->SetTuple3(i, vertexMat(0, i), vertexMat(1, i), vertexMat(2, i));
  }
  auto points = vtkSmartPointer<vtkPoints>::New();
  points->SetData(pcoords);

  ref->SetPoints(points);

  auto cell = vtkSmartPointer<vtkCellArray>::New();
  unsigned       cellDim = cellsMat.rows();
  for (unsigned i = 0; i < nCells; i++)
  {
    cell->InsertNextCell(cellDim);
    for (unsigned d = 0; d < cellDim; d++)
    {
      cell->InsertCellPoint(cellsMat(d, i));
    }
  }
  if (cellDim == 2)
  {
    ref->SetLines(cell);
  }
  else
  {
    ref->SetPolys(cell);
  }

  // read the point and cell data
  assert(ref->GetPointData() != 0);
  if (hdf5utils::ExistsObjectWithName(fg, "pointData"))
  {
    H5::Group pdGroup = fg.openGroup("./pointData");
    if (hdf5utils::ExistsObjectWithName(pdGroup, "scalars"))
    {
      ref->GetPointData()->SetScalars(GetAsDataArray(pdGroup, "scalars"));
    }
    if (hdf5utils::ExistsObjectWithName(pdGroup, "vectors"))
    {
      ref->GetPointData()->SetVectors(GetAsDataArray(pdGroup, "vectors"));
    }
    if (hdf5utils::ExistsObjectWithName(pdGroup, "normals"))
    {
      ref->GetPointData()->SetNormals(GetAsDataArray(pdGroup, "normals"));
    }
  }

  if (hdf5utils::ExistsObjectWithName(fg, "cellData"))
  {
    H5::Group cdGroup = fg.openGroup("./cellData");
    assert(ref->GetCellData() != 0);

    if (hdf5utils::ExistsObjectWithName(cdGroup, "scalars"))
    {
      ref->GetPointData()->SetScalars(GetAsDataArray(cdGroup, "scalars"));
    }
    if (hdf5utils::ExistsObjectWithName(cdGroup, "vectors"))
    {
      ref->GetPointData()->SetVectors(GetAsDataArray(cdGroup, "vectors"));
    }
    if (hdf5utils::ExistsObjectWithName(cdGroup, "normals"))
    {
      ref->GetPointData()->SetNormals(GetAsDataArray(cdGroup, "normals"));
    }
  }
  return ref;
}


vtkStandardMeshRepresenter::DatasetPointerType
vtkStandardMeshRepresenter::LoadRefLegacy(const H5::Group & fg) const
{
  std::string tmpfilename = statismo::utils::CreateTmpName(".vtk");

  hdf5utils::GetFileFromHDF5(fg, "./reference", tmpfilename.c_str());
  vtkNew<vtkPolyDataReader> reader;
  reader->SetFileName(tmpfilename.c_str());
  reader->Update();
  statismo::utils::RemoveFile(tmpfilename);
  if (reader->GetErrorCode() != 0)
  {
    throw StatisticalModelException((std::string("Could not read file ") + tmpfilename).c_str());
  }
  return reader->GetOutput();
}


void
vtkStandardMeshRepresenter::Save(const H5::Group & fg) const
{
  using namespace H5;


  statismo::MatrixType vertexMat = statismo::MatrixType::Zero(3, m_reference->GetNumberOfPoints());

  for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++)
  {
    PointType pt = m_reference->GetPoint(i);
    for (unsigned d = 0; d < 3; d++)
    {
      vertexMat(d, i) = pt[d];
    }
  }
  hdf5utils::WriteMatrix(fg, "./points", vertexMat);

  // check the dimensionality of a face (i.e. the number of points it has). We assume that
  // all the cells are the same.
  unsigned numPointsPerCell = 0;
  if (m_reference->GetNumberOfCells() > 0)
  {
    numPointsPerCell = m_reference->GetCell(0)->GetNumberOfPoints();
  }

  typedef statismo::GenericEigenTraits<unsigned int>::MatrixType UIntMatrixType;
  UIntMatrixType facesMat = UIntMatrixType::Zero(numPointsPerCell, m_reference->GetNumberOfCells());

  for (unsigned i = 0; i < m_reference->GetNumberOfCells(); i++)
  {
    vtkCell * cell = m_reference->GetCell(i);
    assert(numPointsPerCell == cell->GetNumberOfPoints());
    for (unsigned d = 0; d < numPointsPerCell; d++)
    {
      facesMat(d, i) = cell->GetPointIds()->GetId(d);
    }
  }

  hdf5utils::WriteMatrixOfType<unsigned int>(fg, "./cells", facesMat);

  H5::Group pdGroup = fg.createGroup("pointData");

  vtkPointData * pd = m_reference->GetPointData();
  if (pd != 0 && pd->GetScalars() != 0)
  {
    vtkDataArray * scalars = pd->GetScalars();
    WriteDataArray(pdGroup, "scalars", scalars);
  }
  if (pd != 0 && pd->GetVectors() != 0)
  {
    vtkDataArray * vectors = pd->GetVectors();
    WriteDataArray(pdGroup, "vectors", vectors);
  }
  if (pd != 0 && pd->GetNormals() != 0)
  {
    vtkDataArray * normals = pd->GetNormals();
    WriteDataArray(pdGroup, "normals", normals);
  }

  H5::Group     cdGroup = fg.createGroup("cellData");
  vtkCellData * cd = m_reference->GetCellData();
  if (cd != 0 && cd->GetScalars() != 0)
  {
    vtkDataArray * scalars = cd->GetScalars();
    WriteDataArray(cdGroup, "scalars", scalars);
  }
  if (cd != 0 && cd->GetVectors() != 0)
  {
    vtkDataArray * vectors = cd->GetVectors();
    WriteDataArray(cdGroup, "vectors", vectors);
  }
  if (cd != 0 && cd->GetNormals() != 0)
  {
    vtkDataArray * normals = cd->GetNormals();
    WriteDataArray(cdGroup, "normals", normals);
  }
}

statismo::VectorType
vtkStandardMeshRepresenter::PointToVector(const PointType & pt) const
{

  // a vtk point is always 3 dimensional
  const VectorType & v = Eigen::Map<const VectorTypeDoublePrecision>(pt.data(), 3).cast<float>();

  return v;
}


statismo::VectorType
vtkStandardMeshRepresenter::SampleToSampleVector(DatasetConstPointerType _sample) const
{
  assert(m_reference != 0);

  vtkPolyData * sample = const_cast<vtkPolyData *>(_sample);

  VectorType sampleVec = VectorType::Zero(m_reference->GetNumberOfPoints() * 3);
  // TODO make this more efficient using SetVoidArray of vtk
  for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++)
  {
    for (unsigned j = 0; j < 3; j++)
    {
      unsigned idx = MapPointIdToInternalIdx(i, j);
      sampleVec(idx) = sample->GetPoint(i)[j];
    }
  }
  return sampleVec;
}

vtkStandardMeshRepresenter::DatasetPointerType
vtkStandardMeshRepresenter::SampleVectorToSample(const VectorType & sample) const
{

  assert(m_reference != 0);

  vtkPolyData * reference = const_cast<vtkPolyData *>(m_reference.GetPointer());
  auto          pd = vtkSmartPointer<vtkPolyData>::New();
  pd->DeepCopy(reference);

  vtkPoints * points = pd->GetPoints();
  for (unsigned i = 0; i < reference->GetNumberOfPoints(); i++)
  {
    vtkPoint pt;
    for (unsigned d = 0; d < GetDimensions(); d++)
    {
      unsigned idx = MapPointIdToInternalIdx(i, d);
      pt[d] = sample(idx);
    }
    points->SetPoint(i, pt.data());
  }

  return pd;
}

vtkStandardMeshRepresenter::ValueType
vtkStandardMeshRepresenter::PointSampleFromSample(DatasetConstPointerType sample_, unsigned ptid) const
{
  vtkPolyData * sample = const_cast<vtkPolyData *>(sample_);
  if (ptid >= sample->GetNumberOfPoints())
  {
    throw StatisticalModelException("invalid ptid provided to PointSampleFromSample");
  }
  return vtkPoint(sample->GetPoints()->GetPoint(ptid));
}

statismo::VectorType
vtkStandardMeshRepresenter::PointSampleToPointSampleVector(const ValueType & v) const
{
  VectorType vec(GetDimensions());
  for (unsigned i = 0; i < GetDimensions(); i++)
  {
    vec(i) = v[i];
  }
  return vec;
}

vtkStandardMeshRepresenter::ValueType
vtkStandardMeshRepresenter::PointSampleVectorToPointSample(const VectorType & v) const
{
  ValueType value;
  for (unsigned i = 0; i < GetDimensions(); i++)
  {
    value[i] = v(i);
  }
  return value;
}


unsigned
vtkStandardMeshRepresenter::GetPointIdForPoint(const PointType & pt) const
{
  assert(m_reference != 0);
  return this->m_reference->FindPoint(const_cast<double *>(pt.data()));
}


unsigned
vtkStandardMeshRepresenter::GetNumberOfPoints() const
{
  assert(m_reference != 0);

  return this->m_reference->GetNumberOfPoints();
}

vtkSmartPointer<vtkDataArray>
vtkStandardMeshRepresenter::GetAsDataArray(const H5::Group & group, const std::string & name)
{
  vtkSmartPointer<vtkDataArray> dataArray;

  typedef statismo::GenericEigenTraits<double>::MatrixType DoubleMatrixType;
  DoubleMatrixType                                         m;
  hdf5utils::ReadMatrixOfType<double>(group, name.c_str(), m);

  // we open the dataset once more to be able to read its attribute
  H5::DataSet matrixDs = group.openDataSet(name.c_str());
  int         type = hdf5utils::ReadIntAttribute(matrixDs, "datatype");

  switch (type)
  {
    case statismo::UNSIGNED_CHAR:
      dataArray = vtkSmartPointer<vtkUnsignedCharArray>::New();
      break;
    case statismo::SIGNED_CHAR:
      dataArray = vtkSmartPointer<vtkCharArray>::New();
      break;
    case statismo::FLOAT:
      dataArray = vtkSmartPointer<vtkFloatArray>::New();
      break;
    case statismo::DOUBLE:
      dataArray = vtkSmartPointer<vtkDoubleArray>::New();
      break;
    case statismo::UNSIGNED_INT:
      dataArray = vtkSmartPointer<vtkUnsignedIntArray>::New();
      break;
    case statismo::SIGNED_INT:
      dataArray = vtkSmartPointer<vtkIntArray>::New();
      break;
    case statismo::UNSIGNED_SHORT:
      dataArray = vtkSmartPointer<vtkUnsignedShortArray>::New();
      break;
    case statismo::SIGNED_SHORT:
      dataArray = vtkSmartPointer<vtkShortArray>::New();
      break;
    case statismo::UNSIGNED_LONG:
      dataArray = vtkSmartPointer<vtkLongArray>::New();
      break;
    case statismo::SIGNED_LONG:
      dataArray = vtkSmartPointer<vtkUnsignedLongArray>::New();
      break;
    default:
      throw StatisticalModelException(
        "Unsupported data type for dataArray in vtkStandardMeshRepresenter::GetAsDataArray.");
  }
  FillDataArray(m, dataArray);
  return dataArray;
}


void
vtkStandardMeshRepresenter::FillDataArray(const statismo::GenericEigenTraits<double>::MatrixType & m,
                                          vtkDataArray *                                           dataArray)
{
  unsigned numComponents = m.rows();
  unsigned numTuples = m.cols();
  dataArray->SetNumberOfComponents(numComponents);
  dataArray->SetNumberOfTuples(numTuples);
  double * tuple = new double[numComponents];
  for (unsigned i = 0; i < numTuples; i++)
  {
    dataArray->InsertTuple(i, m.col(i).data());
  }
  delete[] tuple;
}


void
vtkStandardMeshRepresenter::SetReference(DatasetConstPointerType reference)
{
  // whta happens if m_refrnece is reference?
  // m_reference = DatasetPointerType::New();
  m_reference->DeepCopy(const_cast<vtkPolyData *>(reference));

  // set the domain
  DomainType::DomainPointsListType ptList;
  for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++)
  {
    double * d = m_reference->GetPoint(i);
    ptList.push_back(vtkPoint(d));
  }
  m_domain = DomainType(ptList);
}


void
vtkStandardMeshRepresenter::WriteDataArray(const H5::H5Location & group,
                                           const std::string &    name,
                                           const vtkDataArray *   _dataArray) const
{
  vtkDataArray *                                           dataArray = const_cast<vtkDataArray *>(_dataArray);
  unsigned                                                 numComponents = dataArray->GetNumberOfComponents();
  unsigned                                                 numTuples = dataArray->GetNumberOfTuples();
  typedef statismo::GenericEigenTraits<double>::MatrixType DoubleMatrixType;
  DoubleMatrixType                                         m = DoubleMatrixType::Zero(numComponents, numTuples);

  for (unsigned i = 0; i < numTuples; i++)
  {
    double * tuple = dataArray->GetTuple(i);
    for (unsigned d = 0; d < numComponents; d++)
    {
      m(d, i) = tuple[d];
    }
  }
  const H5::DataSet ds = hdf5utils::WriteMatrixOfType<double>(group, name.c_str(), m);

  int statismoDataTypeId = vtkHelper::vtkDataTypeIdToStatismoDataTypeId(dataArray->GetDataType());
  hdf5utils::WriteIntAttribute(ds, "datatype", statismoDataTypeId);
}

} // namespace statismo
