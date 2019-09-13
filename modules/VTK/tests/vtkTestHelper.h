#pragma once

#include "CommonTypes.h"

#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkStructuredPointsReader.h>
#include <vtkStructuredPoints.h>

#include <string>

namespace statismo::test
{
    inline vtkPolyData* loadPolyData(const std::string& filename) {
    vtkPolyDataReader* reader = vtkPolyDataReader::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    vtkPolyData* pd = vtkPolyData::New();
    pd->ShallowCopy(reader->GetOutput());
    reader->Delete();
    return pd;
}

inline void writePolyData(vtkPolyData* pd, const std::string& filename) {
    vtkSmartPointer< vtkPolyDataWriter > writer = vtkSmartPointer< vtkPolyDataWriter >::New();
    writer->SetFileName(filename.c_str());
    writer->SetInputData(pd);
    writer->Update();
}

inline vtkStructuredPoints* loadStructuredPoints(const std::string& filename) {
    vtkStructuredPointsReader* reader = vtkStructuredPointsReader::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    vtkStructuredPoints* pd = vtkStructuredPoints::New();
    pd->ShallowCopy(reader->GetOutput());
    reader->Delete();
    return pd;
}

inline vtkPolyData* ReducePoints(vtkPolyData* poly, unsigned num_points) {
  vtkPoints* points = vtkPoints::New();
  unsigned step = unsigned(std::ceil(double(poly->GetPoints()->GetNumberOfPoints())/double(num_points)));
  for(unsigned i=0; i< num_points; i++) {
    points->InsertNextPoint(poly->GetPoints()->GetPoint(i));
  }
  vtkPolyData* res = vtkPolyData::New();
  res->SetPoints(points);
  return res;
}

// https://github.com/statismo/statismo/pull/268/files
inline VectorType::Scalar CompareVectors(const VectorType& v1, const VectorType& v2) {
  VectorType v3 = v1 - v2;
  // permit some slack for small figures to account for numerical inaccuracies (commit b30e7dc1d3c994351d4e6790d071a957f3fb59e0)
  VectorType::Scalar graceForNumericalInaccuracy = v2.array().maxCoeff()*1e-5; 
  for (int i = 0; i < v3.size(); i++) {
	VectorType::Scalar blValue = v2[i];
	if (blValue != 0 || blValue != blValue) {
	  v3[i] = v3[i] / (abs(blValue) + graceForNumericalInaccuracy);
	}
  }
  return v3.array().abs().maxCoeff();
}

inline double CompareMatrices(const statismo::MatrixType& m1, const statismo::MatrixType& m2) {
    return (m1-m2).norm();
}

}