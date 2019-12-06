#ifndef __STATISMO_VTK_TEST_HELPER_H_
#define __STATISMO_VTK_TEST_HELPER_H_

#include "statismo/core/CommonTypes.h"

#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkStructuredPointsReader.h>
#include <vtkStructuredPoints.h>
#include <vtkSmartPointer.h>

#include <string>

namespace statismo::test
{
inline vtkSmartPointer<vtkPolyData>
LoadPolyData(const std::string & filename)
{
  vtkNew<vtkPolyDataReader> reader;
  reader->SetFileName(filename.c_str());
  reader->Update();
  return reader->GetOutput();
}

inline void
WritePolyData(const vtkSmartPointer<vtkPolyData> & pd, const std::string & filename)
{
  vtkNew<vtkPolyDataWriter> writer;
  writer->SetFileName(filename.c_str());
  writer->SetInputData(pd);
  writer->Update();
}

inline vtkSmartPointer<vtkStructuredPoints>
LoadStructuredPoints(const std::string & filename)
{
  vtkNew<vtkStructuredPointsReader> reader;
  reader->SetFileName(filename.c_str());
  reader->Update();
  return reader->GetOutput();
}

inline vtkSmartPointer<vtkPolyData>
ReducePoints(const vtkSmartPointer<vtkPolyData> & poly, unsigned pointsCount)
{
  auto     points = vtkSmartPointer<vtkPoints>::New();
  unsigned step = 1 + poly->GetPoints()->GetNumberOfPoints() / pointsCount;
  for (unsigned i = 0; i < poly->GetPoints()->GetNumberOfPoints(); i += step)
  {
    points->InsertNextPoint(poly->GetPoints()->GetPoint(i));
  }
  auto res = vtkSmartPointer<vtkPolyData>::New();
  res->SetPoints(points);

  return res;
}

// https://github.com/statismo/statismo/pull/268/files
inline VectorType::Scalar
CompareVectors(const VectorType & v1, const VectorType & v2)
{
  VectorType v3 = v1 - v2;
  // permit some slack for small figures to account for numerical inaccuracies (commit
  // b30e7dc1d3c994351d4e6790d071a957f3fb59e0)
  VectorType::Scalar graceForNumericalInaccuracy = v2.array().maxCoeff() * 1e-5;
  for (int i = 0; i < v3.size(); i++)
  {
    VectorType::Scalar blValue = v2[i];
    if (blValue != 0 || blValue != blValue)
    {
      v3[i] = v3[i] / (abs(blValue) + graceForNumericalInaccuracy);
    }
  }
  return v3.array().abs().maxCoeff();
}

inline double
CompareMatrices(const statismo::MatrixType & m1, const statismo::MatrixType & m2)
{
  return (m1 - m2).norm();
}

} // namespace statismo::test

#endif