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
 * PROFITS; OR BUSINESS addINTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "StatismoUnitTest.h"
#include "statismo/core/Exceptions.h"

#include "statismo/core/DataManager.h"
#include "statismo/core/PCAModelBuilder.h"
#include "statismo/core/StatisticalModel.h"
#include "statismo/core/IO.h"
#include "statismo/core/TrivialVectorialRepresenter.h"

#include <memory>

namespace
{
int
Test1()
{
  using RepresenterType = statismo::TrivialVectorialRepresenter;
  using ModelBuilderType = statismo::PCAModelBuilder<statismo::VectorType>;
  using StatisticalModelType = statismo::StatisticalModel<statismo::VectorType>;
  using DataManagerType = statismo::BasicDataManager<statismo::VectorType>;

  const unsigned                   Dim = 3;
  std::unique_ptr<RepresenterType> representer(RepresenterType::Create(Dim));
  std::unique_ptr<DataManagerType> dataManager(DataManagerType::Create(representer.get()));

  // we create three simple datasets
  statismo::VectorType dataset1(Dim), dataset2(Dim), dataset3(Dim);
  dataset1 << 1, 0, 0;
  dataset2 << 0, 2, 0;
  dataset3 << 0, 0, 4;

  dataManager->AddDataset(dataset1, "dataset1");
  dataManager->AddDataset(dataset2, "dataset1");
  dataManager->AddDataset(dataset3, "dataset1");


  statismo::UniquePtrType<ModelBuilderType>     pcaModelBuilder(ModelBuilderType::Create());
  statismo::UniquePtrType<StatisticalModelType> model(pcaModelBuilder->BuildNewModel(dataManager->GetData(), 0.01));

  STATISMO_ASSERT_EQ(model->GetNumberOfPrincipalComponents(), 2U);

  statismo::IO<statismo::VectorType>::SaveStatisticalModel(model.get(), "test.h5");

  auto                                          newRepresenter = RepresenterType::SafeCreate();
  statismo::UniquePtrType<StatisticalModelType> loadedModel(
    statismo::IO<statismo::VectorType>::LoadStatisticalModel(newRepresenter.get(), "test.h5"));

  STATISMO_ASSERT_EQ(model->GetNumberOfPrincipalComponents(), loadedModel->GetNumberOfPrincipalComponents());

  return EXIT_SUCCESS;
}
} // namespace

/**
 * This basic test case covers the model creation pipeline and tests whether a model can be successfully
 * saved to disk. If the test runs correctly, it merely means that statismo has been setup correclty and hdf5
 * works.
 *
 * Real unit tests that test the functionality of statismo are provided in the statismoTests directory (these tests
 * require VTK to be installed and the statismo python wrapping to be working).
 */
int basicStatismoTest([[maybe_unused]] int argc, [[maybe_unused]] char * argv[])
{
  auto res = statismo::Translate([]() {
    return statismo::test::RunAllTests("basicStatismoTest", { { "Test1", Test1 } });
  });

  return !CheckResultAndAssert(res, EXIT_SUCCESS);
}
