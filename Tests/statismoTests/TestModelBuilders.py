#
# This file is part of the statismo library.
#
# Author: Marcel Luethi (marcel.luethi@unibas.ch)
#
# Copyright (c) 2011 University of Basel
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# Neither the name of the project's author nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
import unittest
from os import listdir
from os.path import join
from scipy import zeros, randn, log, isnan, any, sqrt

import statismo

from statismoTestUtils import getDataFiles, DATADIR, getPDPointWithId, read_vtkpd
import tempfile


class Test(unittest.TestCase):
    
    def setUp(self):

        self.datafiles = getDataFiles(DATADIR)
        ref = read_vtkpd(self.datafiles[0])
        self.representer = statismo.vtkPolyDataRepresenter.Create(ref, statismo.vtkPolyDataRepresenter.RIGID)        
        self.dataManager = statismo.DataManager_vtkPD.Create(self.representer)
        
        datasets = map(read_vtkpd, self.datafiles)
        for (dataset, filename) in zip(datasets, self.datafiles):        
            self.dataManager.AddDataset(dataset, filename)

        
    def tearDown(self):
        pass

    def checkPointsAlmostEqual(self, pts1, pts2, numPoints, noise):
        step =  pts1.GetNumberOfPoints() / numPoints
        for i in xrange(0, pts1.GetNumberOfPoints(), step ):
            self.assertTrue(abs(pts1.GetPoint(i)[0] - pts2.GetPoint(i)[0]) <= max(sqrt(noise), 1e-2))
            self.assertTrue(abs(pts1.GetPoint(i)[1] - pts2.GetPoint(i)[1]) <= max(sqrt(noise), 1e-2))
            self.assertTrue(abs(pts1.GetPoint(i)[2] - pts2.GetPoint(i)[2]) <= max(sqrt(noise), 1e-2))
        
    def buildAndTestPCAModel(self, noise):
        modelbuilder = statismo.PCAModelBuilder_vtkPD.Create()
 
        model = modelbuilder.BuildNewModel(self.dataManager.GetSampleDataStructure(), noise)
                        
        self.assertTrue(model.GetNumberOfPrincipalComponents() <= len(self.datafiles))        
        
        # we cannot have negative eigenvalues
        self.assertTrue((model.GetPCAVarianceVector() >= 0).all() == True)
        self.assertTrue(isnan(model.GetPCAVarianceVector()).any() == False) 

        # we project a dataset into the model and try to restore it.
  
        samples = self.dataManager.GetSampleDataStructure()
        sample = samples[0].GetSample()
        
        coeffs_sample = model.ComputeCoefficientsForDataset(sample)
        restored_sample = model.DrawSample(coeffs_sample)

        self.assertEqual(sample.GetNumberOfPoints(), restored_sample.GetNumberOfPoints())

        self.checkPointsAlmostEqual(sample.GetPoints(), restored_sample.GetPoints(), 100, noise)

        # check if the scores can be used to restore the data in the datamanager
        scores = model.GetModelInfo().GetScoresMatrix()
        for i in xrange(0, scores.shape[1]):
            sample_from_scores = model.DrawSample(scores[:,i])
            sample_from_dm = samples[i].GetSample()

            self.checkPointsAlmostEqual(sample_from_scores.GetPoints(), sample_from_dm.GetPoints(), 100, noise)
        return model

    def testBuildPCAModelWithoutScores(self):
      
        # check if a model can be build when there are no scores
        modelbuilder = statismo.PCAModelBuilder_vtkPD.Create()
 
        model = modelbuilder.BuildNewModel(self.dataManager.GetSampleDataStructure(), 0, False)
                        
        self.assertTrue(model.GetNumberOfPrincipalComponents() <= len(self.datafiles))                

        # we cannot have negative eigenvalues
        self.assertTrue((model.GetPCAVarianceVector() >= 0).all() == True)
        
        # check if the scores can be used to restore the data in the datamanager
        scores = model.GetModelInfo().GetScoresMatrix()
        self.assertTrue (scores.shape[0] == 0 and scores.shape[1] == 0)


        

    def testBuildPCAModelZeroNoise(self):
        model = self.buildAndTestPCAModel(0)
        self.assertAlmostEqual(model.GetNoiseVariance(), 0)
        
    def testBuildPCAModelNonZeroNoise(self):
        model = self.buildAndTestPCAModel(0.1)
        self.assertAlmostEqual(model.GetNoiseVariance(), 0.1)
        
    def testBuildPCAModelWithLargeNoise(self):
        model = self.buildAndTestPCAModel(1000)
        self.assertAlmostEqual(model.GetNoiseVariance(), 1000)
        

        
        
    def testCheckPartiallyFixedModelMean(self):
        # if we fix many points to correspond to one of the samples, and build a 
        # partiallyfixed model, its mean should correspond to the sample
        nPointsFixed = 100
        nPointsTest = 1000        
        
        sample = self.dataManager.GetSampleDataStructure()[0].GetSample()        

        pvList = statismo.PointValueList_vtkPD()        

        reference = self.representer.GetReference()
        domainPoints = self.representer.GetDomain().GetDomainPoints()
        
        for pt_id in xrange(0, len(domainPoints), len(domainPoints) / nPointsFixed):
            fixed_pt = domainPoints[pt_id]
            value = statismo.vtkPoint(*getPDPointWithId(sample, pt_id))
            pointValue = statismo.PointValuePair_vtkPD(fixed_pt, value)
            pvList.append(pointValue)

        pfmodelbuilder = statismo.PartiallyFixedModelBuilder_vtkPD.Create()
        pf_model = pfmodelbuilder.BuildNewModel(self.dataManager.GetSampleDataStructure(), pvList, 0.1, 0.1)

        
        partial_mean = pf_model.DrawMean()

        # now the sample that we used to fix the point should be similar to the mean. We test it by  
        for pt_id in xrange(0, sample.GetNumberOfPoints(), sample.GetNumberOfPoints() / nPointsTest):
            mean_pt = getPDPointWithId(partial_mean, pt_id)
            sample_pt = getPDPointWithId(sample, pt_id)
            self.assertAlmostEqual(mean_pt[0], sample_pt[0], 0) 
            self.assertAlmostEqual(mean_pt[1], sample_pt[1], 0)
            self.assertAlmostEqual(mean_pt[2], sample_pt[2], 0)
 
        
    def testCheckPartiallyFixedModelWithoutConstraints(self):
        # if we fix no point, it should be the same as building a normal pca model                

        pvList = statismo.PointValueList_vtkPD()        
            
        pfmodelbuilder = statismo.PartiallyFixedModelBuilder_vtkPD.Create()
        pf_model = pfmodelbuilder.BuildNewModel(self.dataManager.GetSampleDataStructure(), pvList, 0.1, 0.1)

        pcamodelbuilder = statismo.PCAModelBuilder_vtkPD.Create()
        pca_model = pcamodelbuilder.BuildNewModel(self.dataManager.GetSampleDataStructure(), 0.1)
        
        sample  =  self.dataManager.GetSampleDataStructure()[0].GetSample()
        coeffs_pf_model = pf_model.ComputeCoefficientsForDataset(sample)
        coeffs_pca_model =   pca_model.ComputeCoefficientsForDataset(sample)
        for i in xrange(0, len(coeffs_pf_model)):
            # the sign is allowed to change
            self.assertAlmostEqual(abs(coeffs_pf_model[i]), abs(coeffs_pca_model[i]), 1)
                
    def testCheckPartiallyFixedModelVariancePlausibility(self):         
        # checks whether with every added point, the variance is decreasing
                       
        reference = self.representer.GetReference()
        sample  =  self.dataManager.GetSampleDataStructure()[0].GetSample()
        num_points = sample.GetNumberOfPoints()
        pvList = statismo.PointValueList_vtkPD()
        
        pfmodelbuilder = statismo.PartiallyFixedModelBuilder_vtkPD.Create()
        pf_model = pfmodelbuilder.BuildNewModel(self.dataManager.GetSampleDataStructure(), pvList, 0.1, 0.1)
        total_var = pf_model.GetPCAVarianceVector().sum() 
        for pt_id in xrange(0, num_points, num_points / 10):
            ref_pt = statismo.vtkPoint(*getPDPointWithId(reference, pt_id))
            pt = statismo.vtkPoint(*getPDPointWithId(sample, pt_id))
            pvList.append(statismo.PointValuePair_vtkPD(ref_pt, pt))
            pfmodelbuilder = statismo.PartiallyFixedModelBuilder_vtkPD.Create()
            pf_model = pfmodelbuilder.BuildNewModel(self.dataManager.GetSampleDataStructure(), pvList, 0.1, 0.1)
            total_sdev_prev = total_var
            total_var = pf_model.GetPCAVarianceVector().sum() 
            self.assertTrue(total_var < total_sdev_prev)


    def testPartiallyFixedModelPointStaysPut(self):         
        #Checks if a point that is fixed really stays where it was constrained to stay
                       
        reference = self.representer.GetReference()
        sample  =  self.dataManager.GetSampleDataStructure()[0].GetSample()
        pvList = statismo.PointValueList_vtkPD()
                
        ref_pt = getPDPointWithId(reference, 0)
        fixedpt = getPDPointWithId(sample, 0)
        pvList.append(statismo.PointValuePair_vtkPD(statismo.vtkPoint(*ref_pt),  statismo.vtkPoint(*fixedpt)))
        pfmodelbuilder = statismo.PartiallyFixedModelBuilder_vtkPD.Create()
        pf_model = pfmodelbuilder.BuildNewModel(self.dataManager.GetSampleDataStructure(), pvList, 0.01, 0.01)
        
        # check for some samples if the points stay put
        coeffs1 = zeros(pf_model.GetNumberOfPrincipalComponents())        
        coeffs1[1] = 3
        coeffs2 = zeros(pf_model.GetNumberOfPrincipalComponents())
        coeffs2[0] = -3        
        
        for coeffs in [coeffs1, coeffs2]:
            partiallyFixedSample = pf_model.DrawSample(coeffs)
            self.assertAlmostEqual(partiallyFixedSample.GetPoints().GetPoint(0)[0], fixedpt[0], 1)
            self.assertAlmostEqual(partiallyFixedSample.GetPoints().GetPoint(0)[1], fixedpt[1], 1)
            self.assertAlmostEqual(partiallyFixedSample.GetPoints().GetPoint(0)[2], fixedpt[2], 1)

    def testReducedVarianceModelBuilderCorrectlyReducesTotalVariance(self):
        modelbuilder = statismo.PCAModelBuilder_vtkPD.Create()
 
        model = modelbuilder.BuildNewModel(self.dataManager.GetSampleDataStructure(), 0.)
        reducedVarianceModelBuilder = statismo.ReducedVarianceModelBuilder_vtkPD.Create()

        for totalVariance in [1.0, 0.8, 0.6, 0.4, 0.2, 0]:
            reducedModel = reducedVarianceModelBuilder.BuildNewModelFromModel(model, totalVariance)

            # we keep at least the required percentage of total variance
            self.assertTrue(reducedModel.GetPCAVarianceVector().sum() >= totalVariance * model.GetPCAVarianceVector().sum())

        # check that there is a reduction (though we cannot say how much, as the specified variance is a lower bound)
        reducedModel05 = reducedVarianceModelBuilder.BuildNewModelFromModel(model, 0.5)
        self.assertTrue(reducedModel05.GetPCAVarianceVector().sum() <= model.GetPCAVarianceVector().sum())        

suite = unittest.TestLoader().loadTestsFromTestCase(Test)
            
if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()

