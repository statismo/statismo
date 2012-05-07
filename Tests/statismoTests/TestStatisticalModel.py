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
from scipy import zeros, randn, log

import statismo

from statismoTestUtils import buildPolyDataModel, DATADIR, getPDPointWithId
import tempfile



class Test(unittest.TestCase):
    
    def setUp(self):
        self.model = buildPolyDataModel(DATADIR, 0)

    def tearDown(self):
        pass

    def modelWithLittleNoiseAlmostEqualsModelWithNoNoise(self):
        """ test whether the ppca """
        modelNoNoise =  buildPolyDataModel(DATADIR,  0)
        modelLittleNoise = buildPolyDataModel(DATADIR, 1e-8)


    def testConversionFromSampleVectorToSample(self):
        
        # check whether the mechanism for vectors is ok in the first place
        coeffs = zeros(self.model.GetNumberOfPrincipalComponents())
        self.assertTrue((self.model.DrawSampleVector(coeffs) == self.model.GetMeanVector()).all() == True) 

        # now we check that after drawing a sample the equality still holds
        sample = self.model.DrawSample(coeffs)
        samplePts = sample.GetPoints()
        sampleVec = self.model.DrawSampleVector(coeffs)
        for pt_id in xrange(0, sample.GetNumberOfPoints()):            
            self.assertTrue(samplePts.GetPoint(pt_id)[0] == sampleVec[pt_id * 3] and 
                             samplePts.GetPoint(pt_id)[1] == sampleVec[pt_id * 3 + 1] and
                             samplePts.GetPoint(pt_id)[2] == sampleVec[pt_id * 3 + 2])      
            
            
    def testSampleEqualsSampleAtPoint(self): 
        """ test for a number of points whether DrawSampleAtPoints yields the same  
            values as DrawSample, evaluated at the given points"""
                      
        #choose arbitrary coefficients                       
        coeffs = zeros(self.model.GetNumberOfPrincipalComponents())
        coeffs[0] = -1
        coeffs[1] = 1
        
        sample = self.model.DrawSample(coeffs)
        num_pts = sample.GetNumberOfPoints()
        pt_ids = xrange(0, num_pts, num_pts / 10)        
        for ptid in pt_ids:
            sampleAtPt = self.model.DrawSampleAtPointId(coeffs, ptid)
            self.assertTrue(sampleAtPt[0] == sample.GetPoints().GetPoint(ptid)[0] and 
                            sampleAtPt[1] == sample.GetPoints().GetPoint(ptid)[1] and
                            sampleAtPt[2] == sample.GetPoints().GetPoint(ptid)[2])
            
    
    
    def testLoadSave(self):
        """ test whether saving and loading a model restores the model correctly """
        tmpfile = tempfile.mktemp(suffix="h5")
        self.model.Save(tmpfile)
        
        newModel = statismo.StatisticalModel_vtkPD.Load(tmpfile)
        
        self.assertTrue((self.model.GetPCAVarianceVector() == newModel.GetPCAVarianceVector()).all())
        self.assertTrue((self.model.GetMeanVector() == newModel.GetMeanVector()).all())
        self.assertTrue((self.model.GetPCABasisMatrix() == newModel.GetPCABasisMatrix()).all())        
        
        # check model info
        scores = self.model.GetModelInfo().GetScoresMatrix()
        newScores = newModel.GetModelInfo().GetScoresMatrix()

        di = self.model.GetModelInfo().GetDataInfo()
        newDi = newModel.GetModelInfo().GetDataInfo()
        self.assertEqual(len(di), len(newDi))
        
        for (sortedDi, sortedNewDi) in zip(sorted(di), sorted(newDi)):
            self.assertEqual(sortedDi[0], sortedNewDi[0])
            self.assertEqual(sortedDi[1], sortedNewDi[1])

        
        self.assertTrue(((scores - newScores) < 1e-3).all())
 
        # now we test if loading with less parameters loads the right submatrix
        newModelSub = statismo.StatisticalModel_vtkPD.Load(tmpfile, 2)
        self.assertEqual(newModelSub.GetNumberOfPrincipalComponents(), 2)
        self.assertTrue((newModelSub.GetPCAVarianceVector()[0:1] == self.model.GetPCAVarianceVector()[0:1]).all)
        self.assertTrue((newModelSub.GetPCABasisMatrix()[:,0:1] == self.model.GetPCABasisMatrix()[:,0:1]).all())


    def testLoadSaveNonstandardLocation(self):
        """ test whether saving and loading a model restores the model correctly, when a non-standard location is used"""
        tmpfile = tempfile.mktemp(suffix="h5")

        self.model.Save(tmpfile, "/model1")
        #self.model.Save(tmpfile, "./model1")        
        self.model.Save(tmpfile, "/model2")
        newModel1 = statismo.StatisticalModel_vtkPD.Load(tmpfile, "/model1")
        newModel2 = statismo.StatisticalModel_vtkPD.Load(tmpfile, "/model2")

        # we only run minimal tests, as the basic load save functionality is tested in an individual test
        self.assertEqual(newModel1.GetNumberOfPrincipalComponents(), self.model.GetNumberOfPrincipalComponents())
        self.assertEqual(newModel2.GetNumberOfPrincipalComponents(), self.model.GetNumberOfPrincipalComponents())


    def testDatasetToSample(self):
        """ Checks wheter DatasetToSample applied to a Sample returns the same sample """
        
        meanSample = self.model.DrawMean()
        meanSampleNew = self.model.DatasetToSample(meanSample)

        
        num_pts = meanSample.GetNumberOfPoints()
        pt_ids = xrange(0, num_pts, num_pts / 10) 

        for ptId in pt_ids:
            mp = meanSample.GetPoints().GetPoint(ptId)
            mpn = meanSampleNew.GetPoints().GetPoint(ptId)
            self.assertTrue((mp[0] ==  mpn[0]) and  (mp[1] == mpn[1]) and (mp[2] == mpn[2]))
        
        
    
    def testProbabilityOfDatasetPlausibility(self):
        # do a monte carlo sampling and see whether the intergral over the distribution
        # evaluates roughly to 1
        num_samples = 100

        p_mean = self.model.ComputeProbabilityOfDataset(self.model.DrawMean())
        for i in xrange(0, num_samples):
            coeffs = randn(self.model.GetNumberOfPrincipalComponents())
            s = self.model.DrawSample(coeffs)
            p = self.model.ComputeProbabilityOfDataset(s)
            self.assertTrue(p < p_mean, "Probability cannot be larger than the probabliity of the mean")
            self.assertTrue(p > 0, "Probablity must be positive")
            self.assertTrue(p < 1, "Probability must be smaller 1")
        
    def testLogProbabilityOfDatasetPlausibility(self):

        num_samples = 100

        for i in xrange(0, num_samples):
            coeffs = randn(self.model.GetNumberOfPrincipalComponents())
            s = self.model.DrawSample(coeffs)
            p = self.model.ComputeProbabilityOfDataset(s)
            lp = self.model.ComputeLogProbabilityOfDataset(s)
            self.assertTrue(log(p) -lp < 0.05, "Log probability should roughtly equal the log of the probability")
        
        
    def testCoefficientsGenerateCorrectDataset(self):
        coeffs = randn(self.model.GetNumberOfPrincipalComponents())
        s = self.model.DrawSample(coeffs)
        computed_coeffs = self.model.ComputeCoefficientsForDataset(s)
        diff = (coeffs - computed_coeffs)[:-1] # we ignore the last coefficient
        self.assertTrue((diff < 1e-3).all()) #don't make the threshold too small, as we are dealing with floats
             
             
    def testCoefficientsForPointValues(self):
        coeffs = randn(self.model.GetNumberOfPrincipalComponents())
        s = self.model.DrawSample(coeffs)
                
        num_pts = s.GetNumberOfPoints()
        pt_ids = xrange(0, num_pts, num_pts / 500) 
 
        pvList = statismo.PointValueList_vtkPD()
        pidVList = statismo.PointIdValueList_vtkPD()
        for pt_id in pt_ids:
            
            # we create a list of fixed points once using the point id ....
            val = statismo.vtkPoint(*getPDPointWithId(s, pt_id))            
            pidVPair = statismo.PointIdValuePair_vtkPD(pt_id, val)
            pidVList.append(pidVPair)
            
            # ... and once more with the points
            representer = self.model.GetRepresenter()
            ref_pt = statismo.vtkPoint(*getPDPointWithId(representer.GetReference(), pt_id)) 
            pvPair = statismo.PointValuePair_vtkPD(ref_pt, val)
            pvList.append(pvPair)
            
        computed_coeffs_ptids = self.model.ComputeCoefficientsForPointValues(pidVList)                        
        computed_coeffs_pts = self.model.ComputeCoefficientsForPointValues(pvList)
        
        # does the list with the point and the one with the point ids yield the same result
        self.assertTrue((computed_coeffs_ptids == computed_coeffs_pts).all())

        # now compare it to the real coefficients
        diff = (coeffs - computed_coeffs_pts)[:-1] # we ignore the last coefficient
        self.assertTrue((diff < 1e-3).all())                
        

         
        
    
    
    def testInternalMatrixDimensionalities(self):
        representer = self.model.GetRepresenter()
        num_points = representer.GetReference().GetNumberOfPoints()
        dim = statismo.vtkPolyDataRepresenter.GetDimensions()
        pcaBasisMatrix= self.model.GetPCABasisMatrix()
        self.assertEqual(pcaBasisMatrix.shape[0], num_points * dim )
        self.assertEqual(pcaBasisMatrix.shape[1], self.model.GetNumberOfPrincipalComponents())   
        
        meanVec = self.model.GetMeanVector()
        self.assertEqual(meanVec.shape[0], num_points * dim)

suite = unittest.TestLoader().loadTestsFromTestCase(Test)
                
if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
    print "done"
