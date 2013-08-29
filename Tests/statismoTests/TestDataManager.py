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
import tempfile
import os.path
import vtk

import statismo
from statismoTestUtils import DATADIR, getDataFiles, read_vtkpd

class Test(unittest.TestCase):

    def setUp(self):
        self.datafiles = getDataFiles(DATADIR)
        ref = read_vtkpd(self.datafiles[0])
            
        self.representer = statismo.vtkStandardMeshRepresenter.Create(ref)        

    def tearDown(self):
        pass

    def testName(self):
        pass
    

    def testAddDataset(self):
        
        datamanager = statismo.DataManager_vtkPD.Create(self.representer)
        datasets = map(read_vtkpd, self.datafiles)        
        
        for (dataset, filename) in zip(datasets, self.datafiles):        
            datamanager.AddDataset(dataset, filename)                

        self.assertEqual(datamanager.GetNumberOfSamples(),len(self.datafiles))
        
        for (i, sampleData) in enumerate(datamanager.GetData()): 
            self.assertEqual(sampleData.GetDatasetURI(),self.datafiles[i])

            
    def testLoadSave(self):
        datamanager =  statismo.DataManager_vtkPD.Create(self.representer)

        datasets = map(read_vtkpd, self.datafiles)        
        for (dataset, filename) in zip(datasets, self.datafiles):        
            datamanager.AddDataset(dataset, filename)                


        tmpfile = tempfile.mktemp(suffix="h5")
        representer = statismo.vtkStandardMeshRepresenter.Create()
        datamanager.Save(tmpfile)
        datamanager_new =  statismo.DataManager_vtkPD.Load(representer, tmpfile)
        
        self.assertEqual(datamanager.GetNumberOfSamples(), datamanager_new.GetNumberOfSamples())
        
        sampleSet = datamanager.GetData()
        newSampleSet = datamanager_new.GetData()
        for (sample, newSample) in zip(sampleSet, newSampleSet):
            self.assertTrue((sample.GetSampleVector() == newSample.GetSampleVector()).all() == True)

    def testLoadSaveSurrogateData(self):
        datamanager =  statismo.DataManagerWithSurrogates_vtkPD.Create(self.representer, os.path.join(DATADIR, "..", "hand_images", "surrogates", "hand_surrogates_types.txt"))

        ds_filename = os.path.join(DATADIR, "hand-1.vtk")
        ds = read_vtkpd(ds_filename)
        surrogate_filename =  os.path.join(DATADIR, "..", "hand_images", "surrogates", "hand-1_surrogates.txt")  
        datamanager.AddDatasetWithSurrogates(ds, ds_filename, surrogate_filename)

        tmpfile = tempfile.mktemp(suffix="h5")
        datamanager.Save(tmpfile)

        representer = statismo.vtkStandardMeshRepresenter.Create()
        datamanager_new = statismo.DataManagerWithSurrogates_vtkPD.Load(representer, tmpfile)

        self.assertEqual(datamanager.GetNumberOfSamples(), datamanager_new.GetNumberOfSamples())
        
        sampleSet = datamanager.GetData()
        newSampleSet = datamanager_new.GetData()
        for (sample, newSample) in zip(sampleSet, newSampleSet):
            self.assertTrue((sample.GetSampleVector() == newSample.GetSampleVector()).all() == True)


    def testCrossValidation(self):
        datamanager =  statismo.DataManager_vtkPD.Create(self.representer)
        datasets = map(read_vtkpd, self.datafiles)        
        for (dataset, filename) in zip(datasets, self.datafiles):        
            datamanager.AddDataset(dataset, filename)                

        cvFolds = datamanager.GetCrossValidationFolds(3, True)
        
        self.assertEqual(len(cvFolds), 3)
        
        training_data = cvFolds[0].GetTrainingData()
        test_data = cvFolds[0].GetTestingData()
              
        self.assertTrue(len(training_data) + len(test_data) == datamanager.GetNumberOfSamples())

        containsSameElement = set(training_data).isdisjoint(test_data)        
        self.assertTrue(containsSameElement, "a dataset is both in the test and training data")
              
        
suite = unittest.TestLoader().loadTestsFromTestCase(Test)

if __name__ == "__main__":
#import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
