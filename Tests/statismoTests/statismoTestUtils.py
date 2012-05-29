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

import statismo
from os import listdir
from os.path import join
import vtk

DATADIR = join("..", "..", "data", "hand_polydata")

def read_vtkpd(filename):
    reader = vtk.vtkPolyDataReader()
    reader.SetFileName(filename)
    reader.Update()
    
    pd = vtk.vtkPolyData()
    pd.ShallowCopy(reader.GetOutput())
    return pd


def getDataFiles(datadir):
    return [join(datadir, f) for f in listdir(datadir) if f.endswith('.vtk')]

def buildPolyDataModel(datadir, noise):
    
    files = getDataFiles(datadir)
    ref = read_vtkpd(files[0])
        
    representer = statismo.vtkPolyDataRepresenter.Create(ref, statismo.vtkPolyDataRepresenter.RIGID)    
    dm = statismo.DataManager_vtkPD.Create(representer)

    datasets = map(read_vtkpd, files)
    for (dataset, filename) in zip(datasets, files):        
        dm.AddDataset(dataset, filename)
    
    builder = statismo.PCAModelBuilder_vtkPD.Create()
    model =  builder.BuildNewModel(dm.GetSampleData(), noise)

    return model
  
def getPDPointWithId(pd, id):
    x = pd.GetPoints().GetPoint(id)[0]
    y = pd.GetPoints().GetPoint(id)[1]
    z = pd.GetPoints().GetPoint(id)[2]
    return (x, y, z)
