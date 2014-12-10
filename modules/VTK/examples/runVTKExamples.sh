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

#!/bin/sh

export DATADIR=$PWD/../share/data
export RESDIR=/tmp/results
mkdir $RESDIR

# build a shape model from the hand data
./vtkBuildShapeModelExample $DATADIR/hand_polydata/ $RESDIR/vtkShapeModel.h5

# build an intensity model 
./vtkBuildIntensityModelExample $DATADIR/hand_images $RESDIR/./vtkIntensityModel.h5
 
# sample from the model and save results
./vtkBasicSamplingExample $RESDIR/vtkShapeModel.h5 $RESDIR

# Crossvalidation 
./vtkCrossValidationExample $DATADIR/hand_polydata/ $RESDIR/

# Build a partially fixed model
./vtkBuildPosteriorModelExample $RESDIR/vtkShapeModel.h5 $DATADIR/hand_polydata/partial/hand-0-part.vtk $RESDIR/vtkPosteriorModel.h5 $RESDIR/hand-0-reconstruction.vtk

# Build a conditional model
./vtkBuildConditionalModelExample $DATADIR/hand_images $RESDIR/vtkConditionalModel.h5
