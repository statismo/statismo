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

export DATADIR=$PWD/../../../data/
export RESDIR=/tmp/results
mkdir $RESDIR

# build a shape model from the hand data
./itkBuildShapeModel $DATADIR/hand_polydata/hand-0.vtk $DATADIR/hand_polydata/ $RESDIR/itkShapeModel.h5

# build a shape model from the hand data keeping 75pc of the total variance
./itkBuildShapeModel_75pcvar $DATADIR/hand_polydata/hand-0.vtk $DATADIR/hand_polydata/ $RESDIR/itkShapeModel_75pcvar.h5

# build an Image Intensity model using a ROI to ignore some pixels of the images
./itkBuildImageIntensityModelOnROI $DATADIR/hand_images/hand-0.vtk $DATADIR/hand_images/hand_ROI.png $DATADIR/hand_images/ $RESDIR/ImageModelWithROI_Mean.vtk

# Fitting the shape model to a point set (mesh)
./itkShapeModelFitting $RESDIR/itkShapeModel.h5 $DATADIR/hand_polydata/hand-1.vtk $RESDIR/fitted-hand-shape.vtk
 
# build a deformation model of the displacement fields that relate hand-0 with all the other examples
./itkBuildDeformationModel 2 $DATADIR/hand_dfs/ $RESDIR/itkDeformationModel.h5

# Fitting of the deformation model. For the fixed image we need to take image hand-0, as this was used as the fixed image
# for the registration of the images (which resulted in the displacement fields).
./itkDeformationModelFitting $RESDIR/itkDeformationModel.h5 $DATADIR/hand_images/hand-0.vtk $DATADIR/hand_images/hand-1.vtk $RESDIR/fittet-hand-df.vtk

# Perform a basic Gaussian Process registration 
./itkSimpleGaussianProcessImageToImageRegistration $DATADIR/hand_images/hand-1.vtk $DATADIR/hand_images/hand-2.vtk $RESDIR/deformationfield-simplegp.vtk

# Perform hybrid, Gaussian process registration
./itkLowRankGaussianProcessImageToImageRegistration $DATADIR/hand_images/hand-1.vtk $DATADIR/hand_landmarks/hand-1.fcsv $DATADIR/hand_images/hand-2.vtk $DATADIR/hand_landmarks/hand-2.fcsv $RESDIR/registered.vtk $RESDIR/deformationfield-hybridgp.vtk MeanSquares 70 100 0.1 100 100

