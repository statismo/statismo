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
./vtkBuildPosteriorModelExample $RESDIR/vtkShapeModel.h5 $RESDIR/vtkPosteriorModel.h5

# Build a conditional model
./vtkBuildConditionalModelExample $DATADIR/hand_images $RESDIR/vtkConditionalModel.h5
