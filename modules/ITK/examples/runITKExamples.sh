#!/bin/sh

export DATADIR=$PWD/../share/data
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
./bin/itkLowRankGaussianProcessImageToImageRegistration $DATADIR/hand_images/hand-1.vtk $DATADIR/hand_landmarks/hand-1.fcsv $DATADIR/hand_images/hand-2.vtk $DATADIR/hand_landmarks/hand-2.fcsv $RESDIR/registered.vtk $RESDIR/deformationfield-hybridgp.vtk MeanSquares 70 100 0.1 100 100



