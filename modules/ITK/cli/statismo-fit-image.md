% STATISMO-FIT-IMAGE(8)

# NAME

statismo-fit-image - fits a model iteratively to an image.

# SYNOPSIS

statismo-fit-image [*options*] -i *input-file* *output-file*

# DESCRIPTION

statismo-fit-image iteratively fits a target image with the help of a model and a reference image. It's possible to fit with and without landmarks. The optimizer can get stuck in a local minima that doesn't represent a satisfactory result if no landmarks are provided.

# OPTIONS

## General options
-i, \--input-model *MODEL_FILE*
:	*MODEL_FILE* is the path to the model.

-m, \--input-movingimage *IMAGE_FILE*
:	*IMAGE_FILE* is the path to the moving image.

-f, \--input-fixedimage *IMAGE_FILE*
:	*IMAGE_FILE* is the path to the fixed image.

-w, \--regularization-weight *WEIGHT*
:	*WEIGHT* is the regularization weight that is used to ensure that the model parameters don't deviate too much from the mean. The higher this weight is, the closer the model parameters should stay to the mean. Note: The regularization is the sum over the square of all model parameters.

-n, \--number-of-iterations *NUM_ITERATIONS*
:	the number of iterations used in the fitting process


-d, \--dimensionality 
:	Specifies the dimensionality of the images and the model (either 2 or 3).

-o, \--output-fit *FITTED_IMAGE_FILE*
:	*FITTED_IMAGE_FILE* is the path where the fitted image will be saved.

-a, \--output-model-deformationfield *DEFORMATION_FIELD_FILE*
:	*DEFORMATION_FIELD_FILE* is the path where the deformation field caused by the model will be saved. This is equivalent to the entrie deformation field if landmarks were provided and in the case that no landmarks were provided, it doesn't include the translation and rotation.

-e, \--output-deformationfield *DEFORMATION_FIELD_FILE*
:	*DEFORMATION_FIELD_FILE* is the path where the entire deformation field will be saved. If no landmarks were provided, this includes the rotation and a translation.


## Landmarks (optional, if one is set then all have to be set)

\--fixed-landmarks *FIXED_LANDMARKS_FILE*
:	*FIXED_LANDMARKS_FILE* is the path to the the file containing the fixed landmarks.

\--moving-landmarks *MOVING_LANDMARKS_FILE*
:	*MOVING_LANDMARKS_FILE* is the path to the the file containing the moving landmarks. (That's the landmarks on the target image)

-v, \--landmarks-variance *VARIANCE*
:	*VARIANCE* is the landmarks variance (an estimate for how accurate your landmarks are).

## Print fitting information (optional)

-p, \--print-fitting-information
:	If this option is set, then fitting information such as the iteration number, the score as assigned by the metric and the current parameters will be printed.


# NOTE
The Landmarks format is as follows
:	Landmark name,1.coordinate,2.coordinate,3.coordinate

Example
:	pointA,2,-2,3

	pointB,3.1,3,-5

	pointC,7,8,9.08


Remark
:	In the case of 2D Images, either set the 3.coordinate to 0 or don't set it at all.
 
# Examples 
Fit a 3D image without landmarks:

    statismo-fit-image  -i model.h5 -w 0 -m moving-image.vtk  -f fixed-image.vtk -o projection.vtk


Fit a 3D image with landmarks and print fitting information:

    statismo-fit-image -i model.h5 -m moving-image.vtk -w 0.1 -f fixed-image.vtk -o projection.vtk --fixed-landmarks fixed-landmarks.csv --moving-landmarks moving-landmarks-from-target-image.csv -v 0.1 -p

Fit a 2D image with landmarks and print fitting information:

    statismo-fit-image -d 2 -i model.h5 -m moving-image.vtk -w 0.25 -f fixed-image.vtk -o projection.vtk --fixed-landmarks fixed-landmarks.csv --moving-landmarks moving-landmarks-from-target-image.csv -v 0.1 -p

Fit a 2D image without landmarks, print fitting information and save both the deformation field caused by the model and the entire deformation field:

    statismo-fit-image -d 2 -i model.h5 -m moving-image.vtk -w 0.25 -f fixed-image.vtk -o projection.vtk -e model-deform-field.vtk -a entire-deform-field.vtk -p

Hint
:	Use *statismo-warp-image* to apply the deformation fields to images.


# SEE ALSO

##Building Models:
*statismo-build-shape-model* (8).
Builds shape models from a list of meshes.

*statismo-build-deformation-model* (8).
Builds deformation models from a list of deformation fields

*statismo-build-gp-model* (8).
Builds shape or deformation models from a given gaussian process definition.

##Working with models:

*statismo-sample* (8).
Draws samples from a model.

*statismo-reduce-model* (8).
Reduces the number of components in a model.

*statismo-posterior* (8).
Creates a posterior model from an existing model.

*statismo-fit-surface* (8).
Fits a model iteratively in to a target mesh.

*statismo-fit-image* (8).
Fits a model iteratively to an image.

*statismo-warp-image* (8).
Applies a deformation field to an image.