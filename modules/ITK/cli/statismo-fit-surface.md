% STATISMO-FIT-SURFACE(8)

# NAME

statismo-fit-surface - fits a model iteratively in to a target mesh.

# SYNOPSIS

statismo-fit-surface [*options*] -i *input-file* 

# DESCRIPTION

statismo-fit-surface fits a model iteratively in to a target mesh and then saves at least one of the two following: a fitted mesh or a projected mesh. It's possible to fit with or without landmarks. Fitting without landmarks can go wrong when it gets stuck in a local minima that represents a unsatisfactory result. **If you want to fit a model to a mesh that is already in correspondence, use statismo-posterior to do it analytically.**

# OPTIONS

## General options
-i, \--input-model *MODEL_FILE*
:	*MODEL_FILE* is the path to the model.

-t, \--input-targetmesh *MESH_FILE*
:	*MESH_FILE* is the path to the target mesh in to which the model will be fitted.

-w, \--regularization-weight *WEIGHT*
:	*WEIGHT* is the regularization weight that is used to ensure that the model parameters don't deviate too much from the mean. The higher this weight is, the closer the model parameters should stay to the mean. Note: The regularization is the sum over the square of all model parameters.

-o, \--output-fit *FITTED_MESH_FILE*
:	*FITTED_MESH_FILE* is the path where the fitted mesh will be saved. At least one of the two output meshes has to be specified. It's also possible to save both.

-j, \--output-projected *PROJECTED_MESH_FILE*
:	*PROJECTED_MESH_FILE* is the path where the projected mesh will be saved. At least one of the two output meshes has to be specified. It's also possible to save both.
-n, \--number-of-iterations *NUM_ITERATIONS*
:	the number of iterations used in the fitting process

## Landmarks (optional, if one is set then all have to be set)

-f, \--landmarks-fixed *FIXED_LANDMARKS_FILE*
:	*FIXED_LANDMARKS_FILE* is the path to the the file containing the fixed landmarks.

-m, \--landmarks-moving *MOVING_LANDMARKS_FILE*
:	*MOVING_LANDMARKS_FILE* is the path to the the file containing the moving landmarks. (That's the landmarks on the target mesh)

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
 
# Examples 
Fit a model in to a mesh without landmarks and save the projected as well as the fitted mesh:

    statismo-fit-surface  -i model.h5 -t target-mesh.vtk -w 0.01 -o fitted-mesh.vtk -j projected-mesh.vtk

Fit a model in to a mesh with landmarks and only save the projected mesh:

    statismo-fit-surface  -i model.h5 -t target-mesh.vtk -w 0.1 -j projected-mesh.vtk -f fixed-landmarks.csv  -m moving-landmarks-from-target-mesh.csv -v 0.1

Fit a model in to a mesh with landmarks, save the fitted mesh only and print fitting information:

    statismo-fit-surface  -i model.h5 -t target-mesh.vtk -w 0.05 -o fitted-mesh.vtk -f fixed-landmarks.csv  -m moving-landmarks-from-target-mesh.csv -v 0.1  -p



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
