% STATISMO-POSTERIOR(8)

# NAME

statismo-posterior - creates a posterior model from an existing model.

# SYNOPSIS

statismo-posterior [*options*] -i *input-file* *output-file*

# DESCRIPTION

statismo-posterior makes it possible to create posterior model for both shape and deformation models. It's possible to create a posterior model by providing landmarks or in the case of a shape model it's also possible to use a mesh that is in correspondence. Using a mesh in correspondence only makes sense if you want to project a model instance in to the provided mesh (see examples).

# OPTIONS

-c, \--corresponding-mesh *FILE*
:	*FILE* is the path to a file containing a mesh that is in correspondence. This option can only be used with shape models. If a mesh in correspondence is used, then no landmarks are needed and providing landmarks will result in failure.

 -f, \--landmarks-fixed *FILE*
:	*FILE* is the path to a file containing the fixed landmarks.

 -m, \--landmarks-moving *FILE*
:	*FILE* is the path to a file containing the moving landmarks.

-v, \--landmarks-variance *VARIANCE*
:	*VARIANCE* is the variance that will be used to build the posterior model. This is only needed when building models with landmarks.

-t, \--type *TYPE*
:	Specifies the type of the model. *TYPE* can either be **shape** or **deformation**.

-d, \--dimensionality 
:	Specifies the dimensionality of the model (either 2 or 3). This option is only available if the type is **deformation**.

-i, \--input-file *MODEL_FILE*
:	*MODEL_FILE* is the path to the model.

-o, \--output-file *OUTPUT_FILE*
:	*OUTPUT_FILE* is the path where the sample will be saved.


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
Create a posterior model from a shape model with landmarks:

    statismo-posterior -i model.h5 -o posterior-model.h5 -f fixed-landmarks.csv -m moving-landmarks.csv -v 0.1

or

    statismo-posterior -i model.h5 -t shape -f fixed-landmarks.csv -m moving-landmarks.csv -v 0.1 posterior-model.h5

Create a posterior model from a shape model with a mesh in correspondence and then extract the mean from the posterior model (Projects a model instance in to a mesh and then extracts the projection):

    statismo-posterior -i model.h5 -o posterior-model.h5 -c corresponding-mesh.vtk
    statismo-sample --mean -i posterior-model.h5 -o projected-mesh.vtk

Create a posterior model from a 3D deformation model with landmarks:

    statismo-posterior -i model.h5 -t deformation -f fixed-landmarks.csv -m moving-landmarks.csv -v 0.5 posterior-model.h5

Create a posterior model from a 2D deformation model with landmarks:

    statismo-posterior -d 2 -i model.h5 -t deformation -f fixed-landmarks.csv -m moving-landmarks.csv -v 0.5 posterior-model.h5

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