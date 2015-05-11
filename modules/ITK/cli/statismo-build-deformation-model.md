% STATISMO-BUILD-DEFORMATION-MODEL(8)


# NAME

statismo-build-deformation-model - builds deformation models from a list of deformation fields

# SYNOPSIS

statismo-buid-deformation-model [*options*] *output-file*

# DESCRIPTION

statismo-build-deformation-model is used to build a deformation model from a given list of deformation fields. 
**All the deformation fields need to have the same domain (i.e. origin, spacing and size needs to be the same)**


# OPTIONS
-l, \--data-list *DATA_LIST*
:	*DATA_LIST* is the path to a file containing a list of files storing deformation fields that will be used to create the deformation model. Please only give the path to **one** file per line in the data-list-file.

-o, \--output-file *OUTPUT_FILE*
:	*OUTPUT_FILE* is the path where the newly build model should be saved.

-n, \--noise *NOISE*
:	Specify the noise variance of the PPCA model. Defaults to 0

-d, \--dimensionality 
:	Specifies the dimensionality of the images used to build the model (either 2 or 3).



# Examples 
Build a 3D deformation model from the deformation fields specified in the file data.txt

    statismo-build-deformation-model -l data-list.txt deformationmodel.h5

Build a 2D deformation model from the deformation fields specified in the file data.txt

    statismo-build-deformation-model -d 2 -l data-list.txt deformationmodel.h5

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