% STATISMO-REDUCE-MODEL(8)

# NAME

statismo-reduce-model - Reduces the number of components in a model.

# SYNOPSIS

statismo-reduce-model [*options*] -i *input-file* *output-file*

# DESCRIPTION

statismo-reduce-model is used to reduce the number of components in a model. It's possible to reduce the number of components to a fixed number or to a percentage of the model's total variance.
# OPTIONS

-t, \--type
:   Specifies the model's type. **shape** and **deformation** are available model types.

-d, \--dimensionality 
:	Specifies the dimensionality of the model (either 2 or 3). This option is only available if the type is **deformation**.

-n, \--numcomponents 
:	Creates a new model with the specified number of components

-v, \--totalvariance 
:	Creates a new Model that will have a fraction of the old models' variance. This parameter is in percent and thus ranges from 0 to 100 and is in Percent.

-i, \--input-file *MODEL_FILE*
:	*MODEL_FILE* is the path to the model.

-o, \--output-file *OUTPUT_FILE*
:	*OUTPUT_FILE* is the path where the new model with a reduced amount of components will be saved.

 
# Examples 
Crate a new shape model with 5 components:

    statismo-reduce-model -i model.h5 -n 5 reduced_model.h5


Crate a new shape model with 42.1% of to models' total variance:

    statismo-reduce-model -i model.h5 -v 42.1 reduced_model.h5

Crate a new 3D deformation model with 91% of to models' total variance:

    statismo-reduce-model -t deformation -i model.h5 -v 91 reduced_model.h5

Crate a new 2D deformation model with 91% of to models' total variance:

    statismo-reduce-model -d 2 -t deformation -i model.h5 -v 91 reduced_model.h5


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

