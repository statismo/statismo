% STATISMO-SAMPLE(8)

# NAME

statismo-sample - draws samples from a model

# SYNOPSIS

statismo-sample [*options*] -i *input-file* *output-file*

# DESCRIPTION

statismo-sample is used to draw samples from a model. It's possible to draw different samples such as the mean, a random sample or a sample with given parameters.

# OPTIONS

-m, \--mean 
:	Draws the mean from the model and saves it.

-r, \--reference 
:	Draws the reference from the model and saves it.

-p, \--parameters *PARAMETERS*
:	*PARAMETERS* is a list of parameters and their position that will be used to draw a sample. Parameters are speciefied in the following format: **POSITION1**:**VALUE1** **POSITIONn**:**VALUEn**. Unspecified parameters will be set to 0. The first parameter is at position 1.

-t, \--type *TYPE*
:	Specifies the type of the model. *TYPE* can either be **shape** or **deformation**.

-d, \--dimensionality 
:	Specifies the dimensionality of the model (either 2 or 3). This option is only available if the type is **deformation**.

-i, \--input-file *MODEL_FILE*
:	*MODEL_FILE* is the path to the model.

-o, \--output-file *OUTPUT_FILE*
:	*OUTPUT_FILE* is the path where the sample will be saved.




 
# Examples 
Draw a random sample from a shape model:

    statismo-sample -i model.h5 sample.vtk

or

    statismo-sample -i model.h5 -o sample.vtk

Draw a random sample from a 3D deformation model:

    statismo-sample -t deformation -i model.h5 -o sample.vtk
	
Draw a random sample from a 2D deformation model:

    statismo-sample -d 2 -t deformation -i model.h5 -o sample.vtk

Draw the mean from a shape model:

    statismo-sample -m -i model.h5 mean.vtk
	
Draw the reference from a shape model:

    statismo-sample -r -i model.h5 mean.vtk

Draw a sample from a shape model with the 1st parameter set to 1, the 5th parameter set to 0.1 and the 10th parameter set to 2.5:

    statismo-sample -p 1:1 4:0.1 9:2.5 -i model.h5 sample.vtk

Hint
:	When working with deformation models, use *statismo-warp-image* to apply the sampled deformation fields to images.

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

