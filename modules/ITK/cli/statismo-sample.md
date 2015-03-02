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
:	*PARAMETERS* is a list of parameters and their position that will be used to draw a sample. Parameters are speciefied in the following format: **POSITION1**:**VALUE1** **POSITIONn**:**VALUEn**. Unspecified parameters will be set to 0.

-t, \--type *TYPE*
:	Specifies the type of the model. *TYPE* can either be **shape** or **deformation**.

-i, \--input-file *MODEL_FILE*
:	*MODEL_FILE* is the path to the model.

-o, \--output-file *OUTPUT_FILE*
:	*OUTPUT_FILE* is the path where the sample will be saved.




 
# Examples 
Draw a random sample from a shape model:

    statismo-sample -i model.h5 sample.vtk

or

    statismo-sample -i model.h5 -o sample.vtk

Draw a random sample from a deformation model:

    statismo-sample -t deformation -i model.h5 -o sample.vtk

Draw the mean from a shape model:

    statismo-sample -m -i model.h5 mean.vtk
	
Draw the reference from a shape model:

    statismo-sample -r -i model.h5 mean.vtk

Draw a sample from a shape model with the 1st parameter set to 1, the 5th parameter set to 0.1 and the 10th parameter set to 2.5:

    statismo-sample -p 0:1 4:0.1 9:2.5 -i model.h5 sample.vtk

# SEE ALSO


*statismo-build-shape-model* (8).
Builds shape models from a list of meshes.

*statismo-build-deformation-model* (8).
Builds deformation models from a list of deformation fields

*statismo-build-gp-model* (8).
Builds shape or deformation models from a given gaussian process definition.
