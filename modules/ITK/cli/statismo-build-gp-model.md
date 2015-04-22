% STATISMO-BUILD-GP-MODEL(8)

# NAME

statismo-build-gp-model - builds shape or deformation models from a given gaussian process definition

# SYNOPSIS

statismo-buid-gp-model [*options*] -r *reference-file* *output-file*

# DESCRIPTION

statismo-build-gp-model is used to build a shape or deformation model from a gaussian process definition.

# OPTIONS
-t, \--type *TYPE*
:	Specify the type of the model. *TYPE* can either be **shape** or **deformation**.

-d, \--dimensionality 
:	Specifies the dimensionality of the input image (either 2 or 3). This option is only available if the type if **deformation**

-k, \--kernel *KERNEL* 
:	Specify the kernel (covariance function) of the gaussian process.
   Supported kernels: **gauss**

-p, \--parameters *KERNEL_PARAMETERS*
:	*KERNEL_PARAMETERS* are the kernel parameters. The exact parameters depend on the kernel:

	- gauss: The gaussian kernel has one parameter
		- sigma: This parameter is a float that sets the sigma parameter in the gaussian kernel. Bigger values make the changes "smoother".

<!-- 
	- kernel with 2 parameters: this is an example for man writing purposes and is commented out
		- param1: float of some sort
		- param2 boolean of some sort
-->
	
-s, \--scale *SCALE* 
:	*SCALE* is a scaling factor the kernel is scaled with. The bigger this value is, the stronger the change is when modifying a model parameter.

-n, \--numberofbasisfunctions *NUMBER_OF_BASIS_FUNCTIONS* 
:	*NUMBER_OF_BASIS_FUNCTIONS*  specifies how many parameters the model will have.

-r, \--reference *REFERENCE_FILE*
:	*REFERENCE_FILE* is the path to the reference file (a mesh or an image) that will be used to construct the model.

-o, \--output-file *OUTPUT_FILE*
:	*OUTPUT_FILE* is the path where the model will be saved.

# Examples 

Build a shape model with a gaussian kernel and 50 basis functions:

    statismo-build-gp-model -t shape -k gauss -p 95.5 -s 100.42 -n 50 -r reference-mesh.vtk model.h5

Build a 3D deformation model with a gaussian kernel and 25 basis functions:

    statismo-build-gp-model -t deformation -k gauss -p 75 -s 100 -n 25 -r reference-image.vtk model.h5

Build a 2D deformation model with a gaussian kernel and 25 basis functions:

    statismo-build-gp-model -d 2 -t deformation -k gauss -p 75 -s 100 -n 25 -r reference-image.vtk model.h5


<!-- 
Build a shape model with a multi-parameter kernel and 50 basis functions: (this is an example on how to use multiple kernel parameters for future documentation writers)

    statismo-build-gp-model -t shape -k multi-parameter[float,bool,int] -p 5.12 true -99 -s 100 -n 50 -r reference-mesh.vtk model.h5
	
--> 

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

