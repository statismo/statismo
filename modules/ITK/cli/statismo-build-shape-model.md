% STATISMO-BUILD-SHAPE-MODEL(8)

# NAME

statismo-build-shape-model - builds shape models from a list of meshes

# SYNOPSIS

statismo-buid-shape-model [*options*] *output-file*

# DESCRIPTION

statismo-build-model is used to build a shape model from a given list of meshes. 
**The meshes have to be in correspondence**.

In the simplest case, a model can be built from existing meshes with the following command

    statismo-build-shape-model --data-list data.txt shapemodel.h5

In this case the file data.txt contains a list of filenames of meshes (which are already in correspondence) and writes a model called shapemodel.h5

# OPTIONS

-l, \--data-list *DATA_LIST*
:	*DATA_LIST* is the path to a file containing a list of mesh-files that will be used to create the shape model. Please only give the path to **one** mesh-file per line  in the data-list-file.

-o, \--output-file *OUTPUT_FILE*
:	*OUTPUT_FILE* is the path where the newly build model should be saved.

-p, \--procrustes *PROCRUSTES_MODE*
:	Specify how the data is aligned. *PROCRUSTES_MODE* can be **reference** which aligns all datasets rigidly to the reference or **GPA** which aligns all the datasets to the population mean.

-r, \--reference *FILE* 
:	Specify the reference used for model building. This is needed if *PROCRUSTES-MODE* is **reference**

-n, \--noise *NOISE*
:	Specify the noise variance of the PPCA model. Defaults to 0

 
# Examples 
Build a model where the data is aligned to the procrustes mean

    statismo-build-shape-model -p GPA -l data-list.txt shapemodel.h5

Build a model where the data is aligned to a given reference

    statismo-build-shape-model -p reference -r ref.vtk -l data-list.txt shapemodel.h5


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