% STATISMO-BUILD-DEFORMATION-MODEL(8)


# NAME

statismo-build-deformation-model - builds deformation models from a list of deformation fields

# SYNOPSIS

statismo-buid-deformation-model [*options*] *output-file*

# DESCRIPTION

statismo-build-deformation-model is used to build a deformation model from a given list of deformation fields. 
**All the deformation fields need to have the same domain (i.e. origin, spacing and size needs to be the same)**


# OPTIONS
-d, \--data-list *DATA_LIST*
:	*DATA_LIST* is the path to a file containing a list of files storing deformation fields that will be used to create the deformation model. Please only give the path to **one** file per line in the data-list-file.

-o, \--output-file *OUTPUT_FILE*
:	*OUTPUT_FILE* is the path where the newly build model should be saved.

-n, \--noise *NOISE*
:	Specify the noise variance of the PPCA model. Defaults to 0

<!-- 
-a, \--auto-noise 
:	Estimate noise directly from dropped components.
-->


# Examples 
Build a deformation model from the deformation fields specified in the file data.txt

    statismo-build-deformation-model -d data.txt shapemodel.h5

# SEE ALSO

*statismo-build-shape-model* (8).
Builds shape models from a list of meshes.

*statismo-build-gp-model* (8).
Builds shape or deformation models from a given gaussian process definition.

*statismo-sample* (8).
Draws samples from a model.