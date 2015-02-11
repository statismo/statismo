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

-d, \--data-list *DATA_LIST*
:	*DATA_LIST* is the path to a file containing a list of mesh-files that will be used to create the shape model. Please only give the path to **one** mesh-file per line.

-o, \--output-file *OUTPUT_FILE*
:	*OUTPUT_FILE* is the path where the newly build model should be saved.

-p, \--procrustes *PROCRUSTES_MODE*
:	Specify how the data is aligned. *PROCRUSTES_MODE* can be **reference** (aligns all datasets rigidly to the reference) **GPA** alignes all the datasets to the population mean. This option is only available when *TYPE* is `shape`.

-r, \--reference *FILE* 
:	Specify the reference used for model building. This is needed in the caes that *PROCRUSTES-MODE* is `reference`

--noise *NOISE*
:	Specify the noise variance of the PPCA model. Defaults to 0

 <!---
\--auto-noise 
:	Estimate noise directly from dropped components.
--->
 
# Examples 
Build a model where the datasets are aligned to the procrustes mean

    statismo-build-shape-model -p GPA --data-list data.txt shapemodel.h5

Build a model where the datasets are aligned to a given reference

    statismo-build-shape-model --procrustes=reference --reference=ref.vtk --data-list data.txt shapemodel.h5


# SEE ALSO

