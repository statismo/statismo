# Statismo - Flexible Shape Modeling [![Build Status](https://travis-ci.org/statismo/statismo.svg?branch=develop)](https://travis-ci.org/statismo/statismo)


Statismo is a c++ framework for statistical shape modeling. It supports all shape modeling tasks, from model building to shape analysis. Although the main focus of statismo lies on shape modeling, it is designed such that it supports any kind of PCA based statistical model, including statistical deformation models and intensiy models. One of the main goals of statismo is to make the exchange of statistical shape models easy. This is achieved by using a well documented file format based on [HDF5](http://hdf5group.org).

## Getting Started and Documentation

The easiest way to explore statismo is by using the command line interface. The command line interface provides easy to use tools for the most common shape modelling task. Two example scenarios that show the use of the command line interface are documented [here](https://github.com/statismo/statismo/wiki/statismo-cli) along with a detailed description for each command line tool. 

To perform more complex tasks you can include statismo as a library in your own c++ application. The best way to get started is to check the main concepts on the [Wiki](https://github.com/statismo/statismo/wiki) page and then look at the example code ([ITK Examples](https://github.com/statismo/statismo/tree/master/modules/ITK/examples), [VTK Examples](https://github.com/statismo/statismo/tree/master/modules/VTK/examples)) that are provided with statismo.These example programs are kept simple to illustrate the main principles and meant as a starting point for more complicated applications.

The mathematical principles underlying statismo are described in the paper [Gaussian Process Morphable Models](http://ieeexplore.ieee.org/document/8010438/).

More detailed documentation and links to presentations can be found in the 
[documentation section](https://github.com/statismo/statismo/wiki/Documentation).

There is also the [statismo-users](https://groups.google.com/forum/#!forum/statismo-users) google group for general questions and discussion regarding statismo and shape models in general.

## Installing statismo:

Statismo comes with binary packages for Debian/Ubuntu and Mac (via homebrew). On all other platforms, statismo needs to be compiled from the sources (using [CMake](http://www.cmake.org)). You can find detailed [build instructions](https://github.com/statismo/statismo/wiki/buildinstructions) on the statismo wiki.

##### Installation on Ubuntu and Debian
For Ubuntu 14.04, 14.10 and 15.04 you can install statismo using the following commands:
```
sudo add-apt-repository ppa:zarquon42/ppa
sudo apt-get update
sudo apt-get install statismo statismo-tools
```
and optionally to get docs and example data:
```
sudo apt-get install statismo-doc statismo-example-data
```

##### Installation on Mac via homebrew
To install statismo using hombrew simply issue the following commands
```
    $ brew tap homebrew/science
    $ brew install statismo 
    $ statismo-buid-gp-model -h
```
Updating an existing statismo installation can be achieved by issuing
```
    $ brew update && brew upgrade —all
    $ statismo-buid-gp-model -h
```

## Tools

Statismo (shape) models are best viewed using the [Statismo model viewer](https://github.com/statismo/statismo/wiki/Statismo%20Viewer).
Sometimes it is useful to look at the statismo file itself. [Hdfview](http://www.hdfgroup.org/products/java/hdfview/) provides a graphical interface  to explore the structure and data within a hdf5 file.

## Support for other languages

Statismo is a C++ framework and at the current stage the only the C++ interface is supported and maintained on all platform. However, some work has already been done to make statismo available from other languages: 

* For Statismo's vtk module, experimental Python wrappers are available, which are known to work on Linux systems. These wrappers are internally used for our [unit tests](https://github.com/statismo/statismo/tree/master/modules/VTK/wrapping/tests/statismoTests), which also serve as usage examples.
* Statismo's VTK module can also be acccessed from [R](http://www.r-project.org) using Stefan Schlager's [RvtkStatismo](https://github.com/zarquon42b/RvtkStatismo). 

In principle it should also be easy to wrap the statismo ITK module by using ITK's WrapITK or SimpleITK. However, we currently have no working wrappers for ITK and do not have the resources to work on it. Any help is greatly appreciated. 

## History

Statismo has originally been developed in the context of the [Co-Me](http://www.co-me.ch) research project as a collaboration between the [University of Bern](http://www.istb.unibe.ch), the [University of Basel](http://gravis.cs.unibas.ch) and the [ETH Zurich](http://www.vision.ee.ethz.ch/), with goal of the making it easy to exchange algorithms and shape models between different research groups. The original code has been written by 
* Marcel Luethi, University of Basel  and
* Remi Blanc, formerly at ETH Zurich.

In the meantime, many people have contributed to statismo, including
- Thomas Albrecht
- Tobias Gass
- Arnaud Gelas
- Thomas Gerig
- Christoph Jud
- Christoph Langguth
- Frank Mueller
- Stefan Schlager
- Sandro Schönborn


The main development is currently done by the [Graphics and Vision Research Group](http://gravis.cs.unibas.ch) at the University of Basel.


## Related Projects
* [Scalismo](http://github.com/unibas-gravis/scalismo) Scalismo is a library for image analysis and shape modelling for the Java Virtual Machine. It is written in [Scala](www.scala-lang.org) and based on the same underlying concepts as statismo (and partly developed by the same people).

* [Morpho](http://cran.r-project.org/web/packages/Morpho/index.html) An R toolset for Geometric Morphometrics and mesh processing, which can be used together with statismo using the package [RvtkStatismo](https://github.com/zarquon42b/RvtkStatismo).

* [Deformetrica](http://www.deformetrica.org/) Deformetrica is a software, written in C++, for the statistical analysis of 2D and 3D shape data.

* [Spharm-PDM Toolbox](https://www.nitrc.org/projects/spharm-pdm) is a shape correspondence software package using a parametric boundary description based on spherical harmonics.


## License:

Statismo itself is licensed under the BSD license. It depends, however, on other open source projects, which are distributed under different licenses. Most notably, these are [Eigen](http://eigen.tuxfamily.org) and [HDF5](http://www.hdfgroup.org) and, depending on the configuration, [ITK](http://www.itk.org) and [VTK](http://www.vtk.org).
