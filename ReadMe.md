# Statismo - Flexible Shape Modeling [![Build Status](https://travis-ci.org/statismo/statismo.svg?branch=develop)](https://travis-ci.org/statismo/statismo)


Statismo is a c++ framework for statistical shape modeling. It supports all shape modeling tasks, from model building to shape analysis. Although the main focus of statismo lies on shape modeling, it is designed such that it supports any kind of PCA based statistical model, including statistical deformation models and intensiy models. One of the main goals of statismo is to make the exchange of statistical shape models easy. This is achieved by using a well documented file format based on [HDF5](http://hdf5group.org).

## Getting Started and Documentation

The best way to get started is to check the main concepts on the [Wiki](https://github.com/statismo/statismo/wiki) page and then look at the 
[example code](https://github.com/statismo/statismo/tree/master/Examples) that is provided with statismo.
These example programs are kept simple to illustrate the main principles and meant as a starting point for more complicated applications. Nevertheless, the examples are all working programs and can directly be used for simple shape modeling tasks.

More detailed documentation and links to presentations and articles that describe the underlying theory can be found in the 
[documentation section](https://github.com/statismo/statismo/wiki/Documentation).

There is also the [statismo-users](https://groups.google.com/forum/#!forum/statismo-users) google group for general questions and discussion regarding statismo and shape models in general.

## How to build Statismo:

In the minimal configuration, statismo depends on [Eigen](http://eigen.tuxfamily.org), [Boost](http://www.boost.org) and [HDF5](http://www.hdfgroup.org). For building the example programs, also [ITK](http://www.itk.org) and [VTK](http://www.vtk.org)  need to be installed. To make the installation easier, statismo provides a superbuild that downloads and compiles all the dependencies.

Statismo uses [CMake](http://www.cmake.org) as a build system and the build follows a typical cmake workflow. 

#### Building the superbuild 

We illustrate the workflow for unix systems, assuming that the statismo sources are in the folder statismo-src-dir. 

```
mkdir build   # create a build directory
cd build
ccmake statismo-src-dir/superbuild
```

In the dialog, the dependencies that should be built by the system are selected, or the corresponding path of the system libraries are set. To build all dependencies, type

```
make 
```


#### Configuring and installing statismo

After all dependencies have been set up, statismo itself can be configured and install
```
cd Statismo-build
ccmake .
```

Here, statismo specific options can be configured. Once this is done, type
```
make install
```
to install statismo in the installation directory that was specified in the previous step.

#### Buidling an application that uses statismo as a library

An example CMakeLists file that shows how you can setup your own project to use statismo can be
found on the [Wiki](https://github.com/statismo/statismo/wiki/compilation).


## Tools

Statismo (shape) models are best viewed using the [Statismo model viewer](https://github.com/statismo/statismo/wiki/Statismo%20Viewer).
Sometimes it is useful to look at the statismo file itself. [Hdfview](http://www.hdfgroup.org/products/java/hdfview/) provides a graphical interface  to explore the structure and data within a hdf5 file.

## Support for other languages

Statismo is a C++ framework and at the current stage the only the C++ interface is supported and maintained on all platform. However, some work has already been done to make statismo available from other languages: 

* For Statismo's vtk module, experimental Python wrappers are available, which are known to work on Linux systems. These wrappers are internally used for our [unit tests](https://github.com/statismo/statismo/tree/master/Tests/statismoTests), which also serve as usage examples.
* Statismo's VTK module can also be acccessed from [R](http://www.r-project.org) using Stefan Schlager's [RvtkStatismo](https://github.com/zarquon42b/RvtkStatismo). 

In principle it should also be easy to wrap the statismo ITK module by using ITK's WrapITK mechanism. However, we currently have no working wrappers for ITK and do not have the resources to work on it. Any help is greatly appreciated. 

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
- Stefan Schlager
- Sandro Schönborn


The main development is currently done by the [Graphics and Vision Research Group](http://gravis.cs.unibas.ch) at the University of Basel.

## License:

Statismo itself is licensed under the BSD license. It depends, however, on other open source projects, which are distributed under different licenses. Most notably, these are [Eigen](http://eigen.tuxfamily.org), [Boost](http://www.boost.org) and [HDF5](http://www.hdfgroup.org) and, depending on the configuration [ITK](http://www.itk.org) and [VTK](http://www.vtk.org)
