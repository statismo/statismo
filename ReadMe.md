[![Build Status](https://travis-ci.org/statismo/statismo.svg?branch=arnaudgelas-Superbuild)](https://travis-ci.org/statismo/statismo)

Statismo is a C++ library for generating and manipulating Statistical
Shape and Image Models. It supports all commonly known types of
statistical models, including Shape models, Deformation Models and
Intensity Models. The implementation and interpretation is based on
Probabilistic PCA, which generalizes the standard PCA models and gives
a fully probabilistic interpretation.

How to build Statismo:
----------------------

Statismo is a header only library. There is no need to compile statismo. You simply include the necessary
header files into your application and statismo is compiled together with  your application.
However, statismo depends on some third-party libraries (such as HDF5) and to use it from your applications,
you need to correctly specify all the path to the different include directories and libraries.

To simplify this process, we have provided a CMake file that takes care of this process.

To install statismo use the usual CMake workflow (here demonstrated for linux):

1- Create a directory that  holds your binary data and change into that directory:
```bash
$ mkdir build_dir
$ cd build_dir
```

2- Configure the project.
```bash
$ ccmake path_to_statismo
```
Make sure that you set the CMAKE_INSTALL_PREFIX to the path where you want to install statismo

3- Compile and install statismo
```bash
$ make
$ make install
```

This builds the dependencies and copies the necessary files to the installation directory.
In particular, it generates the file statismo-config.cmake, that holds the correct configuration.
To use statismo from your application, you can now simply include the command
FindPackage(statismo) into your CMakeLists.txt file. This will set up the CMake variables
- STATISMO_DIR
- STATISMO_LIBRARIES
- STATISMO_INCLUDE_DIRS
- STATISMO_LIBRARY_DIR

Example programs are provided in the Example folder.
To build the examples, set the options BUILD_VTK_EXAMPLES and BUILD_ITK_EXAMPLES with CMake and build statismo.

An example CMakeLists file that shows how you can setup your own project to use statismo can be
found on the [Wiki](https://github.com/statismo/statismo/wiki/compilation).


Unit Tests
----------
To test your basic installation, simply run in your build directory.
```bash
$ make test
```
Beside these basic tests, there is a larger set of unit tests available. These unit tests are written in Python, and
require that Statismo has been compiled with the option BUILD_PYTHON_WRAPPING_VTK.

To run the tests, write
```bash
$ make unit_test
```

Limitations and known issues
----------------------------
Statismo has been implemented and thoroughly tested on Linux.
It should also work on Windows (with Visual Studio 2008 and upwards) and MacOSX. However, some components, such as the Python Wrapping, have
not been tested on Windows and will not be supported.


Main Authors:
-------------
- Marcel Luethi
- Remi Blanc

Contributors:
------------
- Thomas Albrecht
- Orcun Goksel
- Arnaud Gelas
- Christoph Jud
- Sandro Schönborn
- Tobias Gass

License:
--------
Statismo itself is licensed under the BSD license. It depends, however, on other open source projects, which are distributed under different licenses. 
Most notably, these are Eigen, Boost and HDF5. You will find a copy of the respective license agreements in the folder ./3rdParty. 
