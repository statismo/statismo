#ifndef __STATISMO_ITK_CONFIG_H
#define __STATISMO_ITK_CONFIG_H

#include "statismo/Config.h"

// in case we are using itk,  we are using the HDF5 version that comes with ITK 
#include "itk_H5Cpp.h"
// prevent standard HDF5 header from being included
#define _H5CPP_H 
#define __H5Cpp_H
#endif // __STATISMO_ITK_CONFIG_H
