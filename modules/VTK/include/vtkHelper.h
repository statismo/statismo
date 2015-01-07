/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *
 * Copyright (c) 2011 University of Basel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __VTK_HELPER_H
#define __VTK_HELPER_H


#include <iostream>
#include <sstream>

#include "CommonTypes.h"
#include "Exceptions.h"

namespace statismo {

/**
  * \brief Helper class that represents a vtkPixel or arbitrary type and dimension
  * In vtk a pixel is just of type  T*. The statismo library relies on a proper
  * copy semantics, and hence requires such a wrapper.
 */
class vtkNDPixel {
  public:

    vtkNDPixel(unsigned dimensions) : m_pixel(new double[dimensions]), m_dimensions(dimensions) { }
    ~vtkNDPixel() {
        delete[] m_pixel;
    }

    vtkNDPixel(double* x, unsigned dimensions)  : m_pixel(new double[dimensions]), m_dimensions(dimensions) {
        for (unsigned d = 0; d < dimensions; d++)
            m_pixel[d] = x[d];

    }

    double& operator[](unsigned i) {
        if (i >= m_dimensions) {
            std::ostringstream os;
            os << "Invalid index for vtkPixel (index = " << i << ")";
            throw statismo::StatisticalModelException(os.str().c_str());
        } else {
            return m_pixel[i];
        }
    }

    const double& operator[](unsigned i) const {
        if (i >= m_dimensions) {
            std::ostringstream os;
            os << "Invalid index for vtkPixel (index = " << i << ")";
            throw statismo::StatisticalModelException(os.str().c_str());
        } else {
            return m_pixel[i];
        }
    }


    vtkNDPixel	& operator=(const vtkNDPixel& rhs) {
        if (this != &rhs) {
            m_dimensions = rhs.m_dimensions;
            m_pixel = new double[rhs.m_dimensions];
            for (unsigned d = 0; d < rhs.m_dimensions; d++) {
                m_pixel[d] = rhs.m_pixel[d];
            }

        }
        return *this;
    }

    vtkNDPixel(const vtkNDPixel& orig) {
        operator=(orig);
    }

  private:
    double* m_pixel;
    unsigned m_dimensions;
};



/**
 * \brief Helper class that represents a vtkPoint.
 * In vtk a point is just of type  T*. The statismo library relies on a proper
 * copy semantics, and hence requires such a wrapper..
 */

class vtkPoint {
  public:
    vtkPoint() {
        m_pt[0]  = 0;
        m_pt[1] = 0;
        m_pt[2] = 0;
    }

    vtkPoint(double x, double y, double z) {
        m_pt[0] = x;
        m_pt[1] = y;
        m_pt[2] = z;
    }

    vtkPoint(double* d) {
        m_pt[0] = d[0];
        m_pt[1] = d[1];
        m_pt[2] = d[2];
    }

    double& operator[](unsigned i) {
        return m_pt[i];
    }
    const double& operator[](unsigned i) const {
        return m_pt[i];
    }

    const double* data() const {
        return m_pt;
    }


    vtkPoint& operator=(const vtkPoint& rhs) {
        if (this != &rhs) {
            m_pt[0] = rhs.m_pt[0];
            m_pt[1] = rhs.m_pt[1];
            m_pt[2] = rhs.m_pt[2];
        }
        return *this;
    }

    vtkPoint(const vtkPoint& orig) {
        operator=(orig);
    }


  private:
    double m_pt[3];
};

class vtkHelper {
  public:
    static int vtkDataTypeIdToStatismoDataTypeId(int vtkDataTypeId) {

        int dataType = statismo::Void;
        switch(vtkDataTypeId) {
        case VTK_UNSIGNED_CHAR:
            dataType = statismo::UNSIGNED_CHAR;
            break;
        case VTK_SIGNED_CHAR:
            dataType = statismo::SIGNED_CHAR;
            break;
        case VTK_FLOAT:
            dataType = statismo::FLOAT;
            break;
        case VTK_DOUBLE:
            dataType = statismo::DOUBLE;
            break;
        case VTK_UNSIGNED_INT:
            dataType = statismo::UNSIGNED_INT;
            break;
        case VTK_INT:
            dataType = statismo::SIGNED_INT;
            break;
        case VTK_UNSIGNED_SHORT:
            dataType = statismo::UNSIGNED_SHORT;
            break;
        case VTK_SHORT:
            dataType = statismo::SIGNED_SHORT;
            break;
        case VTK_UNSIGNED_LONG:
            dataType = statismo::UNSIGNED_LONG;
            break;
        case VTK_LONG:
            dataType = statismo::SIGNED_LONG;
            break;
        default:
            throw statismo::StatisticalModelException("Unsupported data type for dataArray in vtkStandardMeshRepresenter::GetAsDataArray.");
        }
        return dataType;
    }
};

} // namespace statismo
#endif // __VTK_HELPER_H
