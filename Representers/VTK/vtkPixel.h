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


#ifndef VTKPIXEL_H
#define VTKPIXEL_H

#include <iostream>
#include <sstream>
#include "statismo/Exceptions.h"

using statismo::StatisticalModelException;

/**
  * \brief Helper class that represents a vtkPixel or arbitrary type and dimension
  * In vtk a pixel is just of type  T*. The statismo library relies on a proper
  * copy semantics, and hence requires such a wrapper.
 */

template <class TPixel, unsigned TDimensions>
class vtkNDPixel {
public:
	vtkNDPixel() { }
	~vtkNDPixel() {}

	vtkNDPixel(TPixel* x)  {
		for (unsigned d = 0; d < TDimensions; d++)
			m_pixel[d] = x[d];

	}


	TPixel& operator[](unsigned i) {
		if (i >= TDimensions) {
			std::ostringstream os;
			os << "Invalid index for vtkPixel (index = " << i << ")";
			throw StatisticalModelException(os.str().c_str());
		}
		else {
			return m_pixel[i];
		}
	}

	const TPixel& operator[](unsigned i) const {
		if (i >= TDimensions) {
			std::ostringstream os;
			os << "Invalid index for vtkPixel (index = " << i << ")";
			throw StatisticalModelException(os.str().c_str());
		}
		else {
			return m_pixel[i];
		}
	}


	vtkNDPixel	& operator=(const vtkNDPixel& rhs) {
		if (this != &rhs) {
			for (unsigned d = 0; d < TDimensions; d++) {
				m_pixel[d] = rhs.m_pixel[d];
			}

		}
		return *this;
	}

	vtkNDPixel(const vtkNDPixel& orig) {
		operator=(orig);
	}

private:
	TPixel m_pixel[TDimensions];
};



#endif
