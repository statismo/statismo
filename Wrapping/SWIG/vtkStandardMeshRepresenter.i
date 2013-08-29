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
 
%{
#include "statismo/Representer.h"
#include "vtkStandardMeshRepresenter.h"
#include "vtkHelper.h"
#include "vtkPolyData.h"
%}

%include "Representer.i"


namespace statismo { 


class vtkPoint {
	public:
	vtkPoint(double x, double y, double z);
};





%rename(representerTraitsVtkPolyData) RepresenterTraits<vtkPolyData>;  
class RepresenterTraits<vtkPolyData> {
public:
	typedef const vtkPolyData* DatasetConstPointerType;
	typedef vtkPolyData* DatasetPointerType;
	typedef vtkPoint PointType;
	typedef vtkPoint ValueType;	
};



}
%template(RepresenterVTK) statismo::Representer<vtkPolyData>;


namespace statismo { 

class vtkStandardMeshRepresenter : public Representer<vtkPolyData> {
public:

	virtual ~vtkStandardMeshRepresenter();

typedef statismo::Domain<PointType> DomainType;

 
 %newobject Create; 
 static vtkStandardMeshRepresenter* Create(); 
 static vtkStandardMeshRepresenter* Create(const vtkPolyData* reference);
  
  const DomainType& GetDomain() const;
 
 unsigned GetNumberOfPoints();
// unsigned GetPointIdForPoint(const vtkPoint& pt);

private:
 
 vtkStandardMeshRepresenter();

};
} // namespace statismo
