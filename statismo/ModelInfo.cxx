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

#ifndef __MODEL_INFO_H
#define __MODEL_INFO_H



#include "ModelInfo.h"
#include "HDF5Utils.h"
#include "Exceptions.h"
#include <iostream>
#include <ctime>
#include "DataManager.h"


namespace statismo {




inline
void
ModelInfo::Save(const H5::CommonFG& publicFg) const {
	using namespace H5;

	// get time and date

	time_t rawtime;
	struct tm * timeinfo;

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	 try {
		 Group publicInfo = publicFg.createGroup("./modelinfo");
		 HDF5Utils::writeString(publicInfo, "./build-time", asctime (timeinfo));
		 if (m_scores.rows() != 0 && m_scores.cols() != 0) {
			 HDF5Utils::writeMatrix(publicInfo, "./scores", m_scores);
		 }

		 Group dataInfoPublic = publicInfo.createGroup("./dataInfo");
		for (DataInfoList::const_iterator it = m_dataInfo.begin();it != m_dataInfo.end(); ++it)
		{
			HDF5Utils::writeString(dataInfoPublic, it->first.c_str(), it->second.c_str());
		}


		 dataInfoPublic.close();

		 Group builderInfoPublic = publicInfo.createGroup("./builderInfo");
			for (DataInfoList::const_iterator it = m_builderInfo.begin();	it != m_builderInfo.end(); ++it)
			{
				HDF5Utils::writeString(builderInfoPublic, it->first.c_str(), it->second.c_str());
			}

		 builderInfoPublic.close();
		 publicInfo.close();

	} catch (Exception& e) {
		 std::string msg(std::string("an exception occurred while writing model info HDF5 file \n") + e.getCDetailMsg());
		 throw StatisticalModelException(msg.c_str());
	}

}

inline
void
ModelInfo::Load(const H5::CommonFG& publicFg) {
	Group publicModelGroup = publicFg.openGroup("./modelinfo");
	try {
		HDF5Utils::readMatrix(publicModelGroup, "./scores", m_scores);
	} catch (Exception& e) {
		// the likely cause is that there are no scores. so we set them as empty
		m_scores.resize(0,0);
	}

	 Group dataInfoPublic = publicModelGroup.openGroup("./dataInfo");
	 unsigned numEntries = dataInfoPublic.getNumObjs();
	 for (unsigned i = 0; i < numEntries; i++) {
			HDF5Utils::readString(dataInfoPublic, dataInfoPublic.getObjnameByIdx(i).c_str());
	 }
	 dataInfoPublic.close();
	 publicModelGroup.close();
}

} // end namespace

#endif
