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


#ifndef __UTILS_H_
#define __UTILS_H_

#include <fstream>
#include "Exceptions.h"
#include <boost/random.hpp>
#include <cstdlib>
#include <algorithm>
#include <iterator>
#include <iostream>
#include "time.h"
namespace statismo {

/**
 * \brief A number of small utility functions - internal use only.
 */
class Utils {
public:

	/**
	 * return string representation of t
	 */
	template <class T>
	static std::string toString(T t) {
		std::ostringstream os;
		os << t;
		return os.str();
	}


	/** return a N(0,1) vector of size n */
	static VectorType generateNormalVector(unsigned n) {
		VectorType v = VectorType::Zero(n);

		// we would like to use tr1 here as well, but on some versions of visual studio it hangs forever.
		// therefore we use the functionality from boost
		using namespace boost;
		minstd_rand gen;
		normal_distribution<> dist(0, 1);
		gen.seed((unsigned int) time(0));

		variate_generator<minstd_rand, normal_distribution<> > r(gen, dist);

		for (unsigned i=0; i < n; i++) {
			v(i) = r();
		}
		return v;
	}


	static VectorType ReadVectorFromTxtFile(const char *name) {
		typedef std::list<statismo::ScalarType> ListType;
		std::list<statismo::ScalarType> values;
		std::ifstream inFile(name, std::ios::in);
		 if (inFile.good()) {
			std::copy(std::istream_iterator<statismo::ScalarType>(inFile), std::istream_iterator<statismo::ScalarType>(), std::back_insert_iterator<ListType >(values));
			inFile.close();
		} else {
			throw StatisticalModelException((std::string("Could not read text file ") + name).c_str());
		}

		VectorType v = VectorType::Zero(values.size());
		unsigned i = 0;
		for (ListType::const_iterator it = values.begin(); it != values.end(); ++it) {
			v(i) = *it;
			i++;
		}
		return v;
	}




};

} // namespace statismo

#endif /* __UTILS_H_ */
