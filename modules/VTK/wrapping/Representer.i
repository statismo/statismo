

////////////////////////////////////////////////////
// Representer
/////////////////////////////////////////////////////

%{
#include "statismo/Representer.h"
%}


namespace statismo {

template <class T> class RepresenterTraits {};
  
template<class T>
class Representer {
public:

	typedef typename RepresenterTraits<T>::DatasetPointerType DatasetPointerType;
	typedef typename RepresenterTraits<T>::DatasetConstPointerType DatasetConstPointerType;
	typedef typename RepresenterTraits<T>::PointType PointType;
	typedef typename RepresenterTraits<T>::ValueType ValueType;


	virtual ~Representer();
	
	/// Returns a name that identifies the representer
	virtual std::string GetName() const = 0;

 	DatasetConstPointerType GetReference() const = 0;
	virtual unsigned GetDimensions() const = 0; 	
	
	virtual std::string GetVersion() const = 0;

	
};

}

