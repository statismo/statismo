#include <vector>

class Point<unsigned Dimension> {
public:
	Point() : m_ptdata(Dimension){}

	Point(const std::vector<float> ptdata) : m_ptdata(Dimension) {
		if (ptdata.size != Dimension) {
			std::ostrstream os;
			os << "Pointdata of invalid size provided (required ";
			os << Dimension << " got " << ptdata.size << ")";
			throw StatisticalModelException(os.str().c_str());
		}
		m_ptdata = ptdata;
	}

	double& operator[](unsigned i) {
		return m_ptdata[i];}
	}
	const float& operator[](unsigned i) const {return m_ptdata[i];}

	const float* data() const {return m_ptdata.data; }


	Point& operator=(const Point& rhs) {
		m_ptdata = rhs.m_ptdata;
		return *this;
	}

	vtkPoint(const vtkPoint& orig) {
		operator=(orig);
	}


private:
	std::vector<float> m_ptdata;
};
