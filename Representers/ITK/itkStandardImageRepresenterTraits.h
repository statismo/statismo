#include "itkImage.h"
#include "itkVector.h"

namespace statismo {
template<>
struct RepresenterTraits<itk::Image<itk::Vector<float, 3u>, 3u> > {

	typedef itk::Image<itk::Vector<float, 3u>, 3u> VectorImageType;
	typedef VectorImageType::Pointer DatasetPointerType;
	typedef VectorImageType::Pointer DatasetConstPointerType;
	typedef typename VectorImageType::PointType PointType;
	typedef typename VectorImageType::PixelType ValueType;

	static void DeleteDataset(DatasetPointerType d) {VectorImageType::Pointer smartPointerD = d;}
};

template<>
struct RepresenterTraits<itk::Image<itk::Vector<float, 2u>, 2u> > {

	typedef itk::Image<itk::Vector<float, 2u>, 2u> VectorImageType;
	typedef VectorImageType::Pointer DatasetPointerType;
	typedef VectorImageType::Pointer DatasetConstPointerType;
	typedef typename VectorImageType::PointType PointType;
	typedef typename VectorImageType::PixelType ValueType;

	static void DeleteDataset(DatasetPointerType d) {VectorImageType::Pointer smartPointerD = d;}
};


template<>
struct RepresenterTraits<itk::Image<float, 2u> > {

	typedef itk::Image<float, 2u> ImageType;
	typedef ImageType::Pointer DatasetPointerType;
	typedef ImageType::Pointer DatasetConstPointerType;
	typedef typename ImageType::PointType PointType;
	typedef typename ImageType::PixelType ValueType;

	static void DeleteDataset(DatasetPointerType d) {ImageType::Pointer smartPointerD = d;}
};

template<>
struct RepresenterTraits<itk::Image<float, 3u> > {

	typedef itk::Image<float, 3u> ImageType;
	typedef ImageType::Pointer DatasetPointerType;
	typedef ImageType::Pointer DatasetConstPointerType;
	typedef typename ImageType::PointType PointType;
	typedef typename ImageType::PixelType ValueType;

	static void DeleteDataset(DatasetPointerType d) {ImageType::Pointer smartPointerD = d;}
};

template<>
struct RepresenterTraits<itk::Image<float, 2u> > {

	typedef itk::Image<float, 2u> ImageType;
	typedef ImageType::Pointer DatasetPointerType;
	typedef ImageType::Pointer DatasetConstPointerType;
	typedef typename ImageType::PointType PointType;
	typedef typename ImageType::PixelType ValueType;

	static void DeleteDataset(DatasetPointerType d) {ImageType::Pointer smartPointerD = d;}
};

template<>
struct RepresenterTraits<itk::Image<short, 3u> > {

	typedef itk::Image<short, 3u> ImageType;
	typedef ImageType::Pointer DatasetPointerType;
	typedef ImageType::Pointer DatasetConstPointerType;
	typedef typename ImageType::PointType PointType;
	typedef typename ImageType::PixelType ValueType;

	static void DeleteDataset(DatasetPointerType d) {ImageType::Pointer smartPointerD = d;}
};

template<>
struct RepresenterTraits<itk::Image<short, 2u> > {

	typedef itk::Image<short, 2u> ImageType;
	typedef ImageType::Pointer DatasetPointerType;
	typedef ImageType::Pointer DatasetConstPointerType;
	typedef typename ImageType::PointType PointType;
	typedef typename ImageType::PixelType ValueType;

	static void DeleteDataset(DatasetPointerType d) {ImageType::Pointer smartPointerD = d;}
};

template<>
struct RepresenterTraits<itk::Image<unsigned char, 3u> > {

	typedef itk::Image<char, 3u> ImageType;
	typedef ImageType::Pointer DatasetPointerType;
	typedef ImageType::Pointer DatasetConstPointerType;
	typedef typename ImageType::PointType PointType;
	typedef typename ImageType::PixelType ValueType;

	static void DeleteDataset(DatasetPointerType d) {ImageType::Pointer smartPointerD = d;}
};

template<>
struct RepresenterTraits<itk::Image<unsigned char, 2u> > {

	typedef itk::Image<char, 2u> ImageType;
	typedef ImageType::Pointer DatasetPointerType;
	typedef ImageType::Pointer DatasetConstPointerType;
	typedef typename ImageType::PointType PointType;
	typedef typename ImageType::PixelType ValueType;

	static void DeleteDataset(DatasetPointerType d) {ImageType::Pointer smartPointerD = d;}
};


} // namespace statismo
