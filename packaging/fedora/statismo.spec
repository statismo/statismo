Name:		statismo
Version: 	0.10.1
Release:	1%{?dist}
Summary: 	Framework for building Statistical Image And Shape Models.

Group: Scientific
License: BSD
URL: https://github.com/statismo/statismo
Source0: https://github.com/statismo/statismo/archive/v0.10.1.tar.gz

BuildRequires:	cmake
BuildRequires:	doxygen
BuildRequires:	graphviz
BuildRequires: 	gcc-c++
BuildRequires: 	hdf5-devel
BuildRequires: 	eigen3-devel
BuildRequires: 	boost-devel
BuildRequires:	vtk-devel
BuildRequires: 	python2-devel
BuildRequires:	vxl-devel
BuildRequires:	gdcm-devel
BuildRequires: 	InsightToolkit-devel

Requires: 	hdf5
Requires: 	boost-thread
Requires: 	InsightToolkit
Requires: 	vtk

%description
Statismo is a c++ framework for statistical shape modeling. It supports the
creation of different kinds of PCA based shape models, and provides a variety
of tools to manipulate, extend and explore the models. Furthermore, it offers
functionality for model-based shape and image analysis.

#
#	doc
#
%package doc
Summary: 	Includes html documentation for statismo
Group:		documentation
BuildArch:	noarch

%description doc
You should install the statismo-doc package if you want to access upstream
documentation for statismo.

#
#	devel
#
%package devel
Summary: 	Libraries and headers for statismo
Group:		Development/Libraries
Requires:	%{name}%{?_isa} = %{version}-%{release}

%description devel
You should install the statismo-devel package if you want to compile
applications based on statismo.






%prep
%setup -q


%build
mkdir -p %{_target_platform}
pushd %{_target_platform}

cmake .. \
	-DCMAKE_INSTALL_PREFIX:PATH=%{_prefix}/ \
	-DCMAKE_BUILD_TYPE:STRING="RelWithDebInfo" \
	-DBUILD_DOCUMENTATION:BOOL=ON \
	-DBUILD_SHARED_LIBS:BOOL=ON \
	-DBUILD_TESTING:BOOL=OFF \
	-DVTK_SUPPORT:BOOL=ON \
	-DITK_SUPPORT:BOOL=ON \
	-DINSTALL_LIB_DIR:PATH=%{_lib}/ \
	-DINSTALL_INCLUDE_DIR:PAHT=include \
	-DINSTALL_BIN_DIR:PATH=%{_bindir} \
	-DINSTALL_CMAKE_DIR:PATH=%{_lib}/cmake/%{name}

popd

make -C %{_target_platform} # %{?_smp_mflags}


%install
make install DESTDIR=$RPM_BUILD_ROOT -C %{_target_platform}
install -d %{_target_platform}/doc/html $RPM_BUILD_ROOT/%{_docdir}/%{name}/html

%files
%{_libdir}/*.so.*

%files doc
%doc %{_docdir}/%{name}/html

%files devel
%{_libdir}/*.so
%{_libdir}/cmake/*
%{_includedir}/*

%changelog
* Tue Nov 11 2014 Arnaud Gelas <arnaudgelas@gmail.com> - 0.10.1-1
 - Initial rpm
