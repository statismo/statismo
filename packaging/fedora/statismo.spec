Name:		statismo
Version: 	0.10.1	
Release:	4%{?dist}
Summary: 	Framework for building Statistical Image And Shape Models

Group: System Environment/Libraries
License: BSD
URL: https://github.com/statismo/statismo	
Source0: https://github.com/statismo/statismo/archive/v0.10.1.tar.gz
Source1: https://raw.githubusercontent.com/statismo/statismo/master/LICENSE

BuildRequires:	cmake
BuildRequires:	doxygen
BuildRequires:	graphviz
BuildRequires: 	gnuplot
BuildRequires: 	wget
BuildRequires: 	hdf5-devel
BuildRequires: 	eigen3-devel
BuildRequires: 	boost-devel
BuildRequires:	vtk-devel
BuildRequires: 	python2-devel
BuildRequires:	vxl-devel
BuildRequires:	gdcm-devel
BuildRequires: 	InsightToolkit-devel 
BuildRequires: 	jsoncpp-devel 
BuildRequires:  netcdf-cxx-devel

%description
Statismo is a c++ framework for statistical shape modeling. It supports all
shape modeling tasks, from model building to shape analysis. Although the main
focus of statismo lies on shape modeling, it is designed such that it supports
any kind of PCA based statistical model, including statistical deformation
models and intensity models. One of the main goals of statismo is to make the
exchange of statistical shape models easy. This is achieved by using a well
documented file format based on HDF5.	


#
#	doc
#
%package doc
Summary: 	Includes html documentation for statismo
Group:		Documentation
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
# copy License file
cp %{SOURCE1} .


%build
mkdir -p %{_target_platform}
pushd %{_target_platform}
# used -Wl,--as-needed to fix unused-direct-shlib-dependency rpmlint warning
export LDFLAGS="-Wl,--as-needed %{?__global_ldflags}"
%cmake .. \
	-DBUILD_DOCUMENTATION:BOOL=ON \
	-DBUILD_TESTING:BOOL=OFF \
	-DBUILD_WRAPPING:BOOL=OFF \
	-DBoost_INCLUDE_DIR:PATH=%{_includedir} \
	-DBoost_LIBRARY_DIR:PATH=%{_libdir} \
	-DCMAKE_BUILD_TYPE:STRING="RelWithDebInfo" \
	-DEIGEN3_INCLUDE_DIR:PATH=%{_includedir}/eigen3 \
	-DHDF5_CXX_INCLUDE_DIR:PATH=%{_includedir} \
	-DHDF5_C_INCLUDE_DIR:PATH=%{_includedir} \
	-DINSTALL_BIN_DIR:PATH=%{_bindir} \
	-DINSTALL_CMAKE_DIR:PATH=%{_libdir}/cmake/%{name} \
	-DINSTALL_INCLUDE_DIR:PATH=%{_includedir} \
	-DINSTALL_LIB_DIR:PATH=%{_libdir}/ \
        -DITK_DIR:BOOL=%{_libdir}/cmake/InsightToolkit \
	-DITK_SUPPORT:BOOL=ON \
	-DVTK_SUPPORT:BOOL=ON \
	-DVTK_DIR:PATH=%{_libdir}/cmake/vtk

popd

make -C %{_target_platform} # %{?_smp_mflags}

%install
make install DESTDIR=$RPM_BUILD_ROOT -C %{_target_platform}
install -d -p %{_target_platform}/doc/html $RPM_BUILD_ROOT/%{_docdir}/%{name}/html
install -p -m 644 %{SOURCE1} $RPM_BUILD_ROOT/%{_docdir}/%{name}/
install -p -m 644 ReadMe.md $RPM_BUILD_ROOT/%{_docdir}/%{name}/

%post -p /sbin/ldconfig
%postun -p /sbin/ldconfig

%files
%doc ReadMe.md
%license LICENSE
%{_libdir}/*.so.*

%files doc
%doc ReadMe.md
%license LICENSE
%{_docdir}/%{name}/

%files devel
%{_libdir}/*.so
%{_libdir}/cmake/%{name}/
%{_includedir}/%{name}/


%changelog
* Mon Dec 15 2014 Arnaud Gelas <arnaudgelas@gmail.com> - 0.10.1-4
 - fix files section for sub-package doc and devel

* Fri Dec 12 2014 Arnaud Gelas <arnaudgelas@gmail.com> - 0.10.1-3
 - fix mix (spaces/tabs), devel does not package LICENSE, doc subpackage list LICENSE, fix ownership issues

* Wed Dec 10 2014 Arnaud Gelas <arnaudgelas@gmail.com> - 0.10.1-2
 - fix file permissions, license, group (rpmlint issues)

* Tue Nov 11 2014 Arnaud Gelas <arnaudgelas@gmail.com> - 0.10.1-1
 - Initial rpm

