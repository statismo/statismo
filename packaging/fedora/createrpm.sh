#! /bin/bash

if [ ! -d .git ]; then
  echo "this script must be executed from the git source tree"
  exit 0
fi

if [ ! -d ~/rpmbuild ]; then
  echo "set up rpm dev"
  rpmdev-setuptree
fi

version_major=0
version_minor=10
version_patch=1

version=$version_major.$version_minor.$version_patch

echo "creating tarball"
git archive --format=tar --prefix=statismo-$version v$version | gzip -f > statismo-$version.tar.gz
mv statismo-$version.tar.gz ~/rpmbuild/SOURCES/

echo "creating rpm"
pushd ~/rpmbuild/SPECS > /dev/null
  rpmbuild -ba statismo.spec
popd > /dev/null
