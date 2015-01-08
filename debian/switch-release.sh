#! /bin/bash
if [ -z $1 ]; then
    echo "Usage: switch-release.sh distribution-name"
    exit
fi
    
distrib=$1
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
cd $DIR
echo $DIR
cp -f changelog changelog.bck
cd ../
dch -r -D $distrib
debuild -S
cd debian
mv changelog.bck changelog

