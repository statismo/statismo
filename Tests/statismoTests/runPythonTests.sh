#!/bin/bash

SWIGDIR=$1
TESTDIR=$2
cd $TESTDIR
PYTHONPATH=$PYTHONPATH:$SWIGDIR python statismoTests.py
cd - &>/dev/null
