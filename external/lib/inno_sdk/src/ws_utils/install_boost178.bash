#!/bin/bash 
set -x
set -e

################
# this websocket lib requires boost1.78,
# earlier versions may have issue in async IO
#
# once installed, please use the following command to build:
# export BOOST_INC=-I/opt/boost1.78/include/ && export BOOST_LIB=-L/opt/boost1.78/lib/ && export CPLUS_INCLUDE_PATH=/opt/boost1.78/include/:$CPLUS_INCLUDE_PATH && make -j 8
################

BOOST_SERVER=https://boostorg.jfrog.io/artifactory/main/release
BOOST_SUBDIR=1.78.0/source
BOOST_VER=1_78_0
BOOST_TARBALL=boost_$BOOST_VER.tar.gz

rm -f $BOOST_TARBALL \
    && rm -rf boost_$BOOST_VER \
    && wget $BOOST_SERVER/$BOOST_SUBDIR/$BOOST_TARBALL \
    && tar zxf $BOOST_TARBALL \
    && rm $BOOST_TARBALL \
    && cd boost_$BOOST_VER \
    && ./bootstrap.sh \
    && ./b2 \
       --prefix=/opt/boost1.78 \
       --with-system --with-thread \
       --with-date_time --with-regex \
       --with-serialization -j 8 \
       install \
    && cd .. \
    && rm -rf boost_$BOOST_VER
