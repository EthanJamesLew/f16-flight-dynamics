# we're using Ubuntu 21.04 because that has python3 = python3.9,
# which is a requirement for CSAF controls. Also, it was found
# the easiest to get the required Boost utilities setup. Boost
# did not work out of the box because of a strange GCC 10 issue,
# which was resolved by reverting to GCC 8
from ubuntu:21.04
ARG DEBIAN_FRONTEND=noninteractive

# install python and boost
# because of this bs, we have to use an alternative gcc/g++
# https://github.com/boostorg/ublas/issues/96
# graphviz is needed by CSAF
RUN apt-get update &&\
    apt-get install -yq git=1:2.30.2-1ubuntu1 curl=7.74.0-1ubuntu2.3 &&\
    apt-get install -yq python3=3.9.4-1 python3-pip=20.3.4-1ubuntu2 &&\
    apt-get install -yq libboost1.74-all-dev=1.74.0-8ubuntu2 gcc-8=8.4.0-7ubuntu3 g++-8=8.4.0-7ubuntu3 &&\
    apt-get install -yq graphviz=2.42.2-4build2 &&\
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8 &&\
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8 &&\
    update-alternatives --config g++ &&\
    update-alternatives --config gcc

# clone the optimized C++ F16 Model and install the python pieces
ADD . /f16-flight-dynamics
WORKDIR /f16-flight-dynamics
RUN pip install .
WORKDIR /

# now, install CSAF from PyPi
RUN pip install csaf-controls==0.2

# default is to provide a python3 environment
CMD ["python3"]
