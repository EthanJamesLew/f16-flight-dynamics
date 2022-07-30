# we're using Ubuntu 22.04 because that has python3 = python3.9,
# which is a requirement for CSAF controls. Also, it was found
# the easiest to get the required Boost utilities setup. Boost
# did not work out of the box because of a strange GCC 10 issue,
# which was resolved by reverting to GCC 9
from ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

# install python and boost
# because of this bs, we have to use an alternative gcc/g++
# https://github.com/boostorg/ublas/issues/96
# graphviz is needed by CSAF
RUN apt-get --no-install-recommends update &&\
    apt-get --no-install-recommends install -yq git=1:2.34.1-1ubuntu1.4 curl=7.81.0-1ubuntu1.3 cmake=3.22.1-1ubuntu1 &&\
    apt-get --no-install-recommends install -yq python3=3.10.4-0ubuntu2 python3-pip=22.0.2+dfsg-1 &&\
    apt-get --no-install-recommends install -yq libboost1.74-all-dev=1.74.0-14ubuntu3 gcc-9=9.4.0-5ubuntu1 g++-9=9.4.0-5ubuntu1 &&\
    apt-get --no-install-recommends install -yq graphviz &&\
    apt-get clean &&\
    rm -rf /var/lib/apt/lists/* &&\
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9 &&\
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9 &&\
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
