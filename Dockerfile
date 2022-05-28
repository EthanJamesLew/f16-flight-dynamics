# we're using Ubuntu 21.04 because that has python3 = python3.9,
# which is a requirement for CSAF controls. Also, it was found
# the easiest to get the required Boost utilities setup. Boost
# did not work out of the box because of a strange GCC 10 issue,
# which was resolved by reverting to GCC 8
from ubuntu:21.04
RUN apt update
RUN apt install -y git curl
ARG DEBIAN_FRONTEND=noninteractive

# install python and boost
RUN apt install -yq python3 python3-pip
RUN apt install -yq libboost1.74-all-dev

# because of this bs, we have to use an alternative gcc/g++
# https://github.com/boostorg/ublas/issues/96
RUN apt install -yq gcc-8 g++-8
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
RUN update-alternatives --config g++
RUN update-alternatives --config gcc

# clone the optimized C++ F16 Model and install the python pieces
ADD . /f16-flight-dynamics
WORKDIR /f16-flight-dynamics
RUN pip install .
WORKDIR /

# now, install CSAF from PyPi
RUN pip install csaf-controls
RUN apt install -yq graphviz

# default is to provide a python3 environment
CMD ["python3"]
