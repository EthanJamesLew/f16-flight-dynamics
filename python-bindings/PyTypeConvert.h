//
// Created by elew on 4/27/22.
//

#ifndef F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_PYTYPECONVERT_H_
#define F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_PYTYPECONVERT_H_

#include <boost/python/numpy.hpp>

namespace PyTypeConvert {
namespace numpy = boost::python::numpy;

/* numpy ndarray to std::vector */
template<typename T>
inline std::vector<T> ndarray_to_vector(const numpy::ndarray &input);

/* numpy ndarray to boost::array */
template<typename T, size_t S>
boost::array<T, S> ndarray_to_array(const numpy::ndarray &input);

/* boost::array to numpy ndarray */
template<typename T, size_t S>
numpy::ndarray array_to_ndarray(const boost::array<T, S> xd);

}

#endif //F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_PYTYPECONVERT_H_
