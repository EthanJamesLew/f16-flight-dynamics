//
// Created by elew on 4/27/22.
//

#ifndef F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_PYTYPECONVERT_H_
#define F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_PYTYPECONVERT_H_

#include <boost/python/numpy.hpp>
namespace py = boost::python;
namespace numpy = boost::python::numpy;

namespace PyTypeConvert {
/* numpy ndarray to std::vector */
template<typename T>
inline std::vector<T> ndarray_to_vector(const numpy::ndarray &input) {
  int input_size = input.shape(0);
  T *input_ptr = reinterpret_cast<T *>(input.get_data());
  std::vector<T> v(input_size);
  for (int i = 0; i < input_size; ++i)
    v[i] = *(input_ptr + i);
  return v;
}

/* numpy ndarray to boost::array */
template<typename T, size_t S>
boost::array<T, S> ndarray_to_array(const numpy::ndarray &input) {
  int input_size = input.shape(0);

  assert(input_size == S);
  T *input_ptr = reinterpret_cast<T *>(input.get_data());
  boost::array<T, S> v{};
  for (int i = 0; i < input_size; ++i)
    v[i] = *(input_ptr + i);
  return v;
}

/* boost::array to numpy ndarray */
template<typename T, size_t S>
numpy::ndarray array_to_ndarray(const boost::array<T, S> xd) {
  Py_intptr_t shape[1] = {xd.size()};
  numpy::ndarray result = numpy::zeros(1, shape, numpy::dtype::get_builtin<T>());
  std::copy(xd.begin(), xd.end(), reinterpret_cast<T *>(result.get_data()));
  return result;
}
}

#endif //F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_PYTYPECONVERT_H_
