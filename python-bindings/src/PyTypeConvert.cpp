//
// Created by elew on 4/27/22.
//

#include <python-bindings/PyTypeConvert.h>

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
}
