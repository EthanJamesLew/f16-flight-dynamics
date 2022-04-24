//
// Created by elew on 4/23/22.
//
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <iostream>
#include <boost/range/algorithm.hpp>
#include <f16_flight_dynamics/F16Model/LowLevelFunctions.h>
#include <f16_flight_dynamics/F16Model/F16Plant.h>

namespace py = boost::python;
namespace numpy = boost::python::numpy;

template< typename T >
inline
std::vector<T> ndarray_to_vector(const numpy::ndarray& input){
  int input_size = input.shape(0);
  T* input_ptr = reinterpret_cast<T*>(input.get_data());
  std::vector<T> v(input_size);
  for (int i = 0; i < input_size; ++i)
    v[i] = *(input_ptr + i);
  return v;
}

template< typename T, size_t S >
boost::array<T, S> ndarray_to_array(const numpy::ndarray& input){
  int input_size = input.shape(0);

  assert(input_size == S);
  T* input_ptr = reinterpret_cast<T*>(input.get_data());
  boost::array<T, S> v{};
  for (int i = 0; i < input_size; ++i)
    v[i] = *(input_ptr + i);
  return v;
}

class F16Plant {
 public:
    numpy::ndarray dxdt(const numpy::ndarray &x,
                          const numpy::ndarray &uinput){
      F16Components::f16_full_type xd;
      F16Components::f16_state_type xa = ndarray_to_array<double, 13>(x);
      F16Components::f16_input_type ua = ndarray_to_array<double, 4>(uinput);

      F16Components::F16Plant plant;
      plant.subf16_model(xa, ua, xd);

      Py_intptr_t  shape[1] = { xd.size() };
      numpy::ndarray result = numpy::zeros(1, shape, numpy::dtype::get_builtin<double>());
      std::copy(xd.begin(), xd.end(), reinterpret_cast<double*>(result.get_data()));
      return result;
    }
  };


BOOST_PYTHON_MODULE (f16dynpy) {
  Py_Initialize();
  numpy::initialize();

  using namespace boost::python;
  def("adc", LowLevelFunctions::adc);
  def("cl", LowLevelFunctions::cl);
  def("cm", LowLevelFunctions::cm);
  def("cn", LowLevelFunctions::cn);
  def("cx", LowLevelFunctions::cx);
  def("cy", LowLevelFunctions::cy);
  def("cz", LowLevelFunctions::cz);

  class_<F16Plant>("F16Plant")
      .def("dxdt", &F16Plant::dxdt)
      ;
}
