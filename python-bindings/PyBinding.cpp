//
// Created by elew on 4/23/22.
//
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <iostream>
#include <boost/range/algorithm.hpp>
#include <f16_flight_dynamics/F16Model/LowLevelFunctions.h>
#include <f16_flight_dynamics/F16Model/F16Plant.h>
#include <f16_flight_dynamics/F16Model/F16Types.h>
#include <f16_flight_dynamics/F16Model/LowLevelController.h>

namespace py = boost::python;
namespace numpy = boost::python::numpy;

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

/* python object for llc */
class LowLevelControllerWrapper {
 public:
  LowLevelControllerWrapper() {
    llc = LowLevelController::LowLevelController();
  }

  /* evolution function (der) */
  numpy::ndarray dxdt(const numpy::ndarray &f16_state, const numpy::ndarray &u) {
    F16Types::llc_input_type ua = ndarray_to_array<double, 4>(u);
    F16Components::f16_full_type xa = ndarray_to_array<double, 17>(f16_state);

    auto xd = llc.dxdt(ua, xa);

    return array_to_ndarray<double, 3>(xd);
  }

  numpy::ndarray output(const numpy::ndarray &f16_state, const numpy::ndarray &u) {
    F16Types::llc_input_type ua = ndarray_to_array<double, 4>(u);
    F16Components::f16_full_type xa = ndarray_to_array<double, 17>(f16_state);

    auto xd = llc.output(ua, xa);

    return array_to_ndarray<double, 4>(xd);
  }

 private:
  LowLevelController::LowLevelController llc;

};

/* python object for plant dynamics */
class F16PlantWrapper {
 public:
  /* evolution function (der) */
  numpy::ndarray dxdt(const numpy::ndarray &x, const numpy::ndarray &uinput) {
    F16Components::f16_full_type xd;
    F16Components::f16_state_type xa = ndarray_to_array<double, 13>(x);
    F16Components::f16_input_type ua = ndarray_to_array<double, 4>(uinput);

    F16Components::F16Plant plant;
    plant.subf16_model(xa, ua, xd);

    return array_to_ndarray<double, 17>(xd);
  }
};

void init_module() {
  Py_Initialize();
  numpy::initialize();
}

BOOST_PYTHON_MODULE (f16dynpy) {
  using namespace boost::python;
  def("init_module", init_module);
  def("adc", LowLevelFunctions::adc);
  def("cl", LowLevelFunctions::cl);
  def("cm", LowLevelFunctions::cm);
  def("cn", LowLevelFunctions::cn);
  def("cx", LowLevelFunctions::cx);
  def("cy", LowLevelFunctions::cy);
  def("cz", LowLevelFunctions::cz);

  class_<F16PlantWrapper>("F16Plant").def("dxdt", &F16PlantWrapper::dxdt);

  class_<LowLevelControllerWrapper>("LowLevelController", init<>()).def("dxdt", &LowLevelControllerWrapper::dxdt).def(
      "output",
      &LowLevelControllerWrapper::output);
}
