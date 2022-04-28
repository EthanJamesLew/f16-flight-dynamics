//
// Created by elew on 4/27/22.
//

#ifndef F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_COMPONENTWRAPPER_H_
#define F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_COMPONENTWRAPPER_H_

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <python-bindings/PyTypeConvert.h>

namespace py = boost::python;
namespace numpy = boost::python::numpy;

/*
 * component wrapper provides a python friendly interface for Component classes, exposing their core
 * method arguments as numpy arrays.
 */
template<typename C, typename In, typename State, typename Out>
class ComponentWrapper {
 public:
  ComponentWrapper();

  /* wrapped evolution function */
  numpy::ndarray dxdt(const numpy::ndarray &state, const numpy::ndarray &u);

  /* wrapped output function */
  numpy::ndarray output(const numpy::ndarray &state, const numpy::ndarray &u);

 private:
  C component;

};

template<typename C, typename In, typename State, typename Out>
ComponentWrapper<C, In, State, Out>::ComponentWrapper() {
  component = C();
}

/* wrapped evolution function */
template<typename C, typename In, typename State, typename Out>
numpy::ndarray ComponentWrapper<C, In, State, Out>::dxdt(const numpy::ndarray &state, const numpy::ndarray &u) {
  State xd{};
  State xa = PyTypeConvert::ndarray_to_array<double, State::size()>(state);
  In ua = PyTypeConvert::ndarray_to_array<double, In::size()>(u);

  component.update(0.0, xa, xd, ua);

  return PyTypeConvert::array_to_ndarray<double, State::size()>(xd);

}

/* wrapped output function */
template<typename C, typename In, typename State, typename Out>
numpy::ndarray ComponentWrapper<C, In, State, Out>::output(const numpy::ndarray &state, const numpy::ndarray &u) {
  Out xd{};
  State xa = PyTypeConvert::ndarray_to_array<double, State::size()>(state);
  In ua = PyTypeConvert::ndarray_to_array<double, In::size()>(u);

  component.output(0.0, xa, xd, ua);

  return PyTypeConvert::array_to_ndarray<double, Out::size()>(xd);
}

#endif //F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_COMPONENTWRAPPER_H_
