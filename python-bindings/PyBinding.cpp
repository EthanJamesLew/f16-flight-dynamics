//
// Created by elew on 4/23/22.
//
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <f16_flight_dynamics/F16Model/LowLevelFunctions.h>
#include <python-bindings/F16PlantWrapper.h>
#include <python-bindings/LowLevelControllerWrapper.h>

namespace py = boost::python;
namespace numpy = boost::python::numpy;

void init_module() {
}

BOOST_PYTHON_MODULE (f16dynpy) {
  using namespace boost::python;
  Py_Initialize();
  numpy::initialize();

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
