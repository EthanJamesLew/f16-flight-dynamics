//
// Created by elew on 4/23/22.
//
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <python-bindings/ComponentWrapper.h>
#include <f16_flight_dynamics/F16Model/LowLevelFunctions.h>
#include <f16_flight_dynamics/F16Model/LowLevelController.h>
#include <f16_flight_dynamics/F16Model/F16Plant.h>
#include <f16_flight_dynamics/F16Model/LowLevelFunctions.h>
namespace py = boost::python;
namespace numpy = boost::python::numpy;

/* create llc wrapper */
class LowLevelControllerWrapper : public ComponentWrapper<LowLevelController::LowLevelController,
                                                          F16Types::llc_input_type,
                                                          F16Types::llc_state_type,
                                                          F16Types::llc_output_type> {
};

/* create f16 wrapper */
class F16PlantWrapper : public ComponentWrapper<F16Components::F16Plant,
                                                          F16Types::f16_input_type ,
                                                          F16Types::f16_state_type ,
                                                          F16Types::f16_output_type> {
 public:
  numpy::ndarray subf16_model(const numpy::ndarray &state, const numpy::ndarray &u) {
    F16Types::f16_full_type xd{};
    F16Types::f16_state_type xa = PyTypeConvert::ndarray_to_array<double, F16Types::f16_state_type ::size()>(state);
    F16Types::f16_input_type ua = PyTypeConvert::ndarray_to_array<double, F16Types::f16_input_type::size()>(u);

    F16Components::F16Plant().subf16_model(xa, ua, xd);

    return PyTypeConvert::array_to_ndarray<double, F16Types::f16_full_type::size()>(xd);
  }
};

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

  class_<F16PlantWrapper>("F16Plant", init<>())
      .def("dxdt", &F16PlantWrapper::dxdt)
      .def("output",&F16PlantWrapper::output)
      .def("f16model", &F16PlantWrapper::subf16_model)
      ;

  class_<LowLevelControllerWrapper>("LowLevelController", init<>())
      .def("dxdt", &LowLevelControllerWrapper::dxdt)
      .def("output",&LowLevelControllerWrapper::output)
      ;
}
