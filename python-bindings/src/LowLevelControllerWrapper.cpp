//
// Created by elew on 4/27/22.
//
#include <python-bindings/PyTypeConvert.h>
#include <f16_flight_dynamics/F16Model/F16Types.h>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <f16_flight_dynamics/F16Model/LowLevelController.h>

namespace py = boost::python;
namespace numpy = boost::python::numpy;

/* python object for llc */
class LowLevelControllerWrapper {
 public:
  LowLevelControllerWrapper() {
    llc = LowLevelController::LowLevelController();
  }

  /* evolution function (der) */
  numpy::ndarray dxdt(const numpy::ndarray &f16_state, const numpy::ndarray &u) {
    F16Types::llc_input_type ua = PyTypeConvert::ndarray_to_array<double, 4>(u);
    F16Types::f16_full_type xa = PyTypeConvert::ndarray_to_array<double, 17>(f16_state);

    auto xd = llc.dxdt(ua, xa);

    return PyTypeConvert::array_to_ndarray<double, 3>(xd);
  }

  numpy::ndarray output(const numpy::ndarray &f16_state, const numpy::ndarray &u) {
    F16Types::llc_input_type ua = PyTypeConvert::ndarray_to_array<double, 4>(u);
    F16Types::f16_full_type xa = PyTypeConvert::ndarray_to_array<double, 17>(f16_state);

    auto xd = llc.output(ua, xa);

    return PyTypeConvert::array_to_ndarray<double, 4>(xd);
  }

 private:
  LowLevelController::LowLevelController llc;

};
