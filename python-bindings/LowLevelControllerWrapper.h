//
// Created by elew on 4/27/22.
//

#ifndef F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_LOWLEVELCONTROLLER_H_
#define F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_LOWLEVELCONTROLLER_H_

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <f16_flight_dynamics/F16Model/LowLevelController.h>

namespace py = boost::python;
namespace numpy = boost::python::numpy;

/* python object for llc */
class LowLevelControllerWrapper {
 public:
  LowLevelControllerWrapper();

  /* evolution function (der) */
  numpy::ndarray dxdt(const numpy::ndarray &f16_state, const numpy::ndarray &u);

  numpy::ndarray output(const numpy::ndarray &f16_state, const numpy::ndarray &u);

 private:
  LowLevelController::LowLevelController llc;
};

#endif //F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_LOWLEVELCONTROLLER_H_
