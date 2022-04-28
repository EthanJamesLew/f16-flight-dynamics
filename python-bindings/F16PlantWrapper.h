//
// Created by elew on 4/27/22.
//

#ifndef F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_F16PLANTWRAPPER_H_
#define F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_F16PLANTWRAPPER_H_

#include <f16_flight_dynamics/F16Model/F16Plant.h>

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
namespace py = boost::python;
namespace numpy = boost::python::numpy;

/* python object for plant dynamics */
class F16PlantWrapper {
 public:
  /* evolution function (der) */
  numpy::ndarray dxdt(const numpy::ndarray &x, const numpy::ndarray &uinput);
};

#endif //F16_FLIGHT_DYNAMICS_PYTHON_BINDINGS_F16PLANTWRAPPER_H_
