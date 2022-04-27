//
// Created by elew on 4/27/22.
//
#include <python-bindings/PyTypeConvert.h>
#include <boost/python/numpy.hpp>
#include <f16_flight_dynamics/F16Model/F16Plant.h>

namespace py = boost::python;
namespace numpy = boost::python::numpy;

/* python object for plant dynamics */
class F16PlantWrapper {
 public:
  /* evolution function (der) */
  numpy::ndarray dxdt(const numpy::ndarray &x, const numpy::ndarray &uinput) {
    F16Components::f16_full_type xd;
    F16Components::f16_state_type xa = PyTypeConvert::ndarray_to_array<double, 13>(x);
    F16Components::f16_input_type ua = PyTypeConvert::ndarray_to_array<double, 4>(uinput);

    F16Components::F16Plant plant;
    plant.subf16_model(xa, ua, xd);

    return PyTypeConvert::array_to_ndarray<double, 17>(xd);
  }
};

