//
// Created by elew on 4/27/22.
//
#include <python-bindings/PyTypeConvert.h>
#include <python-bindings/F16PlantWrapper.h>

  /* evolution function (der) */
numpy::ndarray F16PlantWrapper::dxdt(const numpy::ndarray &x, const numpy::ndarray &uinput) {
  F16Components::f16_full_type xd;
  F16Components::f16_state_type xa = PyTypeConvert::ndarray_to_array<double, 13>(x);
  F16Components::f16_input_type ua = PyTypeConvert::ndarray_to_array<double, 4>(uinput);

  F16Components::F16Plant plant;
  plant.subf16_model(xa, ua, xd);

  return PyTypeConvert::array_to_ndarray<double, 17>(xd);
  }
