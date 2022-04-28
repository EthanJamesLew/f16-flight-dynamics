//
// Created by elew on 4/27/22.
//
#include <python-bindings/PyTypeConvert.h>
#include <f16_flight_dynamics/F16Model/F16Types.h>
#include <python-bindings/LowLevelControllerWrapper.h>

/* python object for llc */
LowLevelControllerWrapper::LowLevelControllerWrapper() {
  llc = LowLevelController::LowLevelController();
}

/* evolution function (der) */
numpy::ndarray LowLevelControllerWrapper::dxdt(const numpy::ndarray &state, const numpy::ndarray &u) {
  F16Types::llc_state_type xd;
  F16Types::llc_state_type xa = PyTypeConvert::ndarray_to_array<double, F16Types::llc_state_type::size()>(state);
  F16Types::llc_input_type ua = PyTypeConvert::ndarray_to_array<double, F16Types::llc_input_type::size()>(u);

  llc.update(0.0, xa, xd, ua);

  return PyTypeConvert::array_to_ndarray<double, F16Types::llc_state_type::size()>(xd);
}

numpy::ndarray LowLevelControllerWrapper::output(const numpy::ndarray &state, const numpy::ndarray &u) {
  F16Types::llc_output_type xd;
  F16Types::llc_state_type xa = PyTypeConvert::ndarray_to_array<double, F16Types::llc_state_type::size()>(state);
  F16Types::llc_input_type ua = PyTypeConvert::ndarray_to_array<double, F16Types::llc_input_type::size()>(u);

  llc.output(0.0, xa, xd, ua);

  return PyTypeConvert::array_to_ndarray<double, F16Types::llc_output_type::size()>(xd);
}

