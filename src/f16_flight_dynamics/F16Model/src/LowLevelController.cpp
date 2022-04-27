//
// Created by elew on 4/26/22.
//
#include <f16_flight_dynamics/F16Model/LowLevelController.h>

#include <f16_flight_dynamics/F16Model/F16Types.h>

namespace LowLevelController {

LowLevelController::LowLevelController() {
  xequil = default_xequil;
  uequil = default_uequil;

  /* build matrix */
  lqr_gain = build_lqr();
}

LowLevelController::LowLevelController(f16_state_type state_trim, llc_input_type control_trim) {
  xequil = state_trim;
  uequil = control_trim;
}

void printMatrix(const matrix<double> &m) {
  for (unsigned i = 0; i < m.size1(); ++i) {
    std::cout << "| ";
    for (unsigned j = 0; j < m.size2(); ++j) {
      std::cout << m(i, j) << " | ";
    }
    std::cout << "|" << std::endl;
  }
}

llc_input_type LowLevelController::output(const llc_input_type &u_ref, const f16_full_type &f16_state) {
  llc_input_type u_deg;

  /* apply trim to a new state variable */
  f16_state_type xdelta{};
  for (size_t i = 0; i < xdelta.size(); i++)
    xdelta[i] = f16_state[i] - xequil[i];

  /* LQR feedback control
   * reorder states to match controller
   */
  auto xvec = reorder_xctrl(f16_state);

  auto uvec = -1.0 * prod(lqr_gain, xvec);
  u_deg[1] = uvec(0, 0);
  u_deg[2] = uvec(1, 0);
  u_deg[3] = uvec(2, 0);

  u_deg[0] = u_ref[3];

  /* TODO: apply limits */

  return u_deg;
}

llc_state_type LowLevelController::dxdt(const llc_input_type &u_ref, const f16_full_type &f16_state) {
  /* apply trim to a new state variable */
  f16_state_type xdelta{};
  for (size_t i = 0; i < xdelta.size(); i++) {
    xdelta[i] = f16_state[i] - xequil[i];
  }
  double ps = xdelta[p] * cos(xdelta[alpha]) + xdelta[r] * sin(xdelta[alpha]);
  double nyr = f16_state[ny] + f16_state[r];
  llc_state_type s = {f16_state[F16StateIdxs::nz] - u_ref[0], ps - u_ref[1], nyr - u_ref[2]};
  return s;
}

inline matrixd LowLevelController::reorder_xctrl(const f16_full_type &state) {
  auto r = matrixd(8, 1);
  r(0, 0) = state[1];
  r(1, 0) = state[7];
  r(2, 0) = state[13];
  r(3, 0) = state[2];
  r(4, 0) = state[6];
  r(5, 0) = state[8];
  r(6, 0) = state[14];
  r(7, 0) = state[15];
  return r;
}

inline matrixd LowLevelController::build_lqr() {
  /* build matrix */
  matrixd gain(3, 8);
  for (unsigned i = 0; i < gain.size1(); ++i) {
    for (unsigned j = 0; j < gain.size2(); ++j) {
      gain(i, j) = 0.0;
    }
  }
  //auto gain = matrixd(3, 8);

  gain(0, 0) = -156.8801506723475;
  gain(0, 1) = -31.037008068526642;
  gain(0, 2) = -38.72983346216317;

  gain(1, 3) = 37.84483;
  gain(1, 4) = -25.40956;
  gain(1, 5) = -6.82876;
  gain(1, 6) = -332.88343;
  gain(1, 7) = -17.15997;
  gain(2, 3) = -23.91233;
  gain(2, 4) = 5.69968;
  gain(2, 5) = -21.63431;
  gain(2, 6) = 64.49490;
  gain(2, 7) = -88.36203;

  return gain;
}

}