//
// Created by elew on 4/26/22.
//
#include <f16_flight_dynamics/F16Model/LowLevelController.h>

#include <f16_flight_dynamics/F16Model/F16Types.h>

namespace LowLevelController {

struct CtrlLimits {
  const float ThrottleMax = 1.0;
  const float ThrottleMin = 0.0;
  const float ElevatorMaxDeg = 25.0;
  const float ElevatorMinDeg = -25.0;
  const float AileronMaxDeg = 21.5;
  const float AileronMinDeg = -21.5;
  const float RudderMaxDeg = 30.0;
  const float RudderMinDeg = -30.0;
  const float MaxBankDeg = 60.0;
  const float NzMax = 6.0;
  const float NzMin = -1.0;
};


CtrlLimits CL = CtrlLimits();

/* state index names */
enum F16StateIdxs {
  vt = 0,
  alpha = 1,
  beta = 2,
  phi = 3,
  theta = 4,
  psi = 5,
  p = 6,
  q = 7,
  r = 8,
  pn = 9,
  pe = 10,
  h = 11,
  power = 12,
  nz = 13,
  ny = 14,
  az = 15,
  ay = 16
} F16StateIdxs;

/* aircraft trim state */
//f16_state_type default_xequil = {502.0, 0.0389, 0.0, 0.0, 0.0389, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 9.0567};
f16_state_type default_xequil = {502.0, 0.03887505597600522, 0.0, 0.0, 0.03887505597600522, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0,
                                 9.05666543872074};

/* control trim input */
//llc_output_type default_uequil = {0.1395, -0.7496, 0.0, 0.0};
llc_output_type default_uequil = {0.13946204864060271, -0.7495784725828754, 0.0, 0.0};

LowLevelController::LowLevelController() {
  xequil = default_xequil;
  uequil = default_uequil;

  /* build matrix */
  lqr_gain = build_lqr();
}

LowLevelController::LowLevelController(f16_state_type state_trim, llc_output_type control_trim) {
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

void LowLevelController::output(double time, const llc_state_type &state, llc_output_type &output, const llc_input_type &input) {
  /* split the input */
  f16_full_type f16_state;
  llc_output_type u_ref;
  std::copy(input.begin(), input.begin() + f16_full_type::size(), f16_state.begin());
  std::copy(input.begin() + f16_full_type::size(), input.end(), u_ref.begin());

  /* apply trim to a new state variable */
  /* you can probably save yourself a copy... */
  f16_state_type xdelta;
  std::copy(f16_state.begin(), f16_state.begin() + f16_state_type::size(), xdelta.begin());
  for (size_t i = 0; i < xdelta.size(); i++)
    xdelta[i] = f16_state[i] - xequil[i];

  /* LQR feedback control
   * reorder states to match controller
   */
  auto xvec = reorder_xctrl(xdelta, state);

  auto uvec = -prod(lqr_gain, xvec);

  output[1] = uvec(0, 0);
  output[2] = uvec(1, 0);
  output[3] = uvec(2, 0);

  output[0] = u_ref[3];

  for (size_t i = 0; i < uequil.size(); i++)
    output[i] += uequil[i];

  output[0] = fmax(fmin(output[0], CL.ThrottleMax), CL.ThrottleMin);
  output[1] = fmax(fmin(output[1], CL.ElevatorMaxDeg), CL.ElevatorMinDeg);
  output[2] = fmax(fmin(output[2], CL.AileronMaxDeg), CL.AileronMinDeg);
  output[3] = fmax(fmin(output[3], CL.RudderMaxDeg), CL.RudderMinDeg);
}

void LowLevelController::update(double time, const llc_state_type &state, llc_state_type &state_up, const llc_input_type &input) {
  /* split the input */
  f16_full_type f16_state;
  llc_output_type u_ref;
  std::copy(input.begin(), input.begin() + f16_full_type::size(), f16_state.begin());
  std::copy(input.begin() + f16_full_type::size(), input.end(), u_ref.begin());

  /* apply trim to a new state variable */
  f16_state_type xdelta;
  std::copy(f16_state.begin(), f16_state.begin() + f16_state_type::size(), xdelta.begin());
  for (size_t i = 0; i < xdelta.size(); i++)
    xdelta[i] = f16_state[i] - xequil[i];

  double ps = xdelta[p] * cos(xdelta[alpha]) + xdelta[r] * sin(xdelta[alpha]);
  double nyr = f16_state[ny] + f16_state[r];

  state_up[0] = f16_state[F16StateIdxs::nz] - u_ref[0];
  state_up[1] = ps - u_ref[1];
  state_up[2] = nyr - u_ref[2];
}

inline matrixd LowLevelController::reorder_xctrl(const f16_state_type &state, const llc_state_type &cstate) {
  auto r = matrixd(8, 1);
  r(0, 0) = state[1];
  r(1, 0) = state[7];
  r(2, 0) = cstate[0];
  r(3, 0) = state[2];
  r(4, 0) = state[6];
  r(5, 0) = state[8];
  r(6, 0) = cstate[1];
  r(7, 0) = cstate[2];
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

  /* // this is AeroBenchVV
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
   */

  // this is CSAF
  gain(1, 3) = 30.511411060051355;
  gain(1, 4) = -5.705403676148551;
  gain(1, 5) = -9.310178739319714;
  gain(1, 6) = -33.97951344944365;
  gain(1, 7) = -10.652777306717681;
  gain(2, 3) = -22.65901530645282;
  gain(2, 4) = 1.3193739204719577;
  gain(2, 5) = -14.2051751789712;
  gain(2, 6) = 6.7374079391328845;
  gain(2, 7) = -53.726328142239225;

  return gain;
}

}