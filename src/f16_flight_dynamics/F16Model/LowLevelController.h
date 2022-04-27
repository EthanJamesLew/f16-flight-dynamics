//
// Created by elew on 4/26/22.
//

#ifndef F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_LOWLEVELCONTROLLER_H_
#define F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_LOWLEVELCONTROLLER_H_
#include <boost/array.hpp>

#include <boost/numeric/ublas/tensor.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
using namespace boost::numeric::ublas;

using tensord = tensor<double>;
using matrixd = matrix<double>;
using vectord = vector<double>;

namespace LowLevelController {
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

/* sized array to store aircraft state */
typedef boost::array<double, 13> f16_state_type;

/* sized array to store aircraft state + output */
typedef boost::array<double, 17> f16_full_type;

/* sized array to store aircraft input */
typedef boost::array<double, 4> f16_input_type;


/* sized array of llc i/o */
typedef boost::array<double, 4> llc_input_type;

/* sized array of llc state */
typedef boost::array<double, 3> llc_state_type;

/* aircraft trim state */
f16_state_type default_xequil = {502.0, 0.0389, 0.0, 0.0, 0.0389, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 1000.0, 9.0567};

/* control trim input */
llc_input_type default_uequil = {0.1395, -0.7496, 0.0, 0.0};

class LowLevelController {
 public:
  /* trim constructor */
  LowLevelController(f16_state_type state_trim, llc_input_type control_trim);

  /* defaults */
  LowLevelController();

  /* llc u_deg output */
  llc_input_type output(const llc_input_type &u_ref, const f16_full_type &f16_state);

  /* llc state update */
  llc_state_type dxdt(const llc_input_type &u_ref, const f16_full_type &f_16_state);

 private:
  f16_state_type xequil;
  llc_input_type uequil;
  matrixd lqr_gain;

  inline matrixd reorder_xctrl(const f16_full_type &state);
  inline matrixd build_lqr();

};

}

#endif //F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_LOWLEVELCONTROLLER_H_
