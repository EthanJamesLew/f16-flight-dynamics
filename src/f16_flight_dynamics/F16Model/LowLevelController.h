//
// Created by elew on 4/26/22.
//

#ifndef F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_LOWLEVELCONTROLLER_H_
#define F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_LOWLEVELCONTROLLER_H_
#include <boost/array.hpp>

#include <boost/numeric/ublas/tensor.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include <f16_flight_dynamics/F16Model/F16Types.h>
#include <f16_flight_dynamics/Component/Component.h>

using namespace boost::numeric::ublas;

using tensord = tensor<double>;
using matrixd = matrix<double>;
using vectord = vector<double>;

namespace LowLevelController {

using namespace F16Types;

class LowLevelController : public Component::ComponentBase<llc_input_type, llc_state_type, llc_output_type> {
 public:
  /* trim constructor */
  LowLevelController(f16_state_type state_trim, llc_output_type control_trim);

  /* defaults */
  LowLevelController();

  /* llc u_deg output */
  void output(double time, const llc_state_type &state, llc_output_type &output, const llc_input_type &input);

  /* llc state update */
  void update(double time, const llc_state_type &state, llc_state_type &state_up, const llc_input_type &input);

 private:
  f16_state_type xequil;
  llc_output_type uequil;
  matrixd lqr_gain;

  inline matrixd reorder_xctrl(const f16_state_type &state, const llc_state_type &cstate);
  inline matrixd build_lqr();

};

}

#endif //F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_LOWLEVELCONTROLLER_H_
