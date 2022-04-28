//
// Created by elew on 4/27/22.
//

#ifndef F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_COMPONENT_H_
#define F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_COMPONENT_H_

namespace Component {

template <typename In, typename State, typename Out>
class ComponentBase {
 public:
  /* component model update function
   *
   * this method is what allows the system to "evolve". In discrete time, this would
   * be the next step. In continuous time, this would be the state derivative.
   */
  virtual void update(double time, const State &state, State &state_up, const In &input) = 0;

  /* component model output function
   *
   * this method is what the system outputs, bsed on its input and state.
   */
  virtual void output(double time, const State &state, Out &output, const In &input) = 0;

  /* TODO: a solve system function? ... */

  void print();
};

}

#endif //F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_COMPONENT_H_
