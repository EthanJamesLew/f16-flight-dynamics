//
// Created by elew on 4/23/22.
//

#ifndef F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_F16PLANT_H_
#define F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_F16PLANT_H_

#include <boost/array.hpp>

namespace F16Components {
/* sized array to store aircraft state */
typedef boost::array<double, 13> f16_state_type;

/* sized array to store aircraft input */
typedef boost::array<double, 4> f16_input_type;

/*
 * a data structure of constants - maybe want to have the user configure this?
 */
struct F16PlantParameters {
  const float xcg = 0.35f;
  const float s = 300.0f;
  const float b = 30.0f;
  const float cbar = 11.32f;
  const float rm = 1.57e-3f;
  const float xcgr = .35f;
  const float he = 160.0f;
  const float c1 = -.770f;
  const float c2 = .02755f;
  const float c3 = 1.055e-4f;
  const float c4 = 1.642e-6f;
  const float c5 = .9604f;
  const float c6 = 1.759e-2f;
  const float c7 = 1.792e-5f;
  const float c8 = -.7336f;
  const float c9 = 1.587e-5f;
  const float rtod = 57.29578f;
  const float g = 32.17f;
};

F16PlantParameters F16Val = F16PlantParameters();

/* enumerate the possible engine models */
enum EngineModelType {
  STEVENS,
  MORELLI
};

/*
 * F16 Aircraft Plant Dynamics (no control)
 *
 * The F16 is an unstable "fly by wire" aircraft. When uncontrolled, expect unstable
 * behavior.
 */
class F16Plant {
 public:
  /*
   * output aircraft state vector derivative for a given input
   * The reference for the model is Appendix A of Stevens & Lewis
   */
  void subf16_model(const f16_state_type &x,
                    const f16_input_type &uinput,
                    f16_state_type &dxdt,
                    EngineModelType model_type = STEVENS,
                    bool adjust_cy = true);

};
}
#endif //F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_F16PLANT_H_
