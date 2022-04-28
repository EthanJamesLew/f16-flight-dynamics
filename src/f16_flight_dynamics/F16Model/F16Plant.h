//
// Created by elew on 4/23/22.
//

#ifndef F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_F16PLANT_H_
#define F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_F16PLANT_H_

#include <boost/array.hpp>

#include <f16_flight_dynamics/F16Model/F16Types.h>

namespace F16Components {
using namespace F16Types;

/*
 * a data structure of constants - maybe want to have the user configure this?
 */
struct F16PlantParameters {
  const double xcg = 0.35f;
  const double s = 300.0f;
  const double b = 30.0f;
  const double cbar = 11.32f;
  const double rm = 1.57e-3f;
  const double xcgr = .35f;
  const double he = 160.0f;
  const double c1 = -.770f;
  const double c2 = .02755f;
  const double c3 = 1.055e-4f;
  const double c4 = 1.642e-6f;
  const double c5 = .9604f;
  const double c6 = 1.759e-2f;
  const double c7 = 1.792e-5f;
  const double c8 = -.7336f;
  const double c9 = 1.587e-5f;
  const double rtod = 57.29578f;
  const double g = 32.17f;
};

/* enumerate the possible engine models */
enum EngineModelType {
  STEVENS, MORELLI
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
                    f16_full_type &dxdt,
                    EngineModelType model_type = MORELLI,
                    bool adjust_cy = true);

};
}
#endif //F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_F16PLANT_H_
