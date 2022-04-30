//
// Created by elew on 4/29/22.
//
#include <gtest/gtest.h>
#include <f16_flight_dynamics/F16Model/F16Plant.h>

TEST(F16Plant, Construct) {
  F16Components::F16Plant plant = F16Components::F16Plant();
  F16Components::f16_input_type in;
  F16Components::f16_state_type x{0};
  F16Components::f16_full_type xd{0};

  /* fill with zeros */
  std::fill(in.begin(), in.end(), 0.1);
  std::fill(x.begin(), x.end(), 0.1);
  std::fill(xd.begin(), xd.end(), 0.0);

  /* hmmm... this isn't good */
  std::for_each(xd.begin(), xd.end(), [](double &d) { ASSERT_EQ(d, 0.0); });

  plant.subf16_model(x, in, xd);

  std::for_each(xd.begin(), xd.end(), [](double &d) { ASSERT_NE(d, 0.0); });
}