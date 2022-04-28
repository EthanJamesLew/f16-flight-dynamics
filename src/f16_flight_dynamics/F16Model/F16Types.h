//
// Created by elew on 4/26/22.
//
#ifndef F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_AIRCRAFTTYPES_H_
#define F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_AIRCRAFTTYPES_H_
#include <boost/array.hpp>

namespace F16Types {

/* sized array to store aircraft state */
typedef boost::array<double, 13> f16_state_type;

/* sized array to store aircraft output */
typedef boost::array<double, 4> f16_output_type;

/* sized array to store aircraft state + output */
typedef boost::array<double, 17> f16_full_type;

/* sized array to store aircraft input */
typedef boost::array<double, 4> f16_input_type;

/* sized array of llc i/o */
typedef boost::array<double, 21> llc_input_type;
typedef boost::array<double, 4> llc_output_type;

/* sized array of llc state */
typedef boost::array<double, 3> llc_state_type;
}

#endif //F16_FLIGHT_DYNAMICS_SRC_F16_FLIGHT_DYNAMICS_F16MODEL_AIRCRAFTTYPES_H_
