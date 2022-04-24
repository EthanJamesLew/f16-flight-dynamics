//
// Created by elew on 4/23/22.
//

#ifndef F16_FLIGHT_DYNAMICS_LOWLEVEL_H
#define F16_FLIGHT_DYNAMICS_LOWLEVEL_H

#include <boost/array.hpp>
#define SIGN_FACTOR 1.1f

namespace LowLevelFunctions {

/* return type for adc function */
struct adc_return {
  float amach;
  float qbar;
};

/* converts velocity (vt) and altitude (alt) to mach number (amach) and dynamic pressure (qbar) */
adc_return adc(float vt, float alt);

/*
 * rolling moment coefficient Cl
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 719
 * taken from fortran code
 */
float cl(float alpha, float beta);

/*
 * pitching coefficient Cm
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 719
 * taken from fortran code
 */
float cm(float alpha, float el);

/*
 * yawing moment Cn
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 720
 * taken from fortran code
 */
float cn(float alpha, float beta);

/*
 * x-axis aerodynamic force coefficient Cx
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 718
 * taken from fortran code
 */
float cx(float alpha, float el);

/*
 * sideforce coefficient Cy
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 718
 * taken from fortran code
 */
float cy(float beta, float ail, float rdr);

/*
 * z-axis aerodynamic force coefficient Cz
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 718
 * taken from fortran code
 */
float cz(float alpha, float beta, float el);

/*
 * various damping coefficients
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 717
 * taken from fortran code
 */
boost::array<float, 9> dampp(float alpha);

/*
 * rolling mom. due to ailerons
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 720
 * taken from fortran code
 */
float dlda(float alpha, float beta);

/*
 * rolling moment due to rudder
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 721
 * taken from fortran code
 */
float dldr(float alpha, float beta);

/*
 * yawing moment due to ailerons
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 721
 * taken from fortran code
 */
float dnda(float alpha, float beta);

/*
 * yawing moment due to rudder
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 721
 * taken from fortran code
 */
float dndr(float alpha, float beta);


/*
 * power vs throttle relationship
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 715
 * taken from fortran code
 */
float tgear(float thtl);

/*
 * rate of change of power
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 715
 * taken from fortran code
 */
float pdot(float p3, float p1);

/*
 * engine thrust model
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 716
 * taken from fortran code
 */
float thrust(float power, float alt, float rmach);

}

#endif //F16_FLIGHT_DYNAMICS_LOWLEVEL_H
