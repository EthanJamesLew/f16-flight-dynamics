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
  double amach;
  double qbar;
};

/* converts velocity (vt) and altitude (alt) to mach number (amach) and dynamic pressure (qbar) */
adc_return adc(double vt, double alt);

/*
 * rolling moment coefficient Cl
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 719
 * taken from fortran code
 */
double cl(double alpha, double beta);

/*
 * pitching coefficient Cm
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 719
 * taken from fortran code
 */
double cm(double alpha, double el);

/*
 * yawing moment Cn
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 720
 * taken from fortran code
 */
double cn(double alpha, double beta);

/*
 * x-axis aerodynamic force coefficient Cx
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 718
 * taken from fortran code
 */
double cx(double alpha, double el);

/*
 * sideforce coefficient Cy
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 718
 * taken from fortran code
 */
double cy(double beta, double ail, double rdr);

/*
 * z-axis aerodynamic force coefficient Cz
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 718
 * taken from fortran code
 */
double cz(double alpha, double beta, double el);

/*
 * various damping coefficients
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 717
 * taken from fortran code
 */
boost::array<double, 9> dampp(double alpha);

/*
 * rolling mom. due to ailerons
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 720
 * taken from fortran code
 */
double dlda(double alpha, double beta);

/*
 * rolling moment due to rudder
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 721
 * taken from fortran code
 */
double dldr(double alpha, double beta);

/*
 * yawing moment due to ailerons
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 721
 * taken from fortran code
 */
double dnda(double alpha, double beta);

/*
 * yawing moment due to rudder
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 721
 * taken from fortran code
 */
double dndr(double alpha, double beta);

/*
 * power vs throttle relationship
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 715
 * taken from fortran code
 */
double tgear(double thtl);

/*
 * rate of change of power
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 715
 * taken from fortran code
 */
double pdot(double p3, double p1);

/*
 * engine thrust model
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 716
 * taken from fortran code
 */
double thrust(double power, double alt, double rmach);

struct MorelliParameters {
  const double a0 = -1.943367e-2;
  const double a1 = 2.136104e-1;
  const double a2 = -2.903457e-1;
  const double a3 = -3.348641e-3;
  const double a4 = -2.060504e-1;
  const double a5 = 6.988016e-1;
  const double a6 = -9.035381e-1;
  const double b0 = 4.833383e-1;
  const double b1 = 8.644627;
  const double b2 = 1.131098e1;
  const double b3 = -7.422961e1;
  const double b4 = 6.075776e1;
  const double c0 = -1.145916;
  const double c1 = 6.016057e-2;
  const double c2 = 1.642479e-1;
  const double d0 = -1.006733e-1;
  const double d1 = 8.679799e-1;
  const double d2 = 4.260586;
  const double d3 = -6.923267;
  const double e0 = 8.071648e-1;
  const double e1 = 1.189633e-1;
  const double e2 = 4.177702;
  const double e3 = -9.162236;
  const double f0 = -1.378278e-1;
  const double f1 = -4.211369;
  const double f2 = 4.775187;
  const double f3 = -1.026225e1;
  const double f4 = 8.399763;
  const double f5 = -4.354000e-1;
  const double g0 = -3.054956e1;
  const double g1 = -4.132305e1;
  const double g2 = 3.292788e2;
  const double g3 = -6.848038e2;
  const double g4 = 4.080244e2;
  const double h0 = -1.05853e-1;
  const double h1 = -5.776677e-1;
  const double h2 = -1.672435e-2;
  const double h3 = 1.357256e-1;
  const double h4 = 2.172952e-1;
  const double h5 = 3.464156;
  const double h6 = -2.835451;
  const double h7 = -1.098104;
  const double i0 = -4.126806e-1;
  const double i1 = -1.189974e-1;
  const double i2 = 1.247721;
  const double i3 = -7.391132e-1;
  const double j0 = 6.250437e-2;
  const double j1 = 6.067723e-1;
  const double j2 = -1.101964;
  const double j3 = 9.100087;
  const double j4 = -1.192672e1;
  const double k0 = -1.463144e-1;
  const double k1 = -4.07391e-2;
  const double k2 = 3.253159e-2;
  const double k3 = 4.851209e-1;
  const double k4 = 2.978850e-1;
  const double k5 = -3.746393e-1;
  const double k6 = -3.213068e-1;
  const double l0 = 2.635729e-2;
  const double l1 = -2.192910e-2;
  const double l2 = -3.152901e-3;
  const double l3 = -5.817803e-2;
  const double l4 = 4.516159e-1;
  const double l5 = -4.928702e-1;
  const double l6 = -1.579864e-2;
  const double m0 = -2.029370e-2;
  const double m1 = 4.660702e-2;
  const double m2 = -6.012308e-1;
  const double m3 = -8.062977e-2;
  const double m4 = 8.320429e-2;
  const double m5 = 5.018538e-1;
  const double m6 = 6.378864e-1;
  const double m7 = 4.226356e-1;
  const double n0 = -5.19153;
  const double n1 = -3.554716;
  const double n2 = -3.598636e1;
  const double n3 = 2.247355e2;
  const double n4 = -4.120991e2;
  const double n5 = 2.411750e2;
  const double o0 = 2.993363e-1;
  const double o1 = 6.594004e-2;
  const double o2 = -2.003125e-1;
  const double o3 = -6.233977e-2;
  const double o4 = -2.107885;
  const double o5 = 2.141420;
  const double o6 = 8.476901e-1;
  const double p0 = 2.677652e-2;
  const double p1 = -3.298246e-1;
  const double p2 = 1.926178e-1;
  const double p3 = 4.013325;
  const double p4 = -4.404302;
  const double q0 = -3.698756e-1;
  const double q1 = -1.167551e-1;
  const double q2 = -7.641297e-1;
  const double r0 = -3.348717e-2;
  const double r1 = 4.276655e-2;
  const double r2 = 6.573646e-3;
  const double r3 = 3.535831e-1;
  const double r4 = -1.373308;
  const double r5 = 1.237582;
  const double r6 = 2.302543e-1;
  const double r7 = -2.512876e-1;
  const double r8 = 1.588105e-1;
  const double r9 = -5.199526e-1;
  const double s0 = -8.115894e-2;
  const double s1 = -1.156580e-2;
  const double s2 = 2.514167e-2;
  const double s3 = 2.038748e-1;
  const double s4 = -3.337476e-1;
  const double s5 = 1.004297e-1;
};

/*
 * morelli model engine dynamics
 */
boost::array<double, 6> morelli(double alpha,
                                double beta,
                                double de,
                                double da,
                                double dr,
                                double p,
                                double q,
                                double r,
                                double cbar,
                                double b,
                                double V,
                                double xcg,
                                double xcgref);

}

#endif //F16_FLIGHT_DYNAMICS_LOWLEVEL_H
