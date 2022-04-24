//
// Created by elew on 4/23/22.
//
#include <f16_flight_dynamics/F16Model/LowLevelFunctions.h>
#include <math.h>

#define SIGN_FACTOR 1.1f

namespace LowLevelFunctions {

/* floor/ceil double based on sign and convert to integer */
inline int fix(double ele) {
  if (ele > 0.0)
    return int(floor(ele));
  else
    return int(ceil(ele));
}

/* sign of a number, return 0 if 0 */
inline int sign(double ele) {
  if (ele < 0.0)
    return -1;
  else if (ele == 0)
    return 0;
  else
    return 1;
}

/* rolling moment LUT */
double cl_a[12][7] = {{0.0f, -0.001f, -0.003f, -0.001f, 0.0f, 0.007f, 0.009f},
                      {0.0f, -0.004f, -0.009f, -0.01f, -0.01f, -0.01f, -0.011f},
                      {0.0f, -0.008f, -0.017f, -0.02f, -0.022f, -0.023f, -0.023f},
                      {0.0f, -0.012f, -0.024f, -0.03f, -0.034f, -0.034f, -0.037f},
                      {0.0f, -0.016f, -0.03f, -0.039f, -0.047f, -0.049f, -0.05f},
                      {0.0f, -0.022f, -0.041f, -0.054f, -0.06f, -0.063f, -0.068f},
                      {0.0f, -0.022f, -0.045f, -0.057f, -0.069f, -0.081f, -0.089f},
                      {0.0f, -0.021f, -0.04f, -0.054f, -0.067f, -0.079f, -0.088f},
                      {0.0f, -0.015f, -0.016f, -0.023f, -0.033f, -0.06f, -0.091f},
                      {0.0f, -0.008f, -0.002f, -0.006f, -0.036f, -0.058f, -0.076f},
                      {0.0f, -0.013f, -0.01f, -0.014f, -0.035f, -0.062f, -0.077f},
                      {0.0f, -0.015f, -0.019f, -0.027f, -0.035f, -0.059f, -0.076f}};

/* pitching moment LUT */
double cm_a[12][5] = {{0.205f, 0.081f, -0.046f, -0.174f, -0.259f},
                      {0.168f, 0.077f, -0.02f, -0.145f, -0.202f},
                      {0.186f, 0.107f, -0.009f, -0.121f, -0.184f},
                      {0.196f, 0.11f, -0.005f, -0.127f, -0.193f},
                      {0.213f, 0.11f, -0.006f, -0.129f, -0.199f},
                      {0.251f, 0.141f, 0.01f, -0.102f, -0.15f},
                      {0.245f, 0.127f, 0.006f, -0.097f, -0.16f},
                      {0.238f, 0.119f, -0.001f, -0.113f, -0.167f},
                      {0.252f, 0.133f, 0.014f, -0.087f, -0.104f},
                      {0.231f, 0.108f, 0.0f, -0.084f, -0.076f},
                      {0.198f, 0.081f, -0.013f, -0.069f, -0.041f},
                      {0.192f, 0.093f, 0.032f, -0.006f, -0.005f}};

/* yawing moment LUT */
double cn_a[12][7] = {{0.0f, 0.018f, 0.038f, 0.056f, 0.064f, 0.074f, 0.079f},
                      {0.0f, 0.019f, 0.042f, 0.057f, 0.077f, 0.086f, 0.09f},
                      {0.0f, 0.018f, 0.042f, 0.059f, 0.076f, 0.093f, 0.106f},
                      {0.0f, 0.019f, 0.042f, 0.058f, 0.074f, 0.089f, 0.106f},
                      {0.0f, 0.019f, 0.043f, 0.058f, 0.073f, 0.08f, 0.096f},
                      {0.0f, 0.018f, 0.039f, 0.053f, 0.057f, 0.062f, 0.08f},
                      {0.0f, 0.013f, 0.03f, 0.032f, 0.029f, 0.049f, 0.068f},
                      {0.0f, 0.007f, 0.017f, 0.012f, 0.007f, 0.022f, 0.03f},
                      {0.0f, 0.004f, 0.004f, 0.002f, 0.012f, 0.028f, 0.064f},
                      {0.0f, -0.014f, -0.035f, -0.046f, -0.034f, -0.012f, 0.015f},
                      {0.0f, -0.017f, -0.047f, -0.071f, -0.065f, -0.002f, 0.011f},
                      {0.0f, -0.033f, -0.057f, -0.073f, -0.041f, -0.013f, -0.001f}};

/* x-axis aerodynamic force coefficient LUT */
double cx_a[12][5] = {{-0.099f, -0.048f, -0.022f, -0.04f, -0.083f},
                      {-0.081f, -0.038f, -0.02f, -0.038f, -0.073f},
                      {-0.081f, -0.04f, -0.021f, -0.039f, -0.076f},
                      {-0.063f, -0.021f, -0.004f, -0.025f, -0.072f},
                      {-0.025f, 0.016f, 0.032f, 0.006f, -0.046f},
                      {0.044f, 0.083f, 0.094f, 0.062f, 0.012f},
                      {0.097f, 0.127f, 0.128f, 0.087f, 0.024f},
                      {0.113f, 0.137f, 0.13f, 0.085f, 0.025f},
                      {0.145f, 0.162f, 0.154f, 0.1f, 0.043f},
                      {0.167f, 0.177f, 0.161f, 0.11f, 0.053f},
                      {0.174f, 0.179f, 0.155f, 0.104f, 0.047f},
                      {0.166f, 0.167f, 0.138f, 0.091f, 0.04f}};
MorelliParameters MV = MorelliParameters();


/* damping coefficients LUT */
double dampp_a[12][9] = {{-0.267f, 0.882f, -0.108f, -8.8f, -0.126f, -0.36f, -7.21f, -0.38f, 0.061f},
                         {-0.11f, 0.852f, -0.108f, -25.8f, -0.026f, -0.359f, -0.54f, -0.363f, 0.052f},
                         {0.308f, 0.876f, -0.188f, -28.9f, 0.063f, -0.443f, -5.23f, -0.378f, 0.052f},
                         {1.34f, 0.958f, 0.11f, -31.4f, 0.113f, -0.42f, -5.26f, -0.386f, -0.012f},
                         {2.08f, 0.962f, 0.258f, -31.2f, 0.208f, -0.383f, -6.11f, -0.37f, -0.013f},
                         {2.91f, 0.974f, 0.226f, -30.7f, 0.23f, -0.375f, -6.64f, -0.453f, -0.024f},
                         {2.76f, 0.819f, 0.344f, -27.7f, 0.319f, -0.329f, -5.69f, -0.55f, 0.05f},
                         {2.05f, 0.483f, 0.362f, -28.2f, 0.437f, -0.294f, -6.0f, -0.582f, 0.15f},
                         {1.5f, 0.59f, 0.611f, -29.0f, 0.68f, -0.23f, -6.2f, -0.595f, 0.13f},
                         {1.49f, 1.21f, 0.529f, -29.8f, 0.1f, -0.21f, -6.4f, -0.637f, 0.158f},
                         {1.83f, -0.493f, 0.298f, -38.3f, 0.447f, -0.12f, -6.6f, -1.02f, 0.24f},
                         {1.21f, -1.04f, -2.27f, -35.3f, -0.33f, -0.1f, -6.0f, -0.84f, 0.15f}};

/* rolling moment due to ailerons LUT */
double dlda_a[12][7] = {{-0.041f, -0.041f, -0.042f, -0.04f, -0.043f, -0.044f, -0.043f},
                        {-0.052f, -0.053f, -0.053f, -0.052f, -0.049f, -0.048f, -0.049f},
                        {-0.053f, -0.053f, -0.052f, -0.051f, -0.048f, -0.048f, -0.047f},
                        {-0.056f, -0.053f, -0.051f, -0.052f, -0.049f, -0.047f, -0.045f},
                        {-0.05f, -0.05f, -0.049f, -0.048f, -0.043f, -0.042f, -0.042f},
                        {-0.056f, -0.051f, -0.049f, -0.048f, -0.042f, -0.041f, -0.037f},
                        {-0.082f, -0.066f, -0.043f, -0.042f, -0.042f, -0.02f, -0.003f},
                        {-0.059f, -0.043f, -0.035f, -0.037f, -0.036f, -0.028f, -0.013f},
                        {-0.042f, -0.038f, -0.026f, -0.031f, -0.025f, -0.013f, -0.01f},
                        {-0.038f, -0.027f, -0.016f, -0.026f, -0.021f, -0.014f, -0.003f},
                        {-0.027f, -0.023f, -0.018f, -0.017f, -0.016f, -0.011f, -0.007f},
                        {-0.017f, -0.016f, -0.014f, -0.012f, -0.011f, -0.01f, -0.008f}};

/* rolling moment due to rudder LUT */
double dldr_a[12][7] = {{0.005f, 0.007f, 0.013f, 0.018f, 0.015f, 0.021f, 0.023f},
                        {0.017f, 0.016f, 0.013f, 0.015f, 0.014f, 0.011f, 0.01f},
                        {0.014f, 0.014f, 0.011f, 0.015f, 0.013f, 0.01f, 0.011f},
                        {0.01f, 0.014f, 0.012f, 0.014f, 0.013f, 0.011f, 0.011f},
                        {-0.005f, 0.013f, 0.011f, 0.014f, 0.012f, 0.01f, 0.011f},
                        {0.009f, 0.009f, 0.009f, 0.014f, 0.011f, 0.009f, 0.01f},
                        {0.019f, 0.012f, 0.008f, 0.014f, 0.011f, 0.008f, 0.008f},
                        {0.005f, 0.005f, 0.005f, 0.015f, 0.01f, 0.01f, 0.01f},
                        {-0.0f, 0.0f, -0.002f, 0.013f, 0.008f, 0.006f, 0.006f},
                        {-0.005f, 0.004f, 0.005f, 0.011f, 0.008f, 0.005f, 0.014f},
                        {-0.011f, 0.009f, 0.003f, 0.006f, 0.007f, 0.0f, 0.02f},
                        {0.008f, 0.007f, 0.005f, 0.001f, 0.003f, 0.001f, 0.0f}};

/* yawing moment due to ailerons LUT */
double dnda_a[12][7] = {{0.001f, 0.002f, -0.006f, -0.011f, -0.015f, -0.024f, -0.022f},
                        {-0.027f, -0.014f, -0.008f, -0.011f, -0.015f, -0.01f, 0.002f},
                        {-0.017f, -0.016f, -0.006f, -0.01f, -0.014f, -0.004f, -0.003f},
                        {-0.013f, -0.016f, -0.006f, -0.009f, -0.012f, -0.002f, -0.005f},
                        {-0.012f, -0.014f, -0.005f, -0.008f, -0.011f, -0.001f, -0.003f},
                        {-0.016f, -0.019f, -0.008f, -0.006f, -0.008f, 0.003f, -0.001f},
                        {0.001f, -0.021f, -0.005f, 0.0f, -0.002f, 0.014f, -0.009f},
                        {0.017f, 0.002f, 0.007f, 0.004f, 0.002f, 0.006f, -0.009f},
                        {0.011f, 0.012f, 0.004f, 0.007f, 0.006f, -0.001f, -0.001f},
                        {0.017f, 0.016f, 0.007f, 0.01f, 0.012f, 0.004f, 0.003f},
                        {0.008f, 0.015f, 0.006f, 0.004f, 0.011f, 0.004f, -0.002f},
                        {0.016f, 0.011f, 0.006f, 0.01f, 0.011f, 0.006f, 0.001f}};

/* yawing moment due to rudder LUT */
double dndr_a[12][7] = {{-0.018f, -0.028f, -0.037f, -0.048f, -0.043f, -0.052f, -0.062f},
                        {-0.052f, -0.051f, -0.041f, -0.045f, -0.044f, -0.034f, -0.034f},
                        {-0.052f, -0.043f, -0.038f, -0.045f, -0.041f, -0.036f, -0.027f},
                        {-0.052f, -0.046f, -0.04f, -0.045f, -0.041f, -0.036f, -0.028f},
                        {-0.054f, -0.045f, -0.04f, -0.044f, -0.04f, -0.035f, -0.027f},
                        {-0.049f, -0.049f, -0.038f, -0.045f, -0.038f, -0.028f, -0.027f},
                        {-0.059f, -0.057f, -0.037f, -0.047f, -0.034f, -0.024f, -0.023f},
                        {-0.051f, -0.052f, -0.03f, -0.048f, -0.035f, -0.023f, -0.023f},
                        {-0.03f, -0.03f, -0.027f, -0.049f, -0.035f, -0.02f, -0.019f},
                        {-0.037f, -0.033f, -0.024f, -0.045f, -0.029f, -0.016f, -0.009f},
                        {-0.026f, -0.03f, -0.019f, -0.033f, -0.022f, -0.01f, -0.025f},
                        {-0.013f, -0.008f, -0.013f, -0.016f, -0.009f, -0.014f, -0.01f}};

/* z-axis aerodynamic force coefficient LUT */
double
    cz_a[12] = {0.77f, 0.241f, -0.1f, -0.415f, -0.731f, -1.053f, -1.355f, -1.646f, -1.917f, -2.12f, -2.248f, -2.229f};

/* converts velocity (vt) and altitude (alt) to mach number (amach) and dynamic pressure (qbar) */
adc_return adc(double vt, double alt) {
  double tfac, t, rho, a, amach, qbar;
  const double ro = 2.377e-3f;

  tfac = 1.0f - 0.703e-5f * alt;

  t = alt >= 35000.f ? 390.f : 519.f * tfac;

  // rho = freestream mass density
  rho = ro * pow(tfac, 4.14f);

  // a = speed of sound at the ambient conditions
  // speed of sound in a fluid is the sqrt of the quotient of the modulus of elasticity over the mass density
  a = sqrt(1.4 * 1716.3 * t);

  // amach = mach number
  amach = vt / a;

  // qbar = dynamic pressure
  qbar = .5 * rho * vt * vt;

  return adc_return({amach, qbar});
}

/*
 * rolling moment coefficient Cl
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 719
 * taken from fortran code
 */
double cl(double alpha, double beta) {
  double s, da, db, t, u, v, w, dum;
  int m, n, k, l;

  s = 0.2f * alpha;
  k = fix(s);

  if (k <= -2)
    k = -1;
  if (k >= 9)
    k = 8;

  da = s - k;
  l = k + fix(SIGN_FACTOR * sign(da));
  s = 0.2f * abs(beta);
  m = fix(s);
  if (m == 0)
    m = 1;
  if (m >= 6)
    m = 5;

  db = s - m;
  n = m + fix(SIGN_FACTOR * sign(db));
  l += 3;
  k += 3;
  m++;
  n++;

  t = cl_a[k - 1][m - 1];
  u = cl_a[k - 1][n - 1];
  v = t + abs(da) * (cl_a[l - 1][m - 1] - t);
  w = u + abs(da) * (cl_a[l - 1][n - 1] - u);
  dum = v + (w - v) * abs(db);

  return dum * sign(beta);
}

/*
 * pitching coefficient Cm
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 719
 * taken from fortran code
 */
double cm(double alpha, double el) {
  double s, de, da, t, u, v, w;
  int m, n, k, l;

  s = 0.2f * alpha;
  k = fix(s);

  if (k <= -2)
    k = -1;
  if (k >= 9)
    k = 8;

  da = s - k;
  l = k + fix(SIGN_FACTOR * sign(da));
  s = el / 12.0f;
  m = fix(s);

  if (m <= -2)
    m = -1;
  if (m >= 2)
    m = 1;

  de = s - m;
  n = m + fix(SIGN_FACTOR * sign(de));
  l += 3;
  k += 3;
  m += 3;
  n += 3;

  t = cm_a[k - 1][m - 1];
  u = cm_a[k - 1][n - 1];
  v = t + abs(da) * (cm_a[l - 1][m - 1] - t);
  w = u + abs(da) * (cm_a[l - 1][n - 1] - u);

  return v + (w - v) * sign(de);
}

/*
 * yawing moment Cn
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 720
 * taken from fortran code
 */
double cn(double alpha, double beta) {
  double s, da, db, t, u, v, w, dum;
  int m, n, k, l;

  s = 0.2f * alpha;
  k = fix(s);

  if (k <= -2)
    k = -1;
  if (k >= 9)
    k = 8;

  da = s - k;
  l = k + fix(SIGN_FACTOR * sign(da));
  s = 0.2f * abs(beta);
  m = fix(s);

  if (m == 0)
    m = 1;
  if (m >= 6)
    m = 5;

  db = s - m;
  n = m + fix(SIGN_FACTOR * sign(db));
  l += 3;
  k += 3;
  m++;
  n++;

  t = cn_a[k - 1][m - 1];
  u = cn_a[k - 1][n - 1];

  v = t + abs(da) * (cn_a[l - 1][m - 1] - t);
  w = u + abs(da) * (cn_a[l - 1][n - 1] - u);
  dum = v + (w - v) * abs(db);

  return dum * sign(beta);
}

/*
 * x-axis aerodynamic force coefficient Cx
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 718
 * taken from fortran code
 */
double cx(double alpha, double el) {
  double s, da, de, t, u, v, w, cxx;
  int m, n, k, l;

  s = 0.2f * alpha;
  k = fix(s);

  if (k <= -2)
    k = -1;
  if (k >= 9)
    k = 8;

  da = s - k;
  l = k + fix(SIGN_FACTOR * sign(da));
  s = el / 12.0f;
  m = fix(s);

  if (m <= -2)
    m = -1;
  if (m >= 2)
    m = 1;

  de = s - m;
  n = m + fix(SIGN_FACTOR * sign(de));
  l += 3;
  k += 3;
  m += 3;
  n += 3;

  t = cx_a[k - 1][m - 1];
  u = cx_a[k - 1][n - 1];

  v = t + abs(da) * (cx_a[l - 1][m - 1] - t);
  w = u + abs(da) * (cx_a[l - 1][n - 1] - u);
  cxx = v + (w - v) * abs(de);

  return cxx;
}

/*
 * sideforce coefficient Cy
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 718
 * taken from fortran code
 */
double cy(double beta, double ail, double rdr) {
  return -0.02f * beta + 0.021f * (ail / 20.0f) + 0.086f * (rdr / 30.0f);
}

/*
 * z-axis aerodynamic force coefficient Cz
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 718
 * taken from fortran code
 */
double cz(double alpha, double beta, double el) {

  double s, da;
  int k, l;

  s = 0.2f * alpha;
  k = fix(s);

  if (k <= -2)
    k = -1;
  if (k >= 9)
    k = 8;

  da = s - k;
  l = k + fix(SIGN_FACTOR * sign(da));
  l += 3;
  k += 3;
  s = cz_a[k - 1] + abs(da) * (cz_a[l - 1] - cz_a[k - 1]);

  return s * (1 - pow(beta / 57.3f, 2.0f) - 0.19f * (el / 25.0f));
}

/*
 * various damping coefficients
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 717
 * taken from fortran code
 */
boost::array<double, 9> dampp(double alpha) {
  double s, da;
  int k, l;

  s = 0.2f * alpha;
  k = fix(s);

  if (k <= -2)
    k = -1;
  if (k >= 9)
    k = 8;

  da = s - k;
  l = k + fix(SIGN_FACTOR * sign(da));
  l += 3;
  k += 3;

  /* store results in a boost array */
  boost::array<double, 9> d = {0.0};
  for (size_t i = 0; i < d.size(); i++) {
    d[i] = dampp_a[k - 1][i] + abs(da) * (dampp_a[l - 1][i] - dampp_a[k - 1][i]);
  }

  return d;
}

/*
 * rolling mom. due to ailerons
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 720
 * taken from fortran code
 */
double dlda(double alpha, double beta) {
  double s, da, db, t, u, v, w;
  int m, n, k, l;

  s = 0.2f * alpha;
  k = fix(s);

  if (k <= -2)
    k = -1;
  if (k >= 9)
    k = 8;

  da = s - k;
  l = k + fix(SIGN_FACTOR * sign(da));
  s = 0.1f * beta;
  m = fix(s);

  if (m <= -3)
    m = -2;
  if (m >= 3)
    m = 2;

  db = s - m;
  n = m + fix(SIGN_FACTOR * sign(db));
  l += 3;
  k += 3;
  m += 4;
  n += 4;

  t = dlda_a[k - 1][m - 1];
  u = dlda_a[k - 1][n - 1];

  v = t + abs(da) * (dlda_a[l - 1][m - 1] - t);
  w = u + abs(da) * (dlda_a[l - 1][n - 1] - u);
  return v + (w - v) * abs(db);

}

/*
 * rolling moment due to rudder
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 721
 * taken from fortran code
 */
double dldr(double alpha, double beta) {
  double s, da, db, t, u, v, w;
  int m, n, k, l;

  s = 0.2f * alpha;
  k = fix(s);

  if (k <= -2)
    k = -1;
  if (k >= 9)
    k = 8;

  da = s - k;
  l = k + fix(SIGN_FACTOR * sign(da));
  s = 0.1f * beta;
  m = fix(s);

  if (m <= -3)
    m = -2;
  if (m >= 3)
    m = 2;

  db = s - m;
  n = m + fix(SIGN_FACTOR * sign(db));
  l += 3;
  k += 3;
  m += 4;
  n += 4;

  t = dldr_a[k - 1][m - 1];
  u = dldr_a[k - 1][n - 1];

  v = t + abs(da) * (dldr_a[l - 1][m - 1] - t);
  w = u + abs(da) * (dldr_a[l - 1][n - 1] - u);
  return v + (w - v) * abs(db);
}

/*
 * yawing moment due to ailerons
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 721
 * taken from fortran code
 */
double dnda(double alpha, double beta) {
  double s, da, db, t, u, v, w;
  int m, n, k, l;

  s = 0.2f * alpha;
  k = fix(s);

  if (k <= -2)
    k = -1;
  if (k >= 9)
    k = 8;

  da = s - k;
  l = k + fix(SIGN_FACTOR * sign(da));
  s = 0.1f * beta;
  m = fix(s);

  if (m <= -3)
    m = -2;
  if (m >= 3)
    m = 2;

  db = s - m;
  n = m + fix(SIGN_FACTOR * sign(db));
  l += 3;
  k += 3;
  m += 4;
  n += 4;

  t = dnda_a[k - 1][m - 1];
  u = dnda_a[k - 1][n - 1];

  v = t + abs(da) * (dnda_a[l - 1][m - 1] - t);
  w = u + abs(da) * (dnda_a[l - 1][n - 1] - u);
  return v + (w - v) * abs(db);

}

/*
 * yawing moment due to rudder
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 721
 * taken from fortran code
 */
double dndr(double alpha, double beta) {
  double s, da, db, t, u, v, w;
  int m, n, k, l;

  s = 0.2f * alpha;
  k = fix(s);

  if (k <= -2)
    k = -1;
  if (k >= 9)
    k = 8;

  da = s - k;
  l = k + fix(SIGN_FACTOR * sign(da));
  s = 0.1f * beta;
  m = fix(s);

  if (m <= -3)
    m = -2;
  if (m >= 3)
    m = 2;

  db = s - m;
  n = m + fix(SIGN_FACTOR * sign(db));
  l += 3;
  k += 3;
  m += 4;
  n += 4;

  t = dndr_a[k - 1][m - 1];
  u = dndr_a[k - 1][n - 1];

  v = t + abs(da) * (dndr_a[l - 1][m - 1] - t);
  w = u + abs(da) * (dndr_a[l - 1][n - 1] - u);
  return v + (w - v) * abs(db);
}

/*
 * power vs throttle relationship
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 715
 * taken from fortran code
 */
double tgear(double thtl) {
  return thtl <= 0.77f ? 64.94f * thtl : 217.38f * thtl - 117.38f;
}

/* helper for pdot */
inline double rtau(double dp) {
  if (dp <= 25.0f) {
    return 1.0;
  } else if (dp >= 50.0f) {
    return 0.1f;
  } else {
    return 1.9f - 0.036f * dp;
  }
}

/*
 * rate of change of power
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 715
 * taken from fortran code
 */
double pdot(double p3, double p1) {
  double t, p2;

  if (p1 >= 50.0) {
    if (p3 >= 50) {
      t = 5.0;
      p2 = p1;
    } else {
      p2 = 60.0f;
      t = rtau(p2 - p3);
    }
  } else {
    if (p3 >= 50) {
      t = 5.0f;
      p2 = 40.0f;
    } else {
      p2 = p1;
      t = rtau(p2 - p3);
    }
  }
  return t * (p2 - p3);
}

double thrust_a[6][6] = {{1060.0f, 635.0f, 60.0f, -1020.0f, -2700.0f, -3600.0f},
                         {670.0f, 425.0f, 25.0f, -170.0f, -1900.0f, -1400.0f},
                         {880.0f, 690.0f, 345.0f, -300.0f, -1300.0f, -595.0f},
                         {1140.0f, 1010.0f, 755.0f, 350.0f, -247.0f, -342.0f},
                         {1500.0f, 1330.0f, 1130.0f, 910.0f, 600.0f, -200.0f},
                         {1860.0f, 1700.0f, 1525.0f, 1360.0f, 1100.0f, 700.0f}};

double thrust_b[6][6] = {{12680.0f, 12680.0f, 12610.0f, 12640.0f, 12390.0f, 11680.0f},
                         {9150.0f, 9150.0f, 9312.0f, 9839.0f, 10176.0f, 9848.0f},
                         {6200.0f, 6313.0f, 6610.0f, 7090.0f, 7750.0f, 8050.0f},
                         {3950.0f, 4040.0f, 4290.0f, 4660.0f, 5320.0f, 6100.0f},
                         {2450.0f, 2470.0f, 2600.0f, 2840.0f, 3250.0f, 3800.0f},
                         {1400.0f, 1400.0f, 1560.0f, 1660.0f, 1930.0f, 2310.0f}};

double thrust_c[6][6] = {{20000.0f, 21420.0f, 22700.0f, 24240.0f, 26070.0f, 28886.0f},
                         {15000.0f, 15700.0f, 16860.0f, 18910.0f, 21075.0f, 23319.0f},
                         {10800.0f, 11225.0f, 12250.0f, 13760.0f, 15975.0f, 18300.0f},
                         {7000.0f, 7323.0f, 8154.0f, 9285.0f, 11115.0f, 13484.0f},
                         {4000.0f, 4435.0f, 5000.0f, 5700.0f, 6860.0f, 8642.0f},
                         {2500.0f, 2600.0f, 2835.0f, 3215.0f, 3950.0f, 5057.0f}};

/*
 * engine thrust model
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 716
 * taken from fortran code
 */
double thrust(double power, double alt, double rmach) {
  double h, dh, dm, cdh, rm, s, t, tmil, tmax, thrst, tidl;
  int m, i;
  alt = alt < 0 ? 0.01f : alt;

  h = 0.0001f * alt;

  i = fix(h);

  if (i >= 5)
    i = 4;

  dh = h - i;
  rm = 5.0f * rmach;
  m = fix(rm);

  if (m >= 5)
    m = 4;
  else if (m <= 0)
    m = 0;

  dm = rm - m;
  cdh = 1 - dh;

  s = thrust_b[i][m] * cdh + thrust_b[i + 1][m] * dh;
  t = thrust_b[i][m + 1] * cdh + thrust_b[i + 1][m + 1] * dh;
  tmil = s + (t - s) * dm;

  if (power <= 50.0f) {
    s = thrust_a[i][m] * cdh + thrust_a[i + 1][m] * dh;
    t = thrust_a[i][m + 1] * cdh + thrust_a[i + 1][m + 1] * dh;
    tidl = s + (t - s) * dm;
    thrst = tidl + (tmil - tidl) * power * 0.02f;
  } else {
    s = thrust_c[i][m] * cdh + thrust_c[i + 1][m] * dh;
    t = thrust_c[i][m + 1] * cdh + thrust_c[i + 1][m + 1] * dh;
    tmax = s + (t - s) * dm;
    thrst = tmil + (tmax - tmil) * (power - 50.0f) * .02f;
  }
  return thrst;
}

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
                                double xcgref) {
  /* TODO this isn't great */
  MorelliParameters MV = MorelliParameters();

  float phat, qhat, rhat;
  phat = p * b / (2 * V);
  qhat = q * cbar / (2 * V);
  rhat = r * b / (2 * V);

  float Cx0, Cxq, Cy0, Cyp, Cyr, Cz0, Czq, Cl0, Clp, Clr, Clda, Cldr, Cm0, Cmq, Cn0, Cnp, Cnr, Cnda, Cndr;
  Cx0 = MV.a0 + MV.a1 * alpha + MV.a2 * pow(de, 2.0) + MV.a3 * de + MV.a4 * alpha * de + MV.a5 * pow(alpha, 2.0)
      + MV.a6 * pow(alpha, 3.0);
  Cxq = MV.b0 + MV.b1 * alpha + MV.b2 * pow(alpha, 2.0) + MV.b3 * pow(alpha, 3.0) + MV.b4 * pow(alpha, 4.0);
  Cy0 = MV.c0 * beta + MV.c1 * da + MV.c2 * dr;
  Cyp = MV.d0 + MV.d1 * alpha + MV.d2 * pow(alpha, 2.0) + MV.d3 * pow(alpha, 3.0);
  Cyr = MV.e0 + MV.e1 * alpha + MV.e2 * pow(alpha, 2.0) + MV.e3 * pow(alpha, 3.0);
  Cz0 = (MV.f0 + MV.f1 * alpha + MV.f2 * pow(alpha, 2.0) + MV.f3 * pow(alpha, 3.0) + MV.f4 * pow(alpha, 4.0))
      * (1 - pow(beta, 2.0)) + MV.f5 * de;
  Czq = MV.g0 + MV.g1 * alpha + MV.g2 * pow(alpha, 2.0) + MV.g3 * pow(alpha, 3.0) + MV.g4 * pow(alpha, 4.0);
  Cl0 = MV.h0 * beta + MV.h1 * alpha * beta + MV.h2 * pow(alpha, 2.0) * beta + MV.h3 * pow(beta, 2.0)
      + MV.h4 * alpha * pow(beta, 2.0) + MV.h5 *
      pow(alpha, 3.0) * beta + MV.h6 * pow(alpha, 4.0) * beta + MV.h7 * pow(alpha, 2.0) * pow(beta, 2.0);
  Clp = MV.i0 + MV.i1 * alpha + MV.i2 * pow(alpha, 2.0) + MV.i3 * pow(alpha, 3.0);
  Clr = MV.j0 + MV.j1 * alpha + MV.j2 * pow(alpha, 2.0) + MV.j3 * pow(alpha, 3.0) + MV.j4 * pow(alpha, 4.0);
  Clda = MV.k0 + MV.k1 * alpha + MV.k2 * beta + MV.k3 * pow(alpha, 2.0) + MV.k4 * alpha * beta
      + MV.k5 * pow(alpha, 2.0) * beta + MV.k6 * pow(alpha, 3.0);
  Cldr = MV.l0 + MV.l1 * alpha + MV.l2 * beta + MV.l3 * alpha * beta + MV.l4 * pow(alpha, 2.0) * beta
      + MV.l5 * pow(alpha, 3.0) * beta + MV.l6 * pow(beta, 2.0);
  Cm0 = MV.m0 + MV.m1 * alpha + MV.m2 * de + MV.m3 * alpha * de + MV.m4 * pow(de, 2.0) + MV.m5 * pow(alpha, 2.0) * de
      + MV.m6 * pow(de, 3.0) + MV.m7 *
      alpha * pow(de, 2.0);

  Cmq = MV.n0 + MV.n1 * alpha + MV.n2 * pow(alpha, 2.0) + MV.n3 * pow(alpha, 3.0) + MV.n4 * pow(alpha, 4.0)
      + MV.n5 * pow(alpha, 5.0);
  Cn0 = MV.o0 * beta + MV.o1 * alpha * beta + MV.o2 * pow(beta, 2.0) + MV.o3 * alpha * pow(beta, 2.0)
      + MV.o4 * pow(alpha, 2.0) * beta + MV.o5 *
      pow(alpha, 2.0) * pow(beta, 2.0) + MV.o6 * pow(alpha, 3.0) * beta;
  Cnp = MV.p0 + MV.p1 * alpha + MV.p2 * pow(alpha, 2.0) + MV.p3 * pow(alpha, 3.0) + MV.p4 * pow(alpha, 4.0);
  Cnr = MV.q0 + MV.q1 * alpha + MV.q2 * pow(alpha, 2.0);
  Cnda = MV.r0 + MV.r1 * alpha + MV.r2 * beta + MV.r3 * alpha * beta + MV.r4 * pow(alpha, 2.0) * beta
      + MV.r5 * pow(alpha, 3.0) * beta + MV.r6 *
      pow(alpha, 2.0) + MV.r7 * pow(alpha, 3.0) + MV.r8 * pow(beta, 3.0) + MV.r9 * alpha * pow(beta, 3.0);
  Cndr = MV.s0 + MV.s1 * alpha + MV.s2 * beta + MV.s3 * alpha * beta + MV.s4 * pow(alpha, 2.0) * beta
      + MV.s5 * pow(alpha, 2.0);

  float Cx, Cy, Cz, Cl, Cm, Cn;
  Cx = Cx0 + Cxq * qhat;
  Cy = Cy0 + Cyp * phat + Cyr * rhat;
  Cz = Cz0 + Czq * qhat;
  Cl = Cl0 + Clp * phat + Clr * rhat + Clda * da + Cldr * dr;
  Cm = Cm0 + Cmq * qhat + Cz * (xcgref - xcg);
  Cn = Cn0 + Cnp * phat + Cnr * rhat + Cnda * da + Cndr * dr - Cy * (xcgref - xcg) * (cbar / b);

  return boost::array<double, 6>({Cx, Cy, Cz, Cl, Cm, Cn});
}

}