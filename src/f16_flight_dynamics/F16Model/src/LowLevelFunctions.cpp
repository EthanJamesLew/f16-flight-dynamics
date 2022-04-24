//
// Created by elew on 4/23/22.
//
#include <f16_flight_dynamics/F16Model/LowLevelFunctions.h>
#include <math.h>

#define SIGN_FACTOR 1.1f

namespace LowLevelFunctions {

/* floor/ceil float based on sign and convert to integer */
inline int fix(float ele) {
  if (ele > 0)
    return int(floor(ele));
  else
    return int(ceil(ele));
}

/* sign of a number, return 0 if 0 */
inline int sign(float ele) {
  if (ele < 0)
    return -1;
  else if (ele == 0)
    return 0;
  else
    return 1;
}

/* rolling moment LUT */
float cl_a[12][7] = {{0.0f, -0.001f, -0.003f, -0.001f, 0.0f, 0.007f, 0.009f},
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
float cm_a[12][5] = {{0.205f, 0.081f, -0.046f, -0.174f, -0.259f},
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
float cn_a[12][7] = {{0.0f, 0.018f, 0.038f, 0.056f, 0.064f, 0.074f, 0.079f},
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
float cx_a[12][5] = {{-0.099f, -0.048f, -0.022f, -0.04f, -0.083f},
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

/* damping coefficients LUT */
float dampp_a[12][9] = {{-0.267f, 0.882f, -0.108f, -8.8f, -0.126f, -0.36f, -7.21f, -0.38f, 0.061f},
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
float dlda_a[12][7] = {{-0.041f, -0.041f, -0.042f, -0.04f, -0.043f, -0.044f, -0.043f},
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
float dldr_a[12][7] = {{0.005f, 0.007f, 0.013f, 0.018f, 0.015f, 0.021f, 0.023f},
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
float dnda_a[12][7] = {{0.001f, 0.002f, -0.006f, -0.011f, -0.015f, -0.024f, -0.022f},
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
float dndr_a[12][7] = {{-0.018f, -0.028f, -0.037f, -0.048f, -0.043f, -0.052f, -0.062f},
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
float cz_a[12] = {0.77f, 0.241f, -0.1f, -0.415f, -0.731f, -1.053f, -1.355f, -1.646f, -1.917f, -2.12f, -2.248f, -2.229f};

/* converts velocity (vt) and altitude (alt) to mach number (amach) and dynamic pressure (qbar) */
adc_return adc(float vt, float alt) {
  float tfac, t, rho, a, amach, qbar;
  const float ro = 2.377e-3f;

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
float cl(float alpha, float beta) {
  float s, da, db, t, u, v, w, dum;
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
float cm(float alpha, float el) {
  float s, de, da, t, u, v, w;
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

  t = cl_a[k - 1][m - 1];
  u = cl_a[k - 1][n - 1];
  v = t + abs(da) * (cm_a[l - 1][m - 1] - t);
  w = u + abs(da) * (cm_a[l - 1][n - 1] - u);

  return v + (w - v) * sign(de);
}

/*
 * yawing moment Cn
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 720
 * taken from fortran code
 */
float cn(float alpha, float beta) {
  float s, da, db, t, u, v, w, dum;
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
float cx(float alpha, float el) {
  float s, da, de, t, u, v, w, cxx;
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
float cy(float beta, float ail, float rdr) {
  return -0.02f * beta + 0.021f * (ail / 20.0f) + 0.086f * (rdr / 30.0f);
}

/*
 * z-axis aerodynamic force coefficient Cz
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 718
 * taken from fortran code
 */
float cz(float alpha, float beta, float el) {

  float s, da;
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
boost::array<float, 9> dampp(float alpha) {
  float s, da;
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
  boost::array<float, 9> d = {0.0f};
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
float dlda(float alpha, float beta) {
  float s, da, db, t, u, v, w;
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
float dldr(float alpha, float beta) {
  float s, da, db, t, u, v, w;
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
float dnda(float alpha, float beta) {
  float s, da, db, t, u, v, w;
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
float dndr(float alpha, float beta) {
  float s, da, db, t, u, v, w;
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
float tgear(float thtl) {
  return thtl <= 0.77f ? 64.94f * thtl : 217.38f * thtl - 117.38f;
}

/* helper for pdot */
inline float rtau(float dp) {
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
float pdot(float p3, float p1) {
  float t, p2;

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

float thrust_a[6][6] = {{1060.0f, 635.0f, 60.0f, -1020.0f, -2700.0f, -3600.0f},
                        {670.0f, 425.0f, 25.0f, -170.0f, -1900.0f, -1400.0f},
                        {880.0f, 690.0f, 345.0f, -300.0f, -1300.0f, -595.0f},
                        {1140.0f, 1010.0f, 755.0f, 350.0f, -247.0f, -342.0f},
                        {1500.0f, 1330.0f, 1130.0f, 910.0f, 600.0f, -200.0f},
                        {1860.0f, 1700.0f, 1525.0f, 1360.0f, 1100.0f, 700.0f}};

float thrust_b[6][6] = {{1060.0f, 635.0f, 60.0f, -1020.0f, -2700.0f, -3600.0f},
                        {670.0f, 425.0f, 25.0f, -170.0f, -1900.0f, -1400.0f},
                        {880.0f, 690.0f, 345.0f, -300.0f, -1300.0f, -595.0f},
                        {1140.0f, 1010.0f, 755.0f, 350.0f, -247.0f, -342.0f},
                        {1500.0f, 1330.0f, 1130.0f, 910.0f, 600.0f, -200.0f},
                        {1860.0f, 1700.0f, 1525.0f, 1360.0f, 1100.0f, 700.0f}};

float thrust_c[6][6] = {{1060.0f, 635.0f, 60.0f, -1020.0f, -2700.0f, -3600.0f},
                        {670.0f, 425.0f, 25.0f, -170.0f, -1900.0f, -1400.0f},
                        {880.0f, 690.0f, 345.0f, -300.0f, -1300.0f, -595.0f},
                        {1140.0f, 1010.0f, 755.0f, 350.0f, -247.0f, -342.0f},
                        {1500.0f, 1330.0f, 1130.0f, 910.0f, 600.0f, -200.0f},
                        {1860.0f, 1700.0f, 1525.0f, 1360.0f, 1100.0f, 700.0f}};

/*
 * engine thrust model
 * Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed pages 716
 * taken from fortran code
 */
float thrust(float power, float alt, float rmach) {
  float h, dh, dm, cdh, rm, s, t, tmil, tmax, thrst, tidl;
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
}