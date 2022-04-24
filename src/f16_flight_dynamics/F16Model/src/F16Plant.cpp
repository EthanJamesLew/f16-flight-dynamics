//
// Created by elew on 4/23/22.
//

#include <f16_flight_dynamics/F16Model/F16Plant.h>
#include <f16_flight_dynamics/F16Model/LowLevelFunctions.h>
#include <math.h>

namespace F16Components {
using namespace LowLevelFunctions;

void F16Plant::subf16_model(const f16_state_type &x,
                            const f16_input_type &uinput,
                            f16_full_type &dxdt,
                            EngineModelType model_type,
                            bool adjust_cy) {

  double thtlc, el, ail, rdr;
  double vt, alpha, beta, phi, theta, psi, p, q, r, alt, power;
  double amach, qbar, cpow;
  double t, dail, drdr;
  double cxt, cyt, czt, clt, cmt, cnt;
  double tvt, b2v, cq;
  double t1, t2, t3, s1, s2, s3, s4, s5, s6, s7, s8;
  double cbta, u, v, w, sth, cth, sph, cph, spsi, cpsi, qs, qsb, rmqs, gcth, qsph, ay, az;
  double udot, vdot, wdot, dum;

  boost::array<double, 9> d;

  thtlc = uinput[0];
  el = uinput[1];
  ail = uinput[2];
  rdr = uinput[3];

  vt = x[0];
  alpha = x[1] * F16Val.rtod;
  beta = x[2] * F16Val.rtod;
  phi = x[3];
  theta = x[4];
  psi = x[5];
  p = x[6];
  q = x[7];
  r = x[8];
  alt = x[11];
  power = x[12];

  adc_return adc_ret = adc(vt, alt);
  amach = adc_ret.amach;
  qbar = adc_ret.qbar;

  cpow = tgear(thtlc);

  dxdt[12] = pdot(power, cpow);

  t = thrust(power, alt, amach);
  dail = ail / 20.0f;
  drdr = rdr / 30.0f;

  if (model_type == STEVENS) {
    /* stevens & lewis (look up table version) */
    cxt = cx(alpha, el);
    cyt = cy(beta, ail, rdr);
    czt = cz(alpha, beta, el);

    clt = cl(alpha, beta) + dlda(alpha, beta) * dail + dldr(alpha, beta) * drdr;
    cmt = cm(alpha, el);
    cnt = cn(alpha, beta) + dnda(alpha, beta) * dail + dndr(alpha, beta) * drdr;
  } else if (model_type == MORELLI) {

    boost::array<double, 6> mret= morelli(alpha*M_PI/180.0, beta*M_PI/180.0, el*M_PI/180.0, ail*M_PI/180.0, rdr*M_PI/180.0,
                                                  p, q, r, F16Val.cbar, F16Val.b, vt, F16Val.xcg, F16Val.xcgr);
    cxt = mret[0];
    cyt = mret[1];
    czt = mret[2];

    clt = mret[3];
    cmt = mret[4];
    cnt = mret[5];

  } else {
    cxt = 0.0f;
    cyt = 0.0f;
    czt = 0.0f;

    clt = 0.0f;
    cmt = 0.0f;
    cnt = 0.0f;
  }

  tvt = .5f / vt;
  b2v = F16Val.b * tvt;
  cq = F16Val.cbar * q * tvt;

  // get ready for state equations
  d = dampp(alpha);
  cxt = cxt + cq * d[0];
  cyt = cyt + b2v * (d[1] * r + d[2] * p);
  czt = czt + cq * d[3];
  clt = clt + b2v * (d[4] * r + d[5] * p);
  cmt = cmt + cq * d[6] + czt * (F16Val.xcgr - F16Val.xcg);
  cnt = cnt + b2v * (d[7] * r + d[8] * p) - cyt * (F16Val.xcgr - F16Val.xcg) * F16Val.cbar / F16Val.b;
  cbta = cos(x[2]);
  u = vt * cos(x[1]) * cbta;
  v = vt * sin(x[2]);
  w = vt * sin(x[1]) * cbta;
  sth = sin(theta);
  cth = cos(theta);
  sph = sin(phi);
  cph = cos(phi);
  spsi = sin(psi);
  cpsi = cos(psi);
  qs = qbar * F16Val.s;
  qsb = qs * F16Val.b;
  rmqs = F16Val.rm * qs;
  gcth = F16Val.g * cth;
  qsph = q * sph;
  ay = rmqs * cyt;
  az = rmqs * czt;

  //force equations
  udot = r * v - q * w - F16Val.g * sth + F16Val.rm * (qs * cxt + t);
  vdot = p * w - r * u + gcth * sph + ay;
  wdot = q * u - p * v + gcth * cph + az;
  dum = (u * u + w * w);

  dxdt[0] = (u * udot + v * vdot + w * wdot) / vt;
  dxdt[1] = (u * wdot - w * udot) / dum;
  dxdt[2] = (vt * vdot - v * dxdt[0]) * cbta / dum;

  // kinematics
  dxdt[3] = p + (sth / cth) * (qsph + r * cph);
  dxdt[4] = q * cph - r * sph;
  dxdt[5] = (qsph + r * cph) / cth;

  // moments
  dxdt[6] = (F16Val.c2 * p + F16Val.c1 * r + F16Val.c4 * F16Val.he) * q + qsb * (F16Val.c3 * clt + F16Val.c4 * cnt);

  dxdt[7] =
      (F16Val.c5 * p - F16Val.c7 * F16Val.he) * r + F16Val.c6 * (r * r - p * p) + qs * F16Val.cbar * F16Val.c7 * cmt;
  dxdt[8] = (F16Val.c8 * p - F16Val.c2 * r + F16Val.c9 * F16Val.he) * q + qsb * (F16Val.c4 * clt + F16Val.c9 * cnt);

  t1 = sph * cpsi;
  t2 = cph * sth;
  t3 = sph * spsi;
  s1 = cth * cpsi;
  s2 = cth * spsi;
  s3 = t1 * sth - cph * spsi;
  s4 = t3 * sth + cph * cpsi;
  s5 = sph * cth;
  s6 = t2 * cpsi + t3;
  s7 = t2 * spsi - t1;
  s8 = cph * cth;
  dxdt[9] = u * s1 + v * s3 + w * s6;
  dxdt[10] = u * s2 + v * s4 + w * s7;
  dxdt[11] = u * sth - v * s5 - w * s8;

  /* a little bit of output stuff */
  const double xa = 15.0;
  az = az-xa * dxdt[7];
  if (adjust_cy) {
    ay = ay+xa*dxdt[8];
  }

  float Nz, Ny;
  Nz = (-az / F16Val.g) - 1.0;
  Ny = ay / F16Val.g;

  dxdt[13] = Nz;
  dxdt[14] = Ny;
  dxdt[15] = az;
  dxdt[16] = ay;
  
}
}
