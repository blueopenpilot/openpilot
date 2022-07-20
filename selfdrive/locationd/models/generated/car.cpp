#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6934071491810961021) {
   out_6934071491810961021[0] = delta_x[0] + nom_x[0];
   out_6934071491810961021[1] = delta_x[1] + nom_x[1];
   out_6934071491810961021[2] = delta_x[2] + nom_x[2];
   out_6934071491810961021[3] = delta_x[3] + nom_x[3];
   out_6934071491810961021[4] = delta_x[4] + nom_x[4];
   out_6934071491810961021[5] = delta_x[5] + nom_x[5];
   out_6934071491810961021[6] = delta_x[6] + nom_x[6];
   out_6934071491810961021[7] = delta_x[7] + nom_x[7];
   out_6934071491810961021[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5348077128676617066) {
   out_5348077128676617066[0] = -nom_x[0] + true_x[0];
   out_5348077128676617066[1] = -nom_x[1] + true_x[1];
   out_5348077128676617066[2] = -nom_x[2] + true_x[2];
   out_5348077128676617066[3] = -nom_x[3] + true_x[3];
   out_5348077128676617066[4] = -nom_x[4] + true_x[4];
   out_5348077128676617066[5] = -nom_x[5] + true_x[5];
   out_5348077128676617066[6] = -nom_x[6] + true_x[6];
   out_5348077128676617066[7] = -nom_x[7] + true_x[7];
   out_5348077128676617066[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4159593663802874514) {
   out_4159593663802874514[0] = 1.0;
   out_4159593663802874514[1] = 0;
   out_4159593663802874514[2] = 0;
   out_4159593663802874514[3] = 0;
   out_4159593663802874514[4] = 0;
   out_4159593663802874514[5] = 0;
   out_4159593663802874514[6] = 0;
   out_4159593663802874514[7] = 0;
   out_4159593663802874514[8] = 0;
   out_4159593663802874514[9] = 0;
   out_4159593663802874514[10] = 1.0;
   out_4159593663802874514[11] = 0;
   out_4159593663802874514[12] = 0;
   out_4159593663802874514[13] = 0;
   out_4159593663802874514[14] = 0;
   out_4159593663802874514[15] = 0;
   out_4159593663802874514[16] = 0;
   out_4159593663802874514[17] = 0;
   out_4159593663802874514[18] = 0;
   out_4159593663802874514[19] = 0;
   out_4159593663802874514[20] = 1.0;
   out_4159593663802874514[21] = 0;
   out_4159593663802874514[22] = 0;
   out_4159593663802874514[23] = 0;
   out_4159593663802874514[24] = 0;
   out_4159593663802874514[25] = 0;
   out_4159593663802874514[26] = 0;
   out_4159593663802874514[27] = 0;
   out_4159593663802874514[28] = 0;
   out_4159593663802874514[29] = 0;
   out_4159593663802874514[30] = 1.0;
   out_4159593663802874514[31] = 0;
   out_4159593663802874514[32] = 0;
   out_4159593663802874514[33] = 0;
   out_4159593663802874514[34] = 0;
   out_4159593663802874514[35] = 0;
   out_4159593663802874514[36] = 0;
   out_4159593663802874514[37] = 0;
   out_4159593663802874514[38] = 0;
   out_4159593663802874514[39] = 0;
   out_4159593663802874514[40] = 1.0;
   out_4159593663802874514[41] = 0;
   out_4159593663802874514[42] = 0;
   out_4159593663802874514[43] = 0;
   out_4159593663802874514[44] = 0;
   out_4159593663802874514[45] = 0;
   out_4159593663802874514[46] = 0;
   out_4159593663802874514[47] = 0;
   out_4159593663802874514[48] = 0;
   out_4159593663802874514[49] = 0;
   out_4159593663802874514[50] = 1.0;
   out_4159593663802874514[51] = 0;
   out_4159593663802874514[52] = 0;
   out_4159593663802874514[53] = 0;
   out_4159593663802874514[54] = 0;
   out_4159593663802874514[55] = 0;
   out_4159593663802874514[56] = 0;
   out_4159593663802874514[57] = 0;
   out_4159593663802874514[58] = 0;
   out_4159593663802874514[59] = 0;
   out_4159593663802874514[60] = 1.0;
   out_4159593663802874514[61] = 0;
   out_4159593663802874514[62] = 0;
   out_4159593663802874514[63] = 0;
   out_4159593663802874514[64] = 0;
   out_4159593663802874514[65] = 0;
   out_4159593663802874514[66] = 0;
   out_4159593663802874514[67] = 0;
   out_4159593663802874514[68] = 0;
   out_4159593663802874514[69] = 0;
   out_4159593663802874514[70] = 1.0;
   out_4159593663802874514[71] = 0;
   out_4159593663802874514[72] = 0;
   out_4159593663802874514[73] = 0;
   out_4159593663802874514[74] = 0;
   out_4159593663802874514[75] = 0;
   out_4159593663802874514[76] = 0;
   out_4159593663802874514[77] = 0;
   out_4159593663802874514[78] = 0;
   out_4159593663802874514[79] = 0;
   out_4159593663802874514[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4252897372159381429) {
   out_4252897372159381429[0] = state[0];
   out_4252897372159381429[1] = state[1];
   out_4252897372159381429[2] = state[2];
   out_4252897372159381429[3] = state[3];
   out_4252897372159381429[4] = state[4];
   out_4252897372159381429[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4252897372159381429[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4252897372159381429[7] = state[7];
   out_4252897372159381429[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6678692079888871950) {
   out_6678692079888871950[0] = 1;
   out_6678692079888871950[1] = 0;
   out_6678692079888871950[2] = 0;
   out_6678692079888871950[3] = 0;
   out_6678692079888871950[4] = 0;
   out_6678692079888871950[5] = 0;
   out_6678692079888871950[6] = 0;
   out_6678692079888871950[7] = 0;
   out_6678692079888871950[8] = 0;
   out_6678692079888871950[9] = 0;
   out_6678692079888871950[10] = 1;
   out_6678692079888871950[11] = 0;
   out_6678692079888871950[12] = 0;
   out_6678692079888871950[13] = 0;
   out_6678692079888871950[14] = 0;
   out_6678692079888871950[15] = 0;
   out_6678692079888871950[16] = 0;
   out_6678692079888871950[17] = 0;
   out_6678692079888871950[18] = 0;
   out_6678692079888871950[19] = 0;
   out_6678692079888871950[20] = 1;
   out_6678692079888871950[21] = 0;
   out_6678692079888871950[22] = 0;
   out_6678692079888871950[23] = 0;
   out_6678692079888871950[24] = 0;
   out_6678692079888871950[25] = 0;
   out_6678692079888871950[26] = 0;
   out_6678692079888871950[27] = 0;
   out_6678692079888871950[28] = 0;
   out_6678692079888871950[29] = 0;
   out_6678692079888871950[30] = 1;
   out_6678692079888871950[31] = 0;
   out_6678692079888871950[32] = 0;
   out_6678692079888871950[33] = 0;
   out_6678692079888871950[34] = 0;
   out_6678692079888871950[35] = 0;
   out_6678692079888871950[36] = 0;
   out_6678692079888871950[37] = 0;
   out_6678692079888871950[38] = 0;
   out_6678692079888871950[39] = 0;
   out_6678692079888871950[40] = 1;
   out_6678692079888871950[41] = 0;
   out_6678692079888871950[42] = 0;
   out_6678692079888871950[43] = 0;
   out_6678692079888871950[44] = 0;
   out_6678692079888871950[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6678692079888871950[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6678692079888871950[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6678692079888871950[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6678692079888871950[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6678692079888871950[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6678692079888871950[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6678692079888871950[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6678692079888871950[53] = -9.8000000000000007*dt;
   out_6678692079888871950[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6678692079888871950[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6678692079888871950[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6678692079888871950[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6678692079888871950[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6678692079888871950[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6678692079888871950[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6678692079888871950[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6678692079888871950[62] = 0;
   out_6678692079888871950[63] = 0;
   out_6678692079888871950[64] = 0;
   out_6678692079888871950[65] = 0;
   out_6678692079888871950[66] = 0;
   out_6678692079888871950[67] = 0;
   out_6678692079888871950[68] = 0;
   out_6678692079888871950[69] = 0;
   out_6678692079888871950[70] = 1;
   out_6678692079888871950[71] = 0;
   out_6678692079888871950[72] = 0;
   out_6678692079888871950[73] = 0;
   out_6678692079888871950[74] = 0;
   out_6678692079888871950[75] = 0;
   out_6678692079888871950[76] = 0;
   out_6678692079888871950[77] = 0;
   out_6678692079888871950[78] = 0;
   out_6678692079888871950[79] = 0;
   out_6678692079888871950[80] = 1;
}
void h_25(double *state, double *unused, double *out_422800814981887828) {
   out_422800814981887828[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6712579314593980538) {
   out_6712579314593980538[0] = 0;
   out_6712579314593980538[1] = 0;
   out_6712579314593980538[2] = 0;
   out_6712579314593980538[3] = 0;
   out_6712579314593980538[4] = 0;
   out_6712579314593980538[5] = 0;
   out_6712579314593980538[6] = 1;
   out_6712579314593980538[7] = 0;
   out_6712579314593980538[8] = 0;
}
void h_24(double *state, double *unused, double *out_7125751847150321414) {
   out_7125751847150321414[0] = state[4];
   out_7125751847150321414[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4486871530615111976) {
   out_4486871530615111976[0] = 0;
   out_4486871530615111976[1] = 0;
   out_4486871530615111976[2] = 0;
   out_4486871530615111976[3] = 0;
   out_4486871530615111976[4] = 1;
   out_4486871530615111976[5] = 0;
   out_4486871530615111976[6] = 0;
   out_4486871530615111976[7] = 0;
   out_4486871530615111976[8] = 0;
   out_4486871530615111976[9] = 0;
   out_4486871530615111976[10] = 0;
   out_4486871530615111976[11] = 0;
   out_4486871530615111976[12] = 0;
   out_4486871530615111976[13] = 0;
   out_4486871530615111976[14] = 1;
   out_4486871530615111976[15] = 0;
   out_4486871530615111976[16] = 0;
   out_4486871530615111976[17] = 0;
}
void h_30(double *state, double *unused, double *out_4487868591059444478) {
   out_4487868591059444478[0] = state[4];
}
void H_30(double *state, double *unused, double *out_204111026897636217) {
   out_204111026897636217[0] = 0;
   out_204111026897636217[1] = 0;
   out_204111026897636217[2] = 0;
   out_204111026897636217[3] = 0;
   out_204111026897636217[4] = 1;
   out_204111026897636217[5] = 0;
   out_204111026897636217[6] = 0;
   out_204111026897636217[7] = 0;
   out_204111026897636217[8] = 0;
}
void h_26(double *state, double *unused, double *out_7108536964396069770) {
   out_7108536964396069770[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6055725250483668634) {
   out_6055725250483668634[0] = 0;
   out_6055725250483668634[1] = 0;
   out_6055725250483668634[2] = 0;
   out_6055725250483668634[3] = 0;
   out_6055725250483668634[4] = 0;
   out_6055725250483668634[5] = 0;
   out_6055725250483668634[6] = 0;
   out_6055725250483668634[7] = 1;
   out_6055725250483668634[8] = 0;
}
void h_27(double *state, double *unused, double *out_180515069694589689) {
   out_180515069694589689[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1970652284902788694) {
   out_1970652284902788694[0] = 0;
   out_1970652284902788694[1] = 0;
   out_1970652284902788694[2] = 0;
   out_1970652284902788694[3] = 1;
   out_1970652284902788694[4] = 0;
   out_1970652284902788694[5] = 0;
   out_1970652284902788694[6] = 0;
   out_1970652284902788694[7] = 0;
   out_1970652284902788694[8] = 0;
}
void h_29(double *state, double *unused, double *out_3128241060050074650) {
   out_3128241060050074650[0] = state[1];
}
void H_29(double *state, double *unused, double *out_714342371212028401) {
   out_714342371212028401[0] = 0;
   out_714342371212028401[1] = 1;
   out_714342371212028401[2] = 0;
   out_714342371212028401[3] = 0;
   out_714342371212028401[4] = 0;
   out_714342371212028401[5] = 0;
   out_714342371212028401[6] = 0;
   out_714342371212028401[7] = 0;
   out_714342371212028401[8] = 0;
}
void h_28(double *state, double *unused, double *out_2856270981749742996) {
   out_2856270981749742996[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4368056645857502173) {
   out_4368056645857502173[0] = 1;
   out_4368056645857502173[1] = 0;
   out_4368056645857502173[2] = 0;
   out_4368056645857502173[3] = 0;
   out_4368056645857502173[4] = 0;
   out_4368056645857502173[5] = 0;
   out_4368056645857502173[6] = 0;
   out_4368056645857502173[7] = 0;
   out_4368056645857502173[8] = 0;
}
void h_31(double *state, double *unused, double *out_5737408863670873035) {
   out_5737408863670873035[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6681933352717020110) {
   out_6681933352717020110[0] = 0;
   out_6681933352717020110[1] = 0;
   out_6681933352717020110[2] = 0;
   out_6681933352717020110[3] = 0;
   out_6681933352717020110[4] = 0;
   out_6681933352717020110[5] = 0;
   out_6681933352717020110[6] = 0;
   out_6681933352717020110[7] = 0;
   out_6681933352717020110[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6934071491810961021) {
  err_fun(nom_x, delta_x, out_6934071491810961021);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5348077128676617066) {
  inv_err_fun(nom_x, true_x, out_5348077128676617066);
}
void car_H_mod_fun(double *state, double *out_4159593663802874514) {
  H_mod_fun(state, out_4159593663802874514);
}
void car_f_fun(double *state, double dt, double *out_4252897372159381429) {
  f_fun(state,  dt, out_4252897372159381429);
}
void car_F_fun(double *state, double dt, double *out_6678692079888871950) {
  F_fun(state,  dt, out_6678692079888871950);
}
void car_h_25(double *state, double *unused, double *out_422800814981887828) {
  h_25(state, unused, out_422800814981887828);
}
void car_H_25(double *state, double *unused, double *out_6712579314593980538) {
  H_25(state, unused, out_6712579314593980538);
}
void car_h_24(double *state, double *unused, double *out_7125751847150321414) {
  h_24(state, unused, out_7125751847150321414);
}
void car_H_24(double *state, double *unused, double *out_4486871530615111976) {
  H_24(state, unused, out_4486871530615111976);
}
void car_h_30(double *state, double *unused, double *out_4487868591059444478) {
  h_30(state, unused, out_4487868591059444478);
}
void car_H_30(double *state, double *unused, double *out_204111026897636217) {
  H_30(state, unused, out_204111026897636217);
}
void car_h_26(double *state, double *unused, double *out_7108536964396069770) {
  h_26(state, unused, out_7108536964396069770);
}
void car_H_26(double *state, double *unused, double *out_6055725250483668634) {
  H_26(state, unused, out_6055725250483668634);
}
void car_h_27(double *state, double *unused, double *out_180515069694589689) {
  h_27(state, unused, out_180515069694589689);
}
void car_H_27(double *state, double *unused, double *out_1970652284902788694) {
  H_27(state, unused, out_1970652284902788694);
}
void car_h_29(double *state, double *unused, double *out_3128241060050074650) {
  h_29(state, unused, out_3128241060050074650);
}
void car_H_29(double *state, double *unused, double *out_714342371212028401) {
  H_29(state, unused, out_714342371212028401);
}
void car_h_28(double *state, double *unused, double *out_2856270981749742996) {
  h_28(state, unused, out_2856270981749742996);
}
void car_H_28(double *state, double *unused, double *out_4368056645857502173) {
  H_28(state, unused, out_4368056645857502173);
}
void car_h_31(double *state, double *unused, double *out_5737408863670873035) {
  h_31(state, unused, out_5737408863670873035);
}
void car_H_31(double *state, double *unused, double *out_6681933352717020110) {
  H_31(state, unused, out_6681933352717020110);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
