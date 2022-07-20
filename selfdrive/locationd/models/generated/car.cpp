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
void err_fun(double *nom_x, double *delta_x, double *out_9182764351597017505) {
   out_9182764351597017505[0] = delta_x[0] + nom_x[0];
   out_9182764351597017505[1] = delta_x[1] + nom_x[1];
   out_9182764351597017505[2] = delta_x[2] + nom_x[2];
   out_9182764351597017505[3] = delta_x[3] + nom_x[3];
   out_9182764351597017505[4] = delta_x[4] + nom_x[4];
   out_9182764351597017505[5] = delta_x[5] + nom_x[5];
   out_9182764351597017505[6] = delta_x[6] + nom_x[6];
   out_9182764351597017505[7] = delta_x[7] + nom_x[7];
   out_9182764351597017505[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6627670019370406735) {
   out_6627670019370406735[0] = -nom_x[0] + true_x[0];
   out_6627670019370406735[1] = -nom_x[1] + true_x[1];
   out_6627670019370406735[2] = -nom_x[2] + true_x[2];
   out_6627670019370406735[3] = -nom_x[3] + true_x[3];
   out_6627670019370406735[4] = -nom_x[4] + true_x[4];
   out_6627670019370406735[5] = -nom_x[5] + true_x[5];
   out_6627670019370406735[6] = -nom_x[6] + true_x[6];
   out_6627670019370406735[7] = -nom_x[7] + true_x[7];
   out_6627670019370406735[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1096019154694829530) {
   out_1096019154694829530[0] = 1.0;
   out_1096019154694829530[1] = 0;
   out_1096019154694829530[2] = 0;
   out_1096019154694829530[3] = 0;
   out_1096019154694829530[4] = 0;
   out_1096019154694829530[5] = 0;
   out_1096019154694829530[6] = 0;
   out_1096019154694829530[7] = 0;
   out_1096019154694829530[8] = 0;
   out_1096019154694829530[9] = 0;
   out_1096019154694829530[10] = 1.0;
   out_1096019154694829530[11] = 0;
   out_1096019154694829530[12] = 0;
   out_1096019154694829530[13] = 0;
   out_1096019154694829530[14] = 0;
   out_1096019154694829530[15] = 0;
   out_1096019154694829530[16] = 0;
   out_1096019154694829530[17] = 0;
   out_1096019154694829530[18] = 0;
   out_1096019154694829530[19] = 0;
   out_1096019154694829530[20] = 1.0;
   out_1096019154694829530[21] = 0;
   out_1096019154694829530[22] = 0;
   out_1096019154694829530[23] = 0;
   out_1096019154694829530[24] = 0;
   out_1096019154694829530[25] = 0;
   out_1096019154694829530[26] = 0;
   out_1096019154694829530[27] = 0;
   out_1096019154694829530[28] = 0;
   out_1096019154694829530[29] = 0;
   out_1096019154694829530[30] = 1.0;
   out_1096019154694829530[31] = 0;
   out_1096019154694829530[32] = 0;
   out_1096019154694829530[33] = 0;
   out_1096019154694829530[34] = 0;
   out_1096019154694829530[35] = 0;
   out_1096019154694829530[36] = 0;
   out_1096019154694829530[37] = 0;
   out_1096019154694829530[38] = 0;
   out_1096019154694829530[39] = 0;
   out_1096019154694829530[40] = 1.0;
   out_1096019154694829530[41] = 0;
   out_1096019154694829530[42] = 0;
   out_1096019154694829530[43] = 0;
   out_1096019154694829530[44] = 0;
   out_1096019154694829530[45] = 0;
   out_1096019154694829530[46] = 0;
   out_1096019154694829530[47] = 0;
   out_1096019154694829530[48] = 0;
   out_1096019154694829530[49] = 0;
   out_1096019154694829530[50] = 1.0;
   out_1096019154694829530[51] = 0;
   out_1096019154694829530[52] = 0;
   out_1096019154694829530[53] = 0;
   out_1096019154694829530[54] = 0;
   out_1096019154694829530[55] = 0;
   out_1096019154694829530[56] = 0;
   out_1096019154694829530[57] = 0;
   out_1096019154694829530[58] = 0;
   out_1096019154694829530[59] = 0;
   out_1096019154694829530[60] = 1.0;
   out_1096019154694829530[61] = 0;
   out_1096019154694829530[62] = 0;
   out_1096019154694829530[63] = 0;
   out_1096019154694829530[64] = 0;
   out_1096019154694829530[65] = 0;
   out_1096019154694829530[66] = 0;
   out_1096019154694829530[67] = 0;
   out_1096019154694829530[68] = 0;
   out_1096019154694829530[69] = 0;
   out_1096019154694829530[70] = 1.0;
   out_1096019154694829530[71] = 0;
   out_1096019154694829530[72] = 0;
   out_1096019154694829530[73] = 0;
   out_1096019154694829530[74] = 0;
   out_1096019154694829530[75] = 0;
   out_1096019154694829530[76] = 0;
   out_1096019154694829530[77] = 0;
   out_1096019154694829530[78] = 0;
   out_1096019154694829530[79] = 0;
   out_1096019154694829530[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4339405799423572566) {
   out_4339405799423572566[0] = state[0];
   out_4339405799423572566[1] = state[1];
   out_4339405799423572566[2] = state[2];
   out_4339405799423572566[3] = state[3];
   out_4339405799423572566[4] = state[4];
   out_4339405799423572566[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4339405799423572566[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4339405799423572566[7] = state[7];
   out_4339405799423572566[8] = state[8];
}
void F_fun(double *state, double dt, double *out_346308560399216339) {
   out_346308560399216339[0] = 1;
   out_346308560399216339[1] = 0;
   out_346308560399216339[2] = 0;
   out_346308560399216339[3] = 0;
   out_346308560399216339[4] = 0;
   out_346308560399216339[5] = 0;
   out_346308560399216339[6] = 0;
   out_346308560399216339[7] = 0;
   out_346308560399216339[8] = 0;
   out_346308560399216339[9] = 0;
   out_346308560399216339[10] = 1;
   out_346308560399216339[11] = 0;
   out_346308560399216339[12] = 0;
   out_346308560399216339[13] = 0;
   out_346308560399216339[14] = 0;
   out_346308560399216339[15] = 0;
   out_346308560399216339[16] = 0;
   out_346308560399216339[17] = 0;
   out_346308560399216339[18] = 0;
   out_346308560399216339[19] = 0;
   out_346308560399216339[20] = 1;
   out_346308560399216339[21] = 0;
   out_346308560399216339[22] = 0;
   out_346308560399216339[23] = 0;
   out_346308560399216339[24] = 0;
   out_346308560399216339[25] = 0;
   out_346308560399216339[26] = 0;
   out_346308560399216339[27] = 0;
   out_346308560399216339[28] = 0;
   out_346308560399216339[29] = 0;
   out_346308560399216339[30] = 1;
   out_346308560399216339[31] = 0;
   out_346308560399216339[32] = 0;
   out_346308560399216339[33] = 0;
   out_346308560399216339[34] = 0;
   out_346308560399216339[35] = 0;
   out_346308560399216339[36] = 0;
   out_346308560399216339[37] = 0;
   out_346308560399216339[38] = 0;
   out_346308560399216339[39] = 0;
   out_346308560399216339[40] = 1;
   out_346308560399216339[41] = 0;
   out_346308560399216339[42] = 0;
   out_346308560399216339[43] = 0;
   out_346308560399216339[44] = 0;
   out_346308560399216339[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_346308560399216339[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_346308560399216339[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_346308560399216339[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_346308560399216339[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_346308560399216339[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_346308560399216339[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_346308560399216339[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_346308560399216339[53] = -9.8000000000000007*dt;
   out_346308560399216339[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_346308560399216339[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_346308560399216339[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_346308560399216339[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_346308560399216339[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_346308560399216339[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_346308560399216339[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_346308560399216339[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_346308560399216339[62] = 0;
   out_346308560399216339[63] = 0;
   out_346308560399216339[64] = 0;
   out_346308560399216339[65] = 0;
   out_346308560399216339[66] = 0;
   out_346308560399216339[67] = 0;
   out_346308560399216339[68] = 0;
   out_346308560399216339[69] = 0;
   out_346308560399216339[70] = 1;
   out_346308560399216339[71] = 0;
   out_346308560399216339[72] = 0;
   out_346308560399216339[73] = 0;
   out_346308560399216339[74] = 0;
   out_346308560399216339[75] = 0;
   out_346308560399216339[76] = 0;
   out_346308560399216339[77] = 0;
   out_346308560399216339[78] = 0;
   out_346308560399216339[79] = 0;
   out_346308560399216339[80] = 1;
}
void h_25(double *state, double *unused, double *out_2234606782992478127) {
   out_2234606782992478127[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6386336540791844398) {
   out_6386336540791844398[0] = 0;
   out_6386336540791844398[1] = 0;
   out_6386336540791844398[2] = 0;
   out_6386336540791844398[3] = 0;
   out_6386336540791844398[4] = 0;
   out_6386336540791844398[5] = 0;
   out_6386336540791844398[6] = 1;
   out_6386336540791844398[7] = 0;
   out_6386336540791844398[8] = 0;
}
void h_24(double *state, double *unused, double *out_3192731147905955648) {
   out_3192731147905955648[0] = state[4];
   out_3192731147905955648[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7313898205951198184) {
   out_7313898205951198184[0] = 0;
   out_7313898205951198184[1] = 0;
   out_7313898205951198184[2] = 0;
   out_7313898205951198184[3] = 0;
   out_7313898205951198184[4] = 1;
   out_7313898205951198184[5] = 0;
   out_7313898205951198184[6] = 0;
   out_7313898205951198184[7] = 0;
   out_7313898205951198184[8] = 0;
   out_7313898205951198184[9] = 0;
   out_7313898205951198184[10] = 0;
   out_7313898205951198184[11] = 0;
   out_7313898205951198184[12] = 0;
   out_7313898205951198184[13] = 0;
   out_7313898205951198184[14] = 1;
   out_7313898205951198184[15] = 0;
   out_7313898205951198184[16] = 0;
   out_7313898205951198184[17] = 0;
}
void h_30(double *state, double *unused, double *out_1863696515286893737) {
   out_1863696515286893737[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3868003582284595771) {
   out_3868003582284595771[0] = 0;
   out_3868003582284595771[1] = 0;
   out_3868003582284595771[2] = 0;
   out_3868003582284595771[3] = 0;
   out_3868003582284595771[4] = 1;
   out_3868003582284595771[5] = 0;
   out_3868003582284595771[6] = 0;
   out_3868003582284595771[7] = 0;
   out_3868003582284595771[8] = 0;
}
void h_26(double *state, double *unused, double *out_7118272656719947028) {
   out_7118272656719947028[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8318904214043650994) {
   out_8318904214043650994[0] = 0;
   out_8318904214043650994[1] = 0;
   out_8318904214043650994[2] = 0;
   out_8318904214043650994[3] = 0;
   out_8318904214043650994[4] = 0;
   out_8318904214043650994[5] = 0;
   out_8318904214043650994[6] = 0;
   out_8318904214043650994[7] = 1;
   out_8318904214043650994[8] = 0;
}
void h_27(double *state, double *unused, double *out_8537932094985248772) {
   out_8537932094985248772[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1644409511100652554) {
   out_1644409511100652554[0] = 0;
   out_1644409511100652554[1] = 0;
   out_1644409511100652554[2] = 0;
   out_1644409511100652554[3] = 1;
   out_1644409511100652554[4] = 0;
   out_1644409511100652554[5] = 0;
   out_1644409511100652554[6] = 0;
   out_1644409511100652554[7] = 0;
   out_1644409511100652554[8] = 0;
}
void h_29(double *state, double *unused, double *out_8262738032700742883) {
   out_8262738032700742883[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3357772237970203587) {
   out_3357772237970203587[0] = 0;
   out_3357772237970203587[1] = 1;
   out_3357772237970203587[2] = 0;
   out_3357772237970203587[3] = 0;
   out_3357772237970203587[4] = 0;
   out_3357772237970203587[5] = 0;
   out_3357772237970203587[6] = 0;
   out_3357772237970203587[7] = 0;
   out_3357772237970203587[8] = 0;
}
void h_28(double *state, double *unused, double *out_2906049274931237065) {
   out_2906049274931237065[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8440171255039734161) {
   out_8440171255039734161[0] = 1;
   out_8440171255039734161[1] = 0;
   out_8440171255039734161[2] = 0;
   out_8440171255039734161[3] = 0;
   out_8440171255039734161[4] = 0;
   out_8440171255039734161[5] = 0;
   out_8440171255039734161[6] = 0;
   out_8440171255039734161[7] = 0;
   out_8440171255039734161[8] = 0;
}
void h_31(double *state, double *unused, double *out_1959412720707972238) {
   out_1959412720707972238[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7692696111810299518) {
   out_7692696111810299518[0] = 0;
   out_7692696111810299518[1] = 0;
   out_7692696111810299518[2] = 0;
   out_7692696111810299518[3] = 0;
   out_7692696111810299518[4] = 0;
   out_7692696111810299518[5] = 0;
   out_7692696111810299518[6] = 0;
   out_7692696111810299518[7] = 0;
   out_7692696111810299518[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_9182764351597017505) {
  err_fun(nom_x, delta_x, out_9182764351597017505);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6627670019370406735) {
  inv_err_fun(nom_x, true_x, out_6627670019370406735);
}
void car_H_mod_fun(double *state, double *out_1096019154694829530) {
  H_mod_fun(state, out_1096019154694829530);
}
void car_f_fun(double *state, double dt, double *out_4339405799423572566) {
  f_fun(state,  dt, out_4339405799423572566);
}
void car_F_fun(double *state, double dt, double *out_346308560399216339) {
  F_fun(state,  dt, out_346308560399216339);
}
void car_h_25(double *state, double *unused, double *out_2234606782992478127) {
  h_25(state, unused, out_2234606782992478127);
}
void car_H_25(double *state, double *unused, double *out_6386336540791844398) {
  H_25(state, unused, out_6386336540791844398);
}
void car_h_24(double *state, double *unused, double *out_3192731147905955648) {
  h_24(state, unused, out_3192731147905955648);
}
void car_H_24(double *state, double *unused, double *out_7313898205951198184) {
  H_24(state, unused, out_7313898205951198184);
}
void car_h_30(double *state, double *unused, double *out_1863696515286893737) {
  h_30(state, unused, out_1863696515286893737);
}
void car_H_30(double *state, double *unused, double *out_3868003582284595771) {
  H_30(state, unused, out_3868003582284595771);
}
void car_h_26(double *state, double *unused, double *out_7118272656719947028) {
  h_26(state, unused, out_7118272656719947028);
}
void car_H_26(double *state, double *unused, double *out_8318904214043650994) {
  H_26(state, unused, out_8318904214043650994);
}
void car_h_27(double *state, double *unused, double *out_8537932094985248772) {
  h_27(state, unused, out_8537932094985248772);
}
void car_H_27(double *state, double *unused, double *out_1644409511100652554) {
  H_27(state, unused, out_1644409511100652554);
}
void car_h_29(double *state, double *unused, double *out_8262738032700742883) {
  h_29(state, unused, out_8262738032700742883);
}
void car_H_29(double *state, double *unused, double *out_3357772237970203587) {
  H_29(state, unused, out_3357772237970203587);
}
void car_h_28(double *state, double *unused, double *out_2906049274931237065) {
  h_28(state, unused, out_2906049274931237065);
}
void car_H_28(double *state, double *unused, double *out_8440171255039734161) {
  H_28(state, unused, out_8440171255039734161);
}
void car_h_31(double *state, double *unused, double *out_1959412720707972238) {
  h_31(state, unused, out_1959412720707972238);
}
void car_H_31(double *state, double *unused, double *out_7692696111810299518) {
  H_31(state, unused, out_7692696111810299518);
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
