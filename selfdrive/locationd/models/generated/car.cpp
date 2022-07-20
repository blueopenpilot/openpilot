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
void err_fun(double *nom_x, double *delta_x, double *out_6300774429342187872) {
   out_6300774429342187872[0] = delta_x[0] + nom_x[0];
   out_6300774429342187872[1] = delta_x[1] + nom_x[1];
   out_6300774429342187872[2] = delta_x[2] + nom_x[2];
   out_6300774429342187872[3] = delta_x[3] + nom_x[3];
   out_6300774429342187872[4] = delta_x[4] + nom_x[4];
   out_6300774429342187872[5] = delta_x[5] + nom_x[5];
   out_6300774429342187872[6] = delta_x[6] + nom_x[6];
   out_6300774429342187872[7] = delta_x[7] + nom_x[7];
   out_6300774429342187872[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7490438056241178548) {
   out_7490438056241178548[0] = -nom_x[0] + true_x[0];
   out_7490438056241178548[1] = -nom_x[1] + true_x[1];
   out_7490438056241178548[2] = -nom_x[2] + true_x[2];
   out_7490438056241178548[3] = -nom_x[3] + true_x[3];
   out_7490438056241178548[4] = -nom_x[4] + true_x[4];
   out_7490438056241178548[5] = -nom_x[5] + true_x[5];
   out_7490438056241178548[6] = -nom_x[6] + true_x[6];
   out_7490438056241178548[7] = -nom_x[7] + true_x[7];
   out_7490438056241178548[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8524316325449410804) {
   out_8524316325449410804[0] = 1.0;
   out_8524316325449410804[1] = 0;
   out_8524316325449410804[2] = 0;
   out_8524316325449410804[3] = 0;
   out_8524316325449410804[4] = 0;
   out_8524316325449410804[5] = 0;
   out_8524316325449410804[6] = 0;
   out_8524316325449410804[7] = 0;
   out_8524316325449410804[8] = 0;
   out_8524316325449410804[9] = 0;
   out_8524316325449410804[10] = 1.0;
   out_8524316325449410804[11] = 0;
   out_8524316325449410804[12] = 0;
   out_8524316325449410804[13] = 0;
   out_8524316325449410804[14] = 0;
   out_8524316325449410804[15] = 0;
   out_8524316325449410804[16] = 0;
   out_8524316325449410804[17] = 0;
   out_8524316325449410804[18] = 0;
   out_8524316325449410804[19] = 0;
   out_8524316325449410804[20] = 1.0;
   out_8524316325449410804[21] = 0;
   out_8524316325449410804[22] = 0;
   out_8524316325449410804[23] = 0;
   out_8524316325449410804[24] = 0;
   out_8524316325449410804[25] = 0;
   out_8524316325449410804[26] = 0;
   out_8524316325449410804[27] = 0;
   out_8524316325449410804[28] = 0;
   out_8524316325449410804[29] = 0;
   out_8524316325449410804[30] = 1.0;
   out_8524316325449410804[31] = 0;
   out_8524316325449410804[32] = 0;
   out_8524316325449410804[33] = 0;
   out_8524316325449410804[34] = 0;
   out_8524316325449410804[35] = 0;
   out_8524316325449410804[36] = 0;
   out_8524316325449410804[37] = 0;
   out_8524316325449410804[38] = 0;
   out_8524316325449410804[39] = 0;
   out_8524316325449410804[40] = 1.0;
   out_8524316325449410804[41] = 0;
   out_8524316325449410804[42] = 0;
   out_8524316325449410804[43] = 0;
   out_8524316325449410804[44] = 0;
   out_8524316325449410804[45] = 0;
   out_8524316325449410804[46] = 0;
   out_8524316325449410804[47] = 0;
   out_8524316325449410804[48] = 0;
   out_8524316325449410804[49] = 0;
   out_8524316325449410804[50] = 1.0;
   out_8524316325449410804[51] = 0;
   out_8524316325449410804[52] = 0;
   out_8524316325449410804[53] = 0;
   out_8524316325449410804[54] = 0;
   out_8524316325449410804[55] = 0;
   out_8524316325449410804[56] = 0;
   out_8524316325449410804[57] = 0;
   out_8524316325449410804[58] = 0;
   out_8524316325449410804[59] = 0;
   out_8524316325449410804[60] = 1.0;
   out_8524316325449410804[61] = 0;
   out_8524316325449410804[62] = 0;
   out_8524316325449410804[63] = 0;
   out_8524316325449410804[64] = 0;
   out_8524316325449410804[65] = 0;
   out_8524316325449410804[66] = 0;
   out_8524316325449410804[67] = 0;
   out_8524316325449410804[68] = 0;
   out_8524316325449410804[69] = 0;
   out_8524316325449410804[70] = 1.0;
   out_8524316325449410804[71] = 0;
   out_8524316325449410804[72] = 0;
   out_8524316325449410804[73] = 0;
   out_8524316325449410804[74] = 0;
   out_8524316325449410804[75] = 0;
   out_8524316325449410804[76] = 0;
   out_8524316325449410804[77] = 0;
   out_8524316325449410804[78] = 0;
   out_8524316325449410804[79] = 0;
   out_8524316325449410804[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3805144880965371850) {
   out_3805144880965371850[0] = state[0];
   out_3805144880965371850[1] = state[1];
   out_3805144880965371850[2] = state[2];
   out_3805144880965371850[3] = state[3];
   out_3805144880965371850[4] = state[4];
   out_3805144880965371850[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3805144880965371850[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3805144880965371850[7] = state[7];
   out_3805144880965371850[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5441793289359834623) {
   out_5441793289359834623[0] = 1;
   out_5441793289359834623[1] = 0;
   out_5441793289359834623[2] = 0;
   out_5441793289359834623[3] = 0;
   out_5441793289359834623[4] = 0;
   out_5441793289359834623[5] = 0;
   out_5441793289359834623[6] = 0;
   out_5441793289359834623[7] = 0;
   out_5441793289359834623[8] = 0;
   out_5441793289359834623[9] = 0;
   out_5441793289359834623[10] = 1;
   out_5441793289359834623[11] = 0;
   out_5441793289359834623[12] = 0;
   out_5441793289359834623[13] = 0;
   out_5441793289359834623[14] = 0;
   out_5441793289359834623[15] = 0;
   out_5441793289359834623[16] = 0;
   out_5441793289359834623[17] = 0;
   out_5441793289359834623[18] = 0;
   out_5441793289359834623[19] = 0;
   out_5441793289359834623[20] = 1;
   out_5441793289359834623[21] = 0;
   out_5441793289359834623[22] = 0;
   out_5441793289359834623[23] = 0;
   out_5441793289359834623[24] = 0;
   out_5441793289359834623[25] = 0;
   out_5441793289359834623[26] = 0;
   out_5441793289359834623[27] = 0;
   out_5441793289359834623[28] = 0;
   out_5441793289359834623[29] = 0;
   out_5441793289359834623[30] = 1;
   out_5441793289359834623[31] = 0;
   out_5441793289359834623[32] = 0;
   out_5441793289359834623[33] = 0;
   out_5441793289359834623[34] = 0;
   out_5441793289359834623[35] = 0;
   out_5441793289359834623[36] = 0;
   out_5441793289359834623[37] = 0;
   out_5441793289359834623[38] = 0;
   out_5441793289359834623[39] = 0;
   out_5441793289359834623[40] = 1;
   out_5441793289359834623[41] = 0;
   out_5441793289359834623[42] = 0;
   out_5441793289359834623[43] = 0;
   out_5441793289359834623[44] = 0;
   out_5441793289359834623[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5441793289359834623[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5441793289359834623[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5441793289359834623[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5441793289359834623[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5441793289359834623[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5441793289359834623[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5441793289359834623[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5441793289359834623[53] = -9.8000000000000007*dt;
   out_5441793289359834623[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5441793289359834623[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5441793289359834623[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5441793289359834623[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5441793289359834623[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5441793289359834623[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5441793289359834623[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5441793289359834623[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5441793289359834623[62] = 0;
   out_5441793289359834623[63] = 0;
   out_5441793289359834623[64] = 0;
   out_5441793289359834623[65] = 0;
   out_5441793289359834623[66] = 0;
   out_5441793289359834623[67] = 0;
   out_5441793289359834623[68] = 0;
   out_5441793289359834623[69] = 0;
   out_5441793289359834623[70] = 1;
   out_5441793289359834623[71] = 0;
   out_5441793289359834623[72] = 0;
   out_5441793289359834623[73] = 0;
   out_5441793289359834623[74] = 0;
   out_5441793289359834623[75] = 0;
   out_5441793289359834623[76] = 0;
   out_5441793289359834623[77] = 0;
   out_5441793289359834623[78] = 0;
   out_5441793289359834623[79] = 0;
   out_5441793289359834623[80] = 1;
}
void h_25(double *state, double *unused, double *out_4139015218604585631) {
   out_4139015218604585631[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3381867638513875071) {
   out_3381867638513875071[0] = 0;
   out_3381867638513875071[1] = 0;
   out_3381867638513875071[2] = 0;
   out_3381867638513875071[3] = 0;
   out_3381867638513875071[4] = 0;
   out_3381867638513875071[5] = 0;
   out_3381867638513875071[6] = 1;
   out_3381867638513875071[7] = 0;
   out_3381867638513875071[8] = 0;
}
void h_24(double *state, double *unused, double *out_2169929365393919604) {
   out_2169929365393919604[0] = state[4];
   out_2169929365393919604[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1156159854535006509) {
   out_1156159854535006509[0] = 0;
   out_1156159854535006509[1] = 0;
   out_1156159854535006509[2] = 0;
   out_1156159854535006509[3] = 0;
   out_1156159854535006509[4] = 1;
   out_1156159854535006509[5] = 0;
   out_1156159854535006509[6] = 0;
   out_1156159854535006509[7] = 0;
   out_1156159854535006509[8] = 0;
   out_1156159854535006509[9] = 0;
   out_1156159854535006509[10] = 0;
   out_1156159854535006509[11] = 0;
   out_1156159854535006509[12] = 0;
   out_1156159854535006509[13] = 0;
   out_1156159854535006509[14] = 1;
   out_1156159854535006509[15] = 0;
   out_1156159854535006509[16] = 0;
   out_1156159854535006509[17] = 0;
}
void h_30(double *state, double *unused, double *out_3753924276279937728) {
   out_3753924276279937728[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3534822702977741684) {
   out_3534822702977741684[0] = 0;
   out_3534822702977741684[1] = 0;
   out_3534822702977741684[2] = 0;
   out_3534822702977741684[3] = 0;
   out_3534822702977741684[4] = 1;
   out_3534822702977741684[5] = 0;
   out_3534822702977741684[6] = 0;
   out_3534822702977741684[7] = 0;
   out_3534822702977741684[8] = 0;
}
void h_26(double *state, double *unused, double *out_8755597543940783373) {
   out_8755597543940783373[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7123370957387931295) {
   out_7123370957387931295[0] = 0;
   out_7123370957387931295[1] = 0;
   out_7123370957387931295[2] = 0;
   out_7123370957387931295[3] = 0;
   out_7123370957387931295[4] = 0;
   out_7123370957387931295[5] = 0;
   out_7123370957387931295[6] = 0;
   out_7123370957387931295[7] = 1;
   out_7123370957387931295[8] = 0;
}
void h_27(double *state, double *unused, double *out_1102229167160252946) {
   out_1102229167160252946[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1360059391177316773) {
   out_1360059391177316773[0] = 0;
   out_1360059391177316773[1] = 0;
   out_1360059391177316773[2] = 0;
   out_1360059391177316773[3] = 1;
   out_1360059391177316773[4] = 0;
   out_1360059391177316773[5] = 0;
   out_1360059391177316773[6] = 0;
   out_1360059391177316773[7] = 0;
   out_1360059391177316773[8] = 0;
}
void h_29(double *state, double *unused, double *out_7690057093636548109) {
   out_7690057093636548109[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4045054047292133868) {
   out_4045054047292133868[0] = 0;
   out_4045054047292133868[1] = 1;
   out_4045054047292133868[2] = 0;
   out_4045054047292133868[3] = 0;
   out_4045054047292133868[4] = 0;
   out_4045054047292133868[5] = 0;
   out_4045054047292133868[6] = 0;
   out_4045054047292133868[7] = 0;
   out_4045054047292133868[8] = 0;
}
void h_28(double *state, double *unused, double *out_158719314258590511) {
   out_158719314258590511[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5435702352761764834) {
   out_5435702352761764834[0] = 1;
   out_5435702352761764834[1] = 0;
   out_5435702352761764834[2] = 0;
   out_5435702352761764834[3] = 0;
   out_5435702352761764834[4] = 0;
   out_5435702352761764834[5] = 0;
   out_5435702352761764834[6] = 0;
   out_5435702352761764834[7] = 0;
   out_5435702352761764834[8] = 0;
}
void h_31(double *state, double *unused, double *out_3863821156320079742) {
   out_3863821156320079742[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3351221676636914643) {
   out_3351221676636914643[0] = 0;
   out_3351221676636914643[1] = 0;
   out_3351221676636914643[2] = 0;
   out_3351221676636914643[3] = 0;
   out_3351221676636914643[4] = 0;
   out_3351221676636914643[5] = 0;
   out_3351221676636914643[6] = 0;
   out_3351221676636914643[7] = 0;
   out_3351221676636914643[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6300774429342187872) {
  err_fun(nom_x, delta_x, out_6300774429342187872);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7490438056241178548) {
  inv_err_fun(nom_x, true_x, out_7490438056241178548);
}
void car_H_mod_fun(double *state, double *out_8524316325449410804) {
  H_mod_fun(state, out_8524316325449410804);
}
void car_f_fun(double *state, double dt, double *out_3805144880965371850) {
  f_fun(state,  dt, out_3805144880965371850);
}
void car_F_fun(double *state, double dt, double *out_5441793289359834623) {
  F_fun(state,  dt, out_5441793289359834623);
}
void car_h_25(double *state, double *unused, double *out_4139015218604585631) {
  h_25(state, unused, out_4139015218604585631);
}
void car_H_25(double *state, double *unused, double *out_3381867638513875071) {
  H_25(state, unused, out_3381867638513875071);
}
void car_h_24(double *state, double *unused, double *out_2169929365393919604) {
  h_24(state, unused, out_2169929365393919604);
}
void car_H_24(double *state, double *unused, double *out_1156159854535006509) {
  H_24(state, unused, out_1156159854535006509);
}
void car_h_30(double *state, double *unused, double *out_3753924276279937728) {
  h_30(state, unused, out_3753924276279937728);
}
void car_H_30(double *state, double *unused, double *out_3534822702977741684) {
  H_30(state, unused, out_3534822702977741684);
}
void car_h_26(double *state, double *unused, double *out_8755597543940783373) {
  h_26(state, unused, out_8755597543940783373);
}
void car_H_26(double *state, double *unused, double *out_7123370957387931295) {
  H_26(state, unused, out_7123370957387931295);
}
void car_h_27(double *state, double *unused, double *out_1102229167160252946) {
  h_27(state, unused, out_1102229167160252946);
}
void car_H_27(double *state, double *unused, double *out_1360059391177316773) {
  H_27(state, unused, out_1360059391177316773);
}
void car_h_29(double *state, double *unused, double *out_7690057093636548109) {
  h_29(state, unused, out_7690057093636548109);
}
void car_H_29(double *state, double *unused, double *out_4045054047292133868) {
  H_29(state, unused, out_4045054047292133868);
}
void car_h_28(double *state, double *unused, double *out_158719314258590511) {
  h_28(state, unused, out_158719314258590511);
}
void car_H_28(double *state, double *unused, double *out_5435702352761764834) {
  H_28(state, unused, out_5435702352761764834);
}
void car_h_31(double *state, double *unused, double *out_3863821156320079742) {
  h_31(state, unused, out_3863821156320079742);
}
void car_H_31(double *state, double *unused, double *out_3351221676636914643) {
  H_31(state, unused, out_3351221676636914643);
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
