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
void err_fun(double *nom_x, double *delta_x, double *out_3756630299165339778) {
   out_3756630299165339778[0] = delta_x[0] + nom_x[0];
   out_3756630299165339778[1] = delta_x[1] + nom_x[1];
   out_3756630299165339778[2] = delta_x[2] + nom_x[2];
   out_3756630299165339778[3] = delta_x[3] + nom_x[3];
   out_3756630299165339778[4] = delta_x[4] + nom_x[4];
   out_3756630299165339778[5] = delta_x[5] + nom_x[5];
   out_3756630299165339778[6] = delta_x[6] + nom_x[6];
   out_3756630299165339778[7] = delta_x[7] + nom_x[7];
   out_3756630299165339778[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1504140271042908252) {
   out_1504140271042908252[0] = -nom_x[0] + true_x[0];
   out_1504140271042908252[1] = -nom_x[1] + true_x[1];
   out_1504140271042908252[2] = -nom_x[2] + true_x[2];
   out_1504140271042908252[3] = -nom_x[3] + true_x[3];
   out_1504140271042908252[4] = -nom_x[4] + true_x[4];
   out_1504140271042908252[5] = -nom_x[5] + true_x[5];
   out_1504140271042908252[6] = -nom_x[6] + true_x[6];
   out_1504140271042908252[7] = -nom_x[7] + true_x[7];
   out_1504140271042908252[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_862615575614504655) {
   out_862615575614504655[0] = 1.0;
   out_862615575614504655[1] = 0;
   out_862615575614504655[2] = 0;
   out_862615575614504655[3] = 0;
   out_862615575614504655[4] = 0;
   out_862615575614504655[5] = 0;
   out_862615575614504655[6] = 0;
   out_862615575614504655[7] = 0;
   out_862615575614504655[8] = 0;
   out_862615575614504655[9] = 0;
   out_862615575614504655[10] = 1.0;
   out_862615575614504655[11] = 0;
   out_862615575614504655[12] = 0;
   out_862615575614504655[13] = 0;
   out_862615575614504655[14] = 0;
   out_862615575614504655[15] = 0;
   out_862615575614504655[16] = 0;
   out_862615575614504655[17] = 0;
   out_862615575614504655[18] = 0;
   out_862615575614504655[19] = 0;
   out_862615575614504655[20] = 1.0;
   out_862615575614504655[21] = 0;
   out_862615575614504655[22] = 0;
   out_862615575614504655[23] = 0;
   out_862615575614504655[24] = 0;
   out_862615575614504655[25] = 0;
   out_862615575614504655[26] = 0;
   out_862615575614504655[27] = 0;
   out_862615575614504655[28] = 0;
   out_862615575614504655[29] = 0;
   out_862615575614504655[30] = 1.0;
   out_862615575614504655[31] = 0;
   out_862615575614504655[32] = 0;
   out_862615575614504655[33] = 0;
   out_862615575614504655[34] = 0;
   out_862615575614504655[35] = 0;
   out_862615575614504655[36] = 0;
   out_862615575614504655[37] = 0;
   out_862615575614504655[38] = 0;
   out_862615575614504655[39] = 0;
   out_862615575614504655[40] = 1.0;
   out_862615575614504655[41] = 0;
   out_862615575614504655[42] = 0;
   out_862615575614504655[43] = 0;
   out_862615575614504655[44] = 0;
   out_862615575614504655[45] = 0;
   out_862615575614504655[46] = 0;
   out_862615575614504655[47] = 0;
   out_862615575614504655[48] = 0;
   out_862615575614504655[49] = 0;
   out_862615575614504655[50] = 1.0;
   out_862615575614504655[51] = 0;
   out_862615575614504655[52] = 0;
   out_862615575614504655[53] = 0;
   out_862615575614504655[54] = 0;
   out_862615575614504655[55] = 0;
   out_862615575614504655[56] = 0;
   out_862615575614504655[57] = 0;
   out_862615575614504655[58] = 0;
   out_862615575614504655[59] = 0;
   out_862615575614504655[60] = 1.0;
   out_862615575614504655[61] = 0;
   out_862615575614504655[62] = 0;
   out_862615575614504655[63] = 0;
   out_862615575614504655[64] = 0;
   out_862615575614504655[65] = 0;
   out_862615575614504655[66] = 0;
   out_862615575614504655[67] = 0;
   out_862615575614504655[68] = 0;
   out_862615575614504655[69] = 0;
   out_862615575614504655[70] = 1.0;
   out_862615575614504655[71] = 0;
   out_862615575614504655[72] = 0;
   out_862615575614504655[73] = 0;
   out_862615575614504655[74] = 0;
   out_862615575614504655[75] = 0;
   out_862615575614504655[76] = 0;
   out_862615575614504655[77] = 0;
   out_862615575614504655[78] = 0;
   out_862615575614504655[79] = 0;
   out_862615575614504655[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8976431745750036532) {
   out_8976431745750036532[0] = state[0];
   out_8976431745750036532[1] = state[1];
   out_8976431745750036532[2] = state[2];
   out_8976431745750036532[3] = state[3];
   out_8976431745750036532[4] = state[4];
   out_8976431745750036532[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8976431745750036532[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8976431745750036532[7] = state[7];
   out_8976431745750036532[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8449402309040632318) {
   out_8449402309040632318[0] = 1;
   out_8449402309040632318[1] = 0;
   out_8449402309040632318[2] = 0;
   out_8449402309040632318[3] = 0;
   out_8449402309040632318[4] = 0;
   out_8449402309040632318[5] = 0;
   out_8449402309040632318[6] = 0;
   out_8449402309040632318[7] = 0;
   out_8449402309040632318[8] = 0;
   out_8449402309040632318[9] = 0;
   out_8449402309040632318[10] = 1;
   out_8449402309040632318[11] = 0;
   out_8449402309040632318[12] = 0;
   out_8449402309040632318[13] = 0;
   out_8449402309040632318[14] = 0;
   out_8449402309040632318[15] = 0;
   out_8449402309040632318[16] = 0;
   out_8449402309040632318[17] = 0;
   out_8449402309040632318[18] = 0;
   out_8449402309040632318[19] = 0;
   out_8449402309040632318[20] = 1;
   out_8449402309040632318[21] = 0;
   out_8449402309040632318[22] = 0;
   out_8449402309040632318[23] = 0;
   out_8449402309040632318[24] = 0;
   out_8449402309040632318[25] = 0;
   out_8449402309040632318[26] = 0;
   out_8449402309040632318[27] = 0;
   out_8449402309040632318[28] = 0;
   out_8449402309040632318[29] = 0;
   out_8449402309040632318[30] = 1;
   out_8449402309040632318[31] = 0;
   out_8449402309040632318[32] = 0;
   out_8449402309040632318[33] = 0;
   out_8449402309040632318[34] = 0;
   out_8449402309040632318[35] = 0;
   out_8449402309040632318[36] = 0;
   out_8449402309040632318[37] = 0;
   out_8449402309040632318[38] = 0;
   out_8449402309040632318[39] = 0;
   out_8449402309040632318[40] = 1;
   out_8449402309040632318[41] = 0;
   out_8449402309040632318[42] = 0;
   out_8449402309040632318[43] = 0;
   out_8449402309040632318[44] = 0;
   out_8449402309040632318[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8449402309040632318[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8449402309040632318[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8449402309040632318[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8449402309040632318[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8449402309040632318[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8449402309040632318[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8449402309040632318[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8449402309040632318[53] = -9.8000000000000007*dt;
   out_8449402309040632318[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8449402309040632318[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8449402309040632318[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8449402309040632318[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8449402309040632318[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8449402309040632318[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8449402309040632318[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8449402309040632318[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8449402309040632318[62] = 0;
   out_8449402309040632318[63] = 0;
   out_8449402309040632318[64] = 0;
   out_8449402309040632318[65] = 0;
   out_8449402309040632318[66] = 0;
   out_8449402309040632318[67] = 0;
   out_8449402309040632318[68] = 0;
   out_8449402309040632318[69] = 0;
   out_8449402309040632318[70] = 1;
   out_8449402309040632318[71] = 0;
   out_8449402309040632318[72] = 0;
   out_8449402309040632318[73] = 0;
   out_8449402309040632318[74] = 0;
   out_8449402309040632318[75] = 0;
   out_8449402309040632318[76] = 0;
   out_8449402309040632318[77] = 0;
   out_8449402309040632318[78] = 0;
   out_8449402309040632318[79] = 0;
   out_8449402309040632318[80] = 1;
}
void h_25(double *state, double *unused, double *out_7280144249566122930) {
   out_7280144249566122930[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5359450340106912024) {
   out_5359450340106912024[0] = 0;
   out_5359450340106912024[1] = 0;
   out_5359450340106912024[2] = 0;
   out_5359450340106912024[3] = 0;
   out_5359450340106912024[4] = 0;
   out_5359450340106912024[5] = 0;
   out_5359450340106912024[6] = 1;
   out_5359450340106912024[7] = 0;
   out_5359450340106912024[8] = 0;
}
void h_24(double *state, double *unused, double *out_1595796958856727745) {
   out_1595796958856727745[0] = state[4];
   out_1595796958856727745[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3186800741101412458) {
   out_3186800741101412458[0] = 0;
   out_3186800741101412458[1] = 0;
   out_3186800741101412458[2] = 0;
   out_3186800741101412458[3] = 0;
   out_3186800741101412458[4] = 1;
   out_3186800741101412458[5] = 0;
   out_3186800741101412458[6] = 0;
   out_3186800741101412458[7] = 0;
   out_3186800741101412458[8] = 0;
   out_3186800741101412458[9] = 0;
   out_3186800741101412458[10] = 0;
   out_3186800741101412458[11] = 0;
   out_3186800741101412458[12] = 0;
   out_3186800741101412458[13] = 0;
   out_3186800741101412458[14] = 1;
   out_3186800741101412458[15] = 0;
   out_3186800741101412458[16] = 0;
   out_3186800741101412458[17] = 0;
}
void h_30(double *state, double *unused, double *out_7529150295889406336) {
   out_7529150295889406336[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6170603392111022837) {
   out_6170603392111022837[0] = 0;
   out_6170603392111022837[1] = 0;
   out_6170603392111022837[2] = 0;
   out_6170603392111022837[3] = 0;
   out_6170603392111022837[4] = 1;
   out_6170603392111022837[5] = 0;
   out_6170603392111022837[6] = 0;
   out_6170603392111022837[7] = 0;
   out_6170603392111022837[8] = 0;
}
void h_26(double *state, double *unused, double *out_6361406494813264599) {
   out_6361406494813264599[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8663976309867712625) {
   out_8663976309867712625[0] = 0;
   out_8663976309867712625[1] = 0;
   out_8663976309867712625[2] = 0;
   out_8663976309867712625[3] = 0;
   out_8663976309867712625[4] = 0;
   out_8663976309867712625[5] = 0;
   out_8663976309867712625[6] = 0;
   out_8663976309867712625[7] = 1;
   out_8663976309867712625[8] = 0;
}
void h_27(double *state, double *unused, double *out_3639637011742573612) {
   out_3639637011742573612[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8345366703911447748) {
   out_8345366703911447748[0] = 0;
   out_8345366703911447748[1] = 0;
   out_8345366703911447748[2] = 0;
   out_8345366703911447748[3] = 1;
   out_8345366703911447748[4] = 0;
   out_8345366703911447748[5] = 0;
   out_8345366703911447748[6] = 0;
   out_8345366703911447748[7] = 0;
   out_8345366703911447748[8] = 0;
}
void h_29(double *state, double *unused, double *out_2527477028228560691) {
   out_2527477028228560691[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5660372047796630653) {
   out_5660372047796630653[0] = 0;
   out_5660372047796630653[1] = 1;
   out_5660372047796630653[2] = 0;
   out_5660372047796630653[3] = 0;
   out_5660372047796630653[4] = 0;
   out_5660372047796630653[5] = 0;
   out_5660372047796630653[6] = 0;
   out_5660372047796630653[7] = 0;
   out_5660372047796630653[8] = 0;
}
void h_28(double *state, double *unused, double *out_8865544227577898559) {
   out_8865544227577898559[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3305615625859022261) {
   out_3305615625859022261[0] = 1;
   out_3305615625859022261[1] = 0;
   out_3305615625859022261[2] = 0;
   out_3305615625859022261[3] = 0;
   out_3305615625859022261[4] = 0;
   out_3305615625859022261[5] = 0;
   out_3305615625859022261[6] = 0;
   out_3305615625859022261[7] = 0;
   out_3305615625859022261[8] = 0;
}
void h_31(double *state, double *unused, double *out_601128401187926255) {
   out_601128401187926255[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6010618483090822339) {
   out_6010618483090822339[0] = 0;
   out_6010618483090822339[1] = 0;
   out_6010618483090822339[2] = 0;
   out_6010618483090822339[3] = 0;
   out_6010618483090822339[4] = 0;
   out_6010618483090822339[5] = 0;
   out_6010618483090822339[6] = 0;
   out_6010618483090822339[7] = 0;
   out_6010618483090822339[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3756630299165339778) {
  err_fun(nom_x, delta_x, out_3756630299165339778);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1504140271042908252) {
  inv_err_fun(nom_x, true_x, out_1504140271042908252);
}
void car_H_mod_fun(double *state, double *out_862615575614504655) {
  H_mod_fun(state, out_862615575614504655);
}
void car_f_fun(double *state, double dt, double *out_8976431745750036532) {
  f_fun(state,  dt, out_8976431745750036532);
}
void car_F_fun(double *state, double dt, double *out_8449402309040632318) {
  F_fun(state,  dt, out_8449402309040632318);
}
void car_h_25(double *state, double *unused, double *out_7280144249566122930) {
  h_25(state, unused, out_7280144249566122930);
}
void car_H_25(double *state, double *unused, double *out_5359450340106912024) {
  H_25(state, unused, out_5359450340106912024);
}
void car_h_24(double *state, double *unused, double *out_1595796958856727745) {
  h_24(state, unused, out_1595796958856727745);
}
void car_H_24(double *state, double *unused, double *out_3186800741101412458) {
  H_24(state, unused, out_3186800741101412458);
}
void car_h_30(double *state, double *unused, double *out_7529150295889406336) {
  h_30(state, unused, out_7529150295889406336);
}
void car_H_30(double *state, double *unused, double *out_6170603392111022837) {
  H_30(state, unused, out_6170603392111022837);
}
void car_h_26(double *state, double *unused, double *out_6361406494813264599) {
  h_26(state, unused, out_6361406494813264599);
}
void car_H_26(double *state, double *unused, double *out_8663976309867712625) {
  H_26(state, unused, out_8663976309867712625);
}
void car_h_27(double *state, double *unused, double *out_3639637011742573612) {
  h_27(state, unused, out_3639637011742573612);
}
void car_H_27(double *state, double *unused, double *out_8345366703911447748) {
  H_27(state, unused, out_8345366703911447748);
}
void car_h_29(double *state, double *unused, double *out_2527477028228560691) {
  h_29(state, unused, out_2527477028228560691);
}
void car_H_29(double *state, double *unused, double *out_5660372047796630653) {
  H_29(state, unused, out_5660372047796630653);
}
void car_h_28(double *state, double *unused, double *out_8865544227577898559) {
  h_28(state, unused, out_8865544227577898559);
}
void car_H_28(double *state, double *unused, double *out_3305615625859022261) {
  H_28(state, unused, out_3305615625859022261);
}
void car_h_31(double *state, double *unused, double *out_601128401187926255) {
  h_31(state, unused, out_601128401187926255);
}
void car_H_31(double *state, double *unused, double *out_6010618483090822339) {
  H_31(state, unused, out_6010618483090822339);
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
