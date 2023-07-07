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
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4702752847547446396) {
   out_4702752847547446396[0] = delta_x[0] + nom_x[0];
   out_4702752847547446396[1] = delta_x[1] + nom_x[1];
   out_4702752847547446396[2] = delta_x[2] + nom_x[2];
   out_4702752847547446396[3] = delta_x[3] + nom_x[3];
   out_4702752847547446396[4] = delta_x[4] + nom_x[4];
   out_4702752847547446396[5] = delta_x[5] + nom_x[5];
   out_4702752847547446396[6] = delta_x[6] + nom_x[6];
   out_4702752847547446396[7] = delta_x[7] + nom_x[7];
   out_4702752847547446396[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7477561154065216360) {
   out_7477561154065216360[0] = -nom_x[0] + true_x[0];
   out_7477561154065216360[1] = -nom_x[1] + true_x[1];
   out_7477561154065216360[2] = -nom_x[2] + true_x[2];
   out_7477561154065216360[3] = -nom_x[3] + true_x[3];
   out_7477561154065216360[4] = -nom_x[4] + true_x[4];
   out_7477561154065216360[5] = -nom_x[5] + true_x[5];
   out_7477561154065216360[6] = -nom_x[6] + true_x[6];
   out_7477561154065216360[7] = -nom_x[7] + true_x[7];
   out_7477561154065216360[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2627163213642550557) {
   out_2627163213642550557[0] = 1.0;
   out_2627163213642550557[1] = 0;
   out_2627163213642550557[2] = 0;
   out_2627163213642550557[3] = 0;
   out_2627163213642550557[4] = 0;
   out_2627163213642550557[5] = 0;
   out_2627163213642550557[6] = 0;
   out_2627163213642550557[7] = 0;
   out_2627163213642550557[8] = 0;
   out_2627163213642550557[9] = 0;
   out_2627163213642550557[10] = 1.0;
   out_2627163213642550557[11] = 0;
   out_2627163213642550557[12] = 0;
   out_2627163213642550557[13] = 0;
   out_2627163213642550557[14] = 0;
   out_2627163213642550557[15] = 0;
   out_2627163213642550557[16] = 0;
   out_2627163213642550557[17] = 0;
   out_2627163213642550557[18] = 0;
   out_2627163213642550557[19] = 0;
   out_2627163213642550557[20] = 1.0;
   out_2627163213642550557[21] = 0;
   out_2627163213642550557[22] = 0;
   out_2627163213642550557[23] = 0;
   out_2627163213642550557[24] = 0;
   out_2627163213642550557[25] = 0;
   out_2627163213642550557[26] = 0;
   out_2627163213642550557[27] = 0;
   out_2627163213642550557[28] = 0;
   out_2627163213642550557[29] = 0;
   out_2627163213642550557[30] = 1.0;
   out_2627163213642550557[31] = 0;
   out_2627163213642550557[32] = 0;
   out_2627163213642550557[33] = 0;
   out_2627163213642550557[34] = 0;
   out_2627163213642550557[35] = 0;
   out_2627163213642550557[36] = 0;
   out_2627163213642550557[37] = 0;
   out_2627163213642550557[38] = 0;
   out_2627163213642550557[39] = 0;
   out_2627163213642550557[40] = 1.0;
   out_2627163213642550557[41] = 0;
   out_2627163213642550557[42] = 0;
   out_2627163213642550557[43] = 0;
   out_2627163213642550557[44] = 0;
   out_2627163213642550557[45] = 0;
   out_2627163213642550557[46] = 0;
   out_2627163213642550557[47] = 0;
   out_2627163213642550557[48] = 0;
   out_2627163213642550557[49] = 0;
   out_2627163213642550557[50] = 1.0;
   out_2627163213642550557[51] = 0;
   out_2627163213642550557[52] = 0;
   out_2627163213642550557[53] = 0;
   out_2627163213642550557[54] = 0;
   out_2627163213642550557[55] = 0;
   out_2627163213642550557[56] = 0;
   out_2627163213642550557[57] = 0;
   out_2627163213642550557[58] = 0;
   out_2627163213642550557[59] = 0;
   out_2627163213642550557[60] = 1.0;
   out_2627163213642550557[61] = 0;
   out_2627163213642550557[62] = 0;
   out_2627163213642550557[63] = 0;
   out_2627163213642550557[64] = 0;
   out_2627163213642550557[65] = 0;
   out_2627163213642550557[66] = 0;
   out_2627163213642550557[67] = 0;
   out_2627163213642550557[68] = 0;
   out_2627163213642550557[69] = 0;
   out_2627163213642550557[70] = 1.0;
   out_2627163213642550557[71] = 0;
   out_2627163213642550557[72] = 0;
   out_2627163213642550557[73] = 0;
   out_2627163213642550557[74] = 0;
   out_2627163213642550557[75] = 0;
   out_2627163213642550557[76] = 0;
   out_2627163213642550557[77] = 0;
   out_2627163213642550557[78] = 0;
   out_2627163213642550557[79] = 0;
   out_2627163213642550557[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2949320639431583799) {
   out_2949320639431583799[0] = state[0];
   out_2949320639431583799[1] = state[1];
   out_2949320639431583799[2] = state[2];
   out_2949320639431583799[3] = state[3];
   out_2949320639431583799[4] = state[4];
   out_2949320639431583799[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2949320639431583799[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2949320639431583799[7] = state[7];
   out_2949320639431583799[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8341996880936103292) {
   out_8341996880936103292[0] = 1;
   out_8341996880936103292[1] = 0;
   out_8341996880936103292[2] = 0;
   out_8341996880936103292[3] = 0;
   out_8341996880936103292[4] = 0;
   out_8341996880936103292[5] = 0;
   out_8341996880936103292[6] = 0;
   out_8341996880936103292[7] = 0;
   out_8341996880936103292[8] = 0;
   out_8341996880936103292[9] = 0;
   out_8341996880936103292[10] = 1;
   out_8341996880936103292[11] = 0;
   out_8341996880936103292[12] = 0;
   out_8341996880936103292[13] = 0;
   out_8341996880936103292[14] = 0;
   out_8341996880936103292[15] = 0;
   out_8341996880936103292[16] = 0;
   out_8341996880936103292[17] = 0;
   out_8341996880936103292[18] = 0;
   out_8341996880936103292[19] = 0;
   out_8341996880936103292[20] = 1;
   out_8341996880936103292[21] = 0;
   out_8341996880936103292[22] = 0;
   out_8341996880936103292[23] = 0;
   out_8341996880936103292[24] = 0;
   out_8341996880936103292[25] = 0;
   out_8341996880936103292[26] = 0;
   out_8341996880936103292[27] = 0;
   out_8341996880936103292[28] = 0;
   out_8341996880936103292[29] = 0;
   out_8341996880936103292[30] = 1;
   out_8341996880936103292[31] = 0;
   out_8341996880936103292[32] = 0;
   out_8341996880936103292[33] = 0;
   out_8341996880936103292[34] = 0;
   out_8341996880936103292[35] = 0;
   out_8341996880936103292[36] = 0;
   out_8341996880936103292[37] = 0;
   out_8341996880936103292[38] = 0;
   out_8341996880936103292[39] = 0;
   out_8341996880936103292[40] = 1;
   out_8341996880936103292[41] = 0;
   out_8341996880936103292[42] = 0;
   out_8341996880936103292[43] = 0;
   out_8341996880936103292[44] = 0;
   out_8341996880936103292[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8341996880936103292[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8341996880936103292[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8341996880936103292[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8341996880936103292[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8341996880936103292[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8341996880936103292[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8341996880936103292[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8341996880936103292[53] = -9.8000000000000007*dt;
   out_8341996880936103292[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8341996880936103292[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8341996880936103292[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8341996880936103292[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8341996880936103292[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8341996880936103292[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8341996880936103292[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8341996880936103292[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8341996880936103292[62] = 0;
   out_8341996880936103292[63] = 0;
   out_8341996880936103292[64] = 0;
   out_8341996880936103292[65] = 0;
   out_8341996880936103292[66] = 0;
   out_8341996880936103292[67] = 0;
   out_8341996880936103292[68] = 0;
   out_8341996880936103292[69] = 0;
   out_8341996880936103292[70] = 1;
   out_8341996880936103292[71] = 0;
   out_8341996880936103292[72] = 0;
   out_8341996880936103292[73] = 0;
   out_8341996880936103292[74] = 0;
   out_8341996880936103292[75] = 0;
   out_8341996880936103292[76] = 0;
   out_8341996880936103292[77] = 0;
   out_8341996880936103292[78] = 0;
   out_8341996880936103292[79] = 0;
   out_8341996880936103292[80] = 1;
}
void h_25(double *state, double *unused, double *out_7940820690712606148) {
   out_7940820690712606148[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5496306684859195057) {
   out_5496306684859195057[0] = 0;
   out_5496306684859195057[1] = 0;
   out_5496306684859195057[2] = 0;
   out_5496306684859195057[3] = 0;
   out_5496306684859195057[4] = 0;
   out_5496306684859195057[5] = 0;
   out_5496306684859195057[6] = 1;
   out_5496306684859195057[7] = 0;
   out_5496306684859195057[8] = 0;
}
void h_24(double *state, double *unused, double *out_8680464193662503182) {
   out_8680464193662503182[0] = state[4];
   out_8680464193662503182[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4664942968776231447) {
   out_4664942968776231447[0] = 0;
   out_4664942968776231447[1] = 0;
   out_4664942968776231447[2] = 0;
   out_4664942968776231447[3] = 0;
   out_4664942968776231447[4] = 1;
   out_4664942968776231447[5] = 0;
   out_4664942968776231447[6] = 0;
   out_4664942968776231447[7] = 0;
   out_4664942968776231447[8] = 0;
   out_4664942968776231447[9] = 0;
   out_4664942968776231447[10] = 0;
   out_4664942968776231447[11] = 0;
   out_4664942968776231447[12] = 0;
   out_4664942968776231447[13] = 0;
   out_4664942968776231447[14] = 1;
   out_4664942968776231447[15] = 0;
   out_4664942968776231447[16] = 0;
   out_4664942968776231447[17] = 0;
}
void h_30(double *state, double *unused, double *out_8414235386947951111) {
   out_8414235386947951111[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2977973726351946430) {
   out_2977973726351946430[0] = 0;
   out_2977973726351946430[1] = 0;
   out_2977973726351946430[2] = 0;
   out_2977973726351946430[3] = 0;
   out_2977973726351946430[4] = 1;
   out_2977973726351946430[5] = 0;
   out_2977973726351946430[6] = 0;
   out_2977973726351946430[7] = 0;
   out_2977973726351946430[8] = 0;
}
void h_26(double *state, double *unused, double *out_1956308648912788502) {
   out_1956308648912788502[0] = state[7];
}
void H_26(double *state, double *unused, double *out_9208934069976300335) {
   out_9208934069976300335[0] = 0;
   out_9208934069976300335[1] = 0;
   out_9208934069976300335[2] = 0;
   out_9208934069976300335[3] = 0;
   out_9208934069976300335[4] = 0;
   out_9208934069976300335[5] = 0;
   out_9208934069976300335[6] = 0;
   out_9208934069976300335[7] = 1;
   out_9208934069976300335[8] = 0;
}
void h_27(double *state, double *unused, double *out_4106881865583096322) {
   out_4106881865583096322[0] = state[3];
}
void H_27(double *state, double *unused, double *out_754379655168003213) {
   out_754379655168003213[0] = 0;
   out_754379655168003213[1] = 0;
   out_754379655168003213[2] = 0;
   out_754379655168003213[3] = 1;
   out_754379655168003213[4] = 0;
   out_754379655168003213[5] = 0;
   out_754379655168003213[6] = 0;
   out_754379655168003213[7] = 0;
   out_754379655168003213[8] = 0;
}
void h_29(double *state, double *unused, double *out_8578567303724458) {
   out_8578567303724458[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2467742382037554246) {
   out_2467742382037554246[0] = 0;
   out_2467742382037554246[1] = 1;
   out_2467742382037554246[2] = 0;
   out_2467742382037554246[3] = 0;
   out_2467742382037554246[4] = 0;
   out_2467742382037554246[5] = 0;
   out_2467742382037554246[6] = 0;
   out_2467742382037554246[7] = 0;
   out_2467742382037554246[8] = 0;
}
void h_28(double *state, double *unused, double *out_5088965958918394434) {
   out_5088965958918394434[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7550141399107084820) {
   out_7550141399107084820[0] = 1;
   out_7550141399107084820[1] = 0;
   out_7550141399107084820[2] = 0;
   out_7550141399107084820[3] = 0;
   out_7550141399107084820[4] = 0;
   out_7550141399107084820[5] = 0;
   out_7550141399107084820[6] = 0;
   out_7550141399107084820[7] = 0;
   out_7550141399107084820[8] = 0;
}
void h_31(double *state, double *unused, double *out_7783634022361477003) {
   out_7783634022361477003[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5465660722982234629) {
   out_5465660722982234629[0] = 0;
   out_5465660722982234629[1] = 0;
   out_5465660722982234629[2] = 0;
   out_5465660722982234629[3] = 0;
   out_5465660722982234629[4] = 0;
   out_5465660722982234629[5] = 0;
   out_5465660722982234629[6] = 0;
   out_5465660722982234629[7] = 0;
   out_5465660722982234629[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4702752847547446396) {
  err_fun(nom_x, delta_x, out_4702752847547446396);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7477561154065216360) {
  inv_err_fun(nom_x, true_x, out_7477561154065216360);
}
void car_H_mod_fun(double *state, double *out_2627163213642550557) {
  H_mod_fun(state, out_2627163213642550557);
}
void car_f_fun(double *state, double dt, double *out_2949320639431583799) {
  f_fun(state,  dt, out_2949320639431583799);
}
void car_F_fun(double *state, double dt, double *out_8341996880936103292) {
  F_fun(state,  dt, out_8341996880936103292);
}
void car_h_25(double *state, double *unused, double *out_7940820690712606148) {
  h_25(state, unused, out_7940820690712606148);
}
void car_H_25(double *state, double *unused, double *out_5496306684859195057) {
  H_25(state, unused, out_5496306684859195057);
}
void car_h_24(double *state, double *unused, double *out_8680464193662503182) {
  h_24(state, unused, out_8680464193662503182);
}
void car_H_24(double *state, double *unused, double *out_4664942968776231447) {
  H_24(state, unused, out_4664942968776231447);
}
void car_h_30(double *state, double *unused, double *out_8414235386947951111) {
  h_30(state, unused, out_8414235386947951111);
}
void car_H_30(double *state, double *unused, double *out_2977973726351946430) {
  H_30(state, unused, out_2977973726351946430);
}
void car_h_26(double *state, double *unused, double *out_1956308648912788502) {
  h_26(state, unused, out_1956308648912788502);
}
void car_H_26(double *state, double *unused, double *out_9208934069976300335) {
  H_26(state, unused, out_9208934069976300335);
}
void car_h_27(double *state, double *unused, double *out_4106881865583096322) {
  h_27(state, unused, out_4106881865583096322);
}
void car_H_27(double *state, double *unused, double *out_754379655168003213) {
  H_27(state, unused, out_754379655168003213);
}
void car_h_29(double *state, double *unused, double *out_8578567303724458) {
  h_29(state, unused, out_8578567303724458);
}
void car_H_29(double *state, double *unused, double *out_2467742382037554246) {
  H_29(state, unused, out_2467742382037554246);
}
void car_h_28(double *state, double *unused, double *out_5088965958918394434) {
  h_28(state, unused, out_5088965958918394434);
}
void car_H_28(double *state, double *unused, double *out_7550141399107084820) {
  H_28(state, unused, out_7550141399107084820);
}
void car_h_31(double *state, double *unused, double *out_7783634022361477003) {
  h_31(state, unused, out_7783634022361477003);
}
void car_H_31(double *state, double *unused, double *out_5465660722982234629) {
  H_31(state, unused, out_5465660722982234629);
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
