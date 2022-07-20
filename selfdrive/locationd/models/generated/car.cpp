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
void err_fun(double *nom_x, double *delta_x, double *out_5379945950210905137) {
   out_5379945950210905137[0] = delta_x[0] + nom_x[0];
   out_5379945950210905137[1] = delta_x[1] + nom_x[1];
   out_5379945950210905137[2] = delta_x[2] + nom_x[2];
   out_5379945950210905137[3] = delta_x[3] + nom_x[3];
   out_5379945950210905137[4] = delta_x[4] + nom_x[4];
   out_5379945950210905137[5] = delta_x[5] + nom_x[5];
   out_5379945950210905137[6] = delta_x[6] + nom_x[6];
   out_5379945950210905137[7] = delta_x[7] + nom_x[7];
   out_5379945950210905137[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6100139837532875915) {
   out_6100139837532875915[0] = -nom_x[0] + true_x[0];
   out_6100139837532875915[1] = -nom_x[1] + true_x[1];
   out_6100139837532875915[2] = -nom_x[2] + true_x[2];
   out_6100139837532875915[3] = -nom_x[3] + true_x[3];
   out_6100139837532875915[4] = -nom_x[4] + true_x[4];
   out_6100139837532875915[5] = -nom_x[5] + true_x[5];
   out_6100139837532875915[6] = -nom_x[6] + true_x[6];
   out_6100139837532875915[7] = -nom_x[7] + true_x[7];
   out_6100139837532875915[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8072557983707861366) {
   out_8072557983707861366[0] = 1.0;
   out_8072557983707861366[1] = 0;
   out_8072557983707861366[2] = 0;
   out_8072557983707861366[3] = 0;
   out_8072557983707861366[4] = 0;
   out_8072557983707861366[5] = 0;
   out_8072557983707861366[6] = 0;
   out_8072557983707861366[7] = 0;
   out_8072557983707861366[8] = 0;
   out_8072557983707861366[9] = 0;
   out_8072557983707861366[10] = 1.0;
   out_8072557983707861366[11] = 0;
   out_8072557983707861366[12] = 0;
   out_8072557983707861366[13] = 0;
   out_8072557983707861366[14] = 0;
   out_8072557983707861366[15] = 0;
   out_8072557983707861366[16] = 0;
   out_8072557983707861366[17] = 0;
   out_8072557983707861366[18] = 0;
   out_8072557983707861366[19] = 0;
   out_8072557983707861366[20] = 1.0;
   out_8072557983707861366[21] = 0;
   out_8072557983707861366[22] = 0;
   out_8072557983707861366[23] = 0;
   out_8072557983707861366[24] = 0;
   out_8072557983707861366[25] = 0;
   out_8072557983707861366[26] = 0;
   out_8072557983707861366[27] = 0;
   out_8072557983707861366[28] = 0;
   out_8072557983707861366[29] = 0;
   out_8072557983707861366[30] = 1.0;
   out_8072557983707861366[31] = 0;
   out_8072557983707861366[32] = 0;
   out_8072557983707861366[33] = 0;
   out_8072557983707861366[34] = 0;
   out_8072557983707861366[35] = 0;
   out_8072557983707861366[36] = 0;
   out_8072557983707861366[37] = 0;
   out_8072557983707861366[38] = 0;
   out_8072557983707861366[39] = 0;
   out_8072557983707861366[40] = 1.0;
   out_8072557983707861366[41] = 0;
   out_8072557983707861366[42] = 0;
   out_8072557983707861366[43] = 0;
   out_8072557983707861366[44] = 0;
   out_8072557983707861366[45] = 0;
   out_8072557983707861366[46] = 0;
   out_8072557983707861366[47] = 0;
   out_8072557983707861366[48] = 0;
   out_8072557983707861366[49] = 0;
   out_8072557983707861366[50] = 1.0;
   out_8072557983707861366[51] = 0;
   out_8072557983707861366[52] = 0;
   out_8072557983707861366[53] = 0;
   out_8072557983707861366[54] = 0;
   out_8072557983707861366[55] = 0;
   out_8072557983707861366[56] = 0;
   out_8072557983707861366[57] = 0;
   out_8072557983707861366[58] = 0;
   out_8072557983707861366[59] = 0;
   out_8072557983707861366[60] = 1.0;
   out_8072557983707861366[61] = 0;
   out_8072557983707861366[62] = 0;
   out_8072557983707861366[63] = 0;
   out_8072557983707861366[64] = 0;
   out_8072557983707861366[65] = 0;
   out_8072557983707861366[66] = 0;
   out_8072557983707861366[67] = 0;
   out_8072557983707861366[68] = 0;
   out_8072557983707861366[69] = 0;
   out_8072557983707861366[70] = 1.0;
   out_8072557983707861366[71] = 0;
   out_8072557983707861366[72] = 0;
   out_8072557983707861366[73] = 0;
   out_8072557983707861366[74] = 0;
   out_8072557983707861366[75] = 0;
   out_8072557983707861366[76] = 0;
   out_8072557983707861366[77] = 0;
   out_8072557983707861366[78] = 0;
   out_8072557983707861366[79] = 0;
   out_8072557983707861366[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3587256842187338616) {
   out_3587256842187338616[0] = state[0];
   out_3587256842187338616[1] = state[1];
   out_3587256842187338616[2] = state[2];
   out_3587256842187338616[3] = state[3];
   out_3587256842187338616[4] = state[4];
   out_3587256842187338616[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3587256842187338616[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3587256842187338616[7] = state[7];
   out_3587256842187338616[8] = state[8];
}
void F_fun(double *state, double dt, double *out_673445270312808434) {
   out_673445270312808434[0] = 1;
   out_673445270312808434[1] = 0;
   out_673445270312808434[2] = 0;
   out_673445270312808434[3] = 0;
   out_673445270312808434[4] = 0;
   out_673445270312808434[5] = 0;
   out_673445270312808434[6] = 0;
   out_673445270312808434[7] = 0;
   out_673445270312808434[8] = 0;
   out_673445270312808434[9] = 0;
   out_673445270312808434[10] = 1;
   out_673445270312808434[11] = 0;
   out_673445270312808434[12] = 0;
   out_673445270312808434[13] = 0;
   out_673445270312808434[14] = 0;
   out_673445270312808434[15] = 0;
   out_673445270312808434[16] = 0;
   out_673445270312808434[17] = 0;
   out_673445270312808434[18] = 0;
   out_673445270312808434[19] = 0;
   out_673445270312808434[20] = 1;
   out_673445270312808434[21] = 0;
   out_673445270312808434[22] = 0;
   out_673445270312808434[23] = 0;
   out_673445270312808434[24] = 0;
   out_673445270312808434[25] = 0;
   out_673445270312808434[26] = 0;
   out_673445270312808434[27] = 0;
   out_673445270312808434[28] = 0;
   out_673445270312808434[29] = 0;
   out_673445270312808434[30] = 1;
   out_673445270312808434[31] = 0;
   out_673445270312808434[32] = 0;
   out_673445270312808434[33] = 0;
   out_673445270312808434[34] = 0;
   out_673445270312808434[35] = 0;
   out_673445270312808434[36] = 0;
   out_673445270312808434[37] = 0;
   out_673445270312808434[38] = 0;
   out_673445270312808434[39] = 0;
   out_673445270312808434[40] = 1;
   out_673445270312808434[41] = 0;
   out_673445270312808434[42] = 0;
   out_673445270312808434[43] = 0;
   out_673445270312808434[44] = 0;
   out_673445270312808434[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_673445270312808434[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_673445270312808434[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_673445270312808434[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_673445270312808434[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_673445270312808434[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_673445270312808434[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_673445270312808434[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_673445270312808434[53] = -9.8000000000000007*dt;
   out_673445270312808434[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_673445270312808434[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_673445270312808434[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_673445270312808434[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_673445270312808434[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_673445270312808434[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_673445270312808434[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_673445270312808434[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_673445270312808434[62] = 0;
   out_673445270312808434[63] = 0;
   out_673445270312808434[64] = 0;
   out_673445270312808434[65] = 0;
   out_673445270312808434[66] = 0;
   out_673445270312808434[67] = 0;
   out_673445270312808434[68] = 0;
   out_673445270312808434[69] = 0;
   out_673445270312808434[70] = 1;
   out_673445270312808434[71] = 0;
   out_673445270312808434[72] = 0;
   out_673445270312808434[73] = 0;
   out_673445270312808434[74] = 0;
   out_673445270312808434[75] = 0;
   out_673445270312808434[76] = 0;
   out_673445270312808434[77] = 0;
   out_673445270312808434[78] = 0;
   out_673445270312808434[79] = 0;
   out_673445270312808434[80] = 1;
}
void h_25(double *state, double *unused, double *out_5795691796466611661) {
   out_5795691796466611661[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1186979588426458488) {
   out_1186979588426458488[0] = 0;
   out_1186979588426458488[1] = 0;
   out_1186979588426458488[2] = 0;
   out_1186979588426458488[3] = 0;
   out_1186979588426458488[4] = 0;
   out_1186979588426458488[5] = 0;
   out_1186979588426458488[6] = 1;
   out_1186979588426458488[7] = 0;
   out_1186979588426458488[8] = 0;
}
void h_24(double *state, double *unused, double *out_1165476922925981880) {
   out_1165476922925981880[0] = state[4];
   out_1165476922925981880[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3412687372405327050) {
   out_3412687372405327050[0] = 0;
   out_3412687372405327050[1] = 0;
   out_3412687372405327050[2] = 0;
   out_3412687372405327050[3] = 0;
   out_3412687372405327050[4] = 1;
   out_3412687372405327050[5] = 0;
   out_3412687372405327050[6] = 0;
   out_3412687372405327050[7] = 0;
   out_3412687372405327050[8] = 0;
   out_3412687372405327050[9] = 0;
   out_3412687372405327050[10] = 0;
   out_3412687372405327050[11] = 0;
   out_3412687372405327050[12] = 0;
   out_3412687372405327050[13] = 0;
   out_3412687372405327050[14] = 1;
   out_3412687372405327050[15] = 0;
   out_3412687372405327050[16] = 0;
   out_3412687372405327050[17] = 0;
}
void h_30(double *state, double *unused, double *out_7091429700375966071) {
   out_7091429700375966071[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8103669929918075243) {
   out_8103669929918075243[0] = 0;
   out_8103669929918075243[1] = 0;
   out_8103669929918075243[2] = 0;
   out_8103669929918075243[3] = 0;
   out_8103669929918075243[4] = 1;
   out_8103669929918075243[5] = 0;
   out_8103669929918075243[6] = 0;
   out_8103669929918075243[7] = 0;
   out_8103669929918075243[8] = 0;
}
void h_26(double *state, double *unused, double *out_2949657553689358892) {
   out_2949657553689358892[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2554523730447597736) {
   out_2554523730447597736[0] = 0;
   out_2554523730447597736[1] = 0;
   out_2554523730447597736[2] = 0;
   out_2554523730447597736[3] = 0;
   out_2554523730447597736[4] = 0;
   out_2554523730447597736[5] = 0;
   out_2554523730447597736[6] = 0;
   out_2554523730447597736[7] = 1;
   out_2554523730447597736[8] = 0;
}
void h_27(double *state, double *unused, double *out_6772766789684224867) {
   out_6772766789684224867[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5928906618117650332) {
   out_5928906618117650332[0] = 0;
   out_5928906618117650332[1] = 0;
   out_5928906618117650332[2] = 0;
   out_5928906618117650332[3] = 1;
   out_5928906618117650332[4] = 0;
   out_5928906618117650332[5] = 0;
   out_5928906618117650332[6] = 0;
   out_5928906618117650332[7] = 0;
   out_5928906618117650332[8] = 0;
}
void h_29(double *state, double *unused, double *out_6733128343877477231) {
   out_6733128343877477231[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8613901274232467427) {
   out_8613901274232467427[0] = 0;
   out_8613901274232467427[1] = 1;
   out_8613901274232467427[2] = 0;
   out_8613901274232467427[3] = 0;
   out_8613901274232467427[4] = 0;
   out_8613901274232467427[5] = 0;
   out_8613901274232467427[6] = 0;
   out_8613901274232467427[7] = 0;
   out_8613901274232467427[8] = 0;
}
void h_28(double *state, double *unused, double *out_1130036918603268032) {
   out_1130036918603268032[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3531502257162936853) {
   out_3531502257162936853[0] = 1;
   out_3531502257162936853[1] = 0;
   out_3531502257162936853[2] = 0;
   out_3531502257162936853[3] = 0;
   out_3531502257162936853[4] = 0;
   out_3531502257162936853[5] = 0;
   out_3531502257162936853[6] = 0;
   out_3531502257162936853[7] = 0;
   out_3531502257162936853[8] = 0;
}
void h_31(double *state, double *unused, double *out_4011174800524398071) {
   out_4011174800524398071[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1217625550303418916) {
   out_1217625550303418916[0] = 0;
   out_1217625550303418916[1] = 0;
   out_1217625550303418916[2] = 0;
   out_1217625550303418916[3] = 0;
   out_1217625550303418916[4] = 0;
   out_1217625550303418916[5] = 0;
   out_1217625550303418916[6] = 0;
   out_1217625550303418916[7] = 0;
   out_1217625550303418916[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5379945950210905137) {
  err_fun(nom_x, delta_x, out_5379945950210905137);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6100139837532875915) {
  inv_err_fun(nom_x, true_x, out_6100139837532875915);
}
void car_H_mod_fun(double *state, double *out_8072557983707861366) {
  H_mod_fun(state, out_8072557983707861366);
}
void car_f_fun(double *state, double dt, double *out_3587256842187338616) {
  f_fun(state,  dt, out_3587256842187338616);
}
void car_F_fun(double *state, double dt, double *out_673445270312808434) {
  F_fun(state,  dt, out_673445270312808434);
}
void car_h_25(double *state, double *unused, double *out_5795691796466611661) {
  h_25(state, unused, out_5795691796466611661);
}
void car_H_25(double *state, double *unused, double *out_1186979588426458488) {
  H_25(state, unused, out_1186979588426458488);
}
void car_h_24(double *state, double *unused, double *out_1165476922925981880) {
  h_24(state, unused, out_1165476922925981880);
}
void car_H_24(double *state, double *unused, double *out_3412687372405327050) {
  H_24(state, unused, out_3412687372405327050);
}
void car_h_30(double *state, double *unused, double *out_7091429700375966071) {
  h_30(state, unused, out_7091429700375966071);
}
void car_H_30(double *state, double *unused, double *out_8103669929918075243) {
  H_30(state, unused, out_8103669929918075243);
}
void car_h_26(double *state, double *unused, double *out_2949657553689358892) {
  h_26(state, unused, out_2949657553689358892);
}
void car_H_26(double *state, double *unused, double *out_2554523730447597736) {
  H_26(state, unused, out_2554523730447597736);
}
void car_h_27(double *state, double *unused, double *out_6772766789684224867) {
  h_27(state, unused, out_6772766789684224867);
}
void car_H_27(double *state, double *unused, double *out_5928906618117650332) {
  H_27(state, unused, out_5928906618117650332);
}
void car_h_29(double *state, double *unused, double *out_6733128343877477231) {
  h_29(state, unused, out_6733128343877477231);
}
void car_H_29(double *state, double *unused, double *out_8613901274232467427) {
  H_29(state, unused, out_8613901274232467427);
}
void car_h_28(double *state, double *unused, double *out_1130036918603268032) {
  h_28(state, unused, out_1130036918603268032);
}
void car_H_28(double *state, double *unused, double *out_3531502257162936853) {
  H_28(state, unused, out_3531502257162936853);
}
void car_h_31(double *state, double *unused, double *out_4011174800524398071) {
  h_31(state, unused, out_4011174800524398071);
}
void car_H_31(double *state, double *unused, double *out_1217625550303418916) {
  H_31(state, unused, out_1217625550303418916);
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
