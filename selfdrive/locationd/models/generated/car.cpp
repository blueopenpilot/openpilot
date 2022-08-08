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
void err_fun(double *nom_x, double *delta_x, double *out_1070714730182928934) {
   out_1070714730182928934[0] = delta_x[0] + nom_x[0];
   out_1070714730182928934[1] = delta_x[1] + nom_x[1];
   out_1070714730182928934[2] = delta_x[2] + nom_x[2];
   out_1070714730182928934[3] = delta_x[3] + nom_x[3];
   out_1070714730182928934[4] = delta_x[4] + nom_x[4];
   out_1070714730182928934[5] = delta_x[5] + nom_x[5];
   out_1070714730182928934[6] = delta_x[6] + nom_x[6];
   out_1070714730182928934[7] = delta_x[7] + nom_x[7];
   out_1070714730182928934[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1165789664415453656) {
   out_1165789664415453656[0] = -nom_x[0] + true_x[0];
   out_1165789664415453656[1] = -nom_x[1] + true_x[1];
   out_1165789664415453656[2] = -nom_x[2] + true_x[2];
   out_1165789664415453656[3] = -nom_x[3] + true_x[3];
   out_1165789664415453656[4] = -nom_x[4] + true_x[4];
   out_1165789664415453656[5] = -nom_x[5] + true_x[5];
   out_1165789664415453656[6] = -nom_x[6] + true_x[6];
   out_1165789664415453656[7] = -nom_x[7] + true_x[7];
   out_1165789664415453656[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_585124696828107175) {
   out_585124696828107175[0] = 1.0;
   out_585124696828107175[1] = 0;
   out_585124696828107175[2] = 0;
   out_585124696828107175[3] = 0;
   out_585124696828107175[4] = 0;
   out_585124696828107175[5] = 0;
   out_585124696828107175[6] = 0;
   out_585124696828107175[7] = 0;
   out_585124696828107175[8] = 0;
   out_585124696828107175[9] = 0;
   out_585124696828107175[10] = 1.0;
   out_585124696828107175[11] = 0;
   out_585124696828107175[12] = 0;
   out_585124696828107175[13] = 0;
   out_585124696828107175[14] = 0;
   out_585124696828107175[15] = 0;
   out_585124696828107175[16] = 0;
   out_585124696828107175[17] = 0;
   out_585124696828107175[18] = 0;
   out_585124696828107175[19] = 0;
   out_585124696828107175[20] = 1.0;
   out_585124696828107175[21] = 0;
   out_585124696828107175[22] = 0;
   out_585124696828107175[23] = 0;
   out_585124696828107175[24] = 0;
   out_585124696828107175[25] = 0;
   out_585124696828107175[26] = 0;
   out_585124696828107175[27] = 0;
   out_585124696828107175[28] = 0;
   out_585124696828107175[29] = 0;
   out_585124696828107175[30] = 1.0;
   out_585124696828107175[31] = 0;
   out_585124696828107175[32] = 0;
   out_585124696828107175[33] = 0;
   out_585124696828107175[34] = 0;
   out_585124696828107175[35] = 0;
   out_585124696828107175[36] = 0;
   out_585124696828107175[37] = 0;
   out_585124696828107175[38] = 0;
   out_585124696828107175[39] = 0;
   out_585124696828107175[40] = 1.0;
   out_585124696828107175[41] = 0;
   out_585124696828107175[42] = 0;
   out_585124696828107175[43] = 0;
   out_585124696828107175[44] = 0;
   out_585124696828107175[45] = 0;
   out_585124696828107175[46] = 0;
   out_585124696828107175[47] = 0;
   out_585124696828107175[48] = 0;
   out_585124696828107175[49] = 0;
   out_585124696828107175[50] = 1.0;
   out_585124696828107175[51] = 0;
   out_585124696828107175[52] = 0;
   out_585124696828107175[53] = 0;
   out_585124696828107175[54] = 0;
   out_585124696828107175[55] = 0;
   out_585124696828107175[56] = 0;
   out_585124696828107175[57] = 0;
   out_585124696828107175[58] = 0;
   out_585124696828107175[59] = 0;
   out_585124696828107175[60] = 1.0;
   out_585124696828107175[61] = 0;
   out_585124696828107175[62] = 0;
   out_585124696828107175[63] = 0;
   out_585124696828107175[64] = 0;
   out_585124696828107175[65] = 0;
   out_585124696828107175[66] = 0;
   out_585124696828107175[67] = 0;
   out_585124696828107175[68] = 0;
   out_585124696828107175[69] = 0;
   out_585124696828107175[70] = 1.0;
   out_585124696828107175[71] = 0;
   out_585124696828107175[72] = 0;
   out_585124696828107175[73] = 0;
   out_585124696828107175[74] = 0;
   out_585124696828107175[75] = 0;
   out_585124696828107175[76] = 0;
   out_585124696828107175[77] = 0;
   out_585124696828107175[78] = 0;
   out_585124696828107175[79] = 0;
   out_585124696828107175[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7399759131319460689) {
   out_7399759131319460689[0] = state[0];
   out_7399759131319460689[1] = state[1];
   out_7399759131319460689[2] = state[2];
   out_7399759131319460689[3] = state[3];
   out_7399759131319460689[4] = state[4];
   out_7399759131319460689[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7399759131319460689[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7399759131319460689[7] = state[7];
   out_7399759131319460689[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6459431986806209049) {
   out_6459431986806209049[0] = 1;
   out_6459431986806209049[1] = 0;
   out_6459431986806209049[2] = 0;
   out_6459431986806209049[3] = 0;
   out_6459431986806209049[4] = 0;
   out_6459431986806209049[5] = 0;
   out_6459431986806209049[6] = 0;
   out_6459431986806209049[7] = 0;
   out_6459431986806209049[8] = 0;
   out_6459431986806209049[9] = 0;
   out_6459431986806209049[10] = 1;
   out_6459431986806209049[11] = 0;
   out_6459431986806209049[12] = 0;
   out_6459431986806209049[13] = 0;
   out_6459431986806209049[14] = 0;
   out_6459431986806209049[15] = 0;
   out_6459431986806209049[16] = 0;
   out_6459431986806209049[17] = 0;
   out_6459431986806209049[18] = 0;
   out_6459431986806209049[19] = 0;
   out_6459431986806209049[20] = 1;
   out_6459431986806209049[21] = 0;
   out_6459431986806209049[22] = 0;
   out_6459431986806209049[23] = 0;
   out_6459431986806209049[24] = 0;
   out_6459431986806209049[25] = 0;
   out_6459431986806209049[26] = 0;
   out_6459431986806209049[27] = 0;
   out_6459431986806209049[28] = 0;
   out_6459431986806209049[29] = 0;
   out_6459431986806209049[30] = 1;
   out_6459431986806209049[31] = 0;
   out_6459431986806209049[32] = 0;
   out_6459431986806209049[33] = 0;
   out_6459431986806209049[34] = 0;
   out_6459431986806209049[35] = 0;
   out_6459431986806209049[36] = 0;
   out_6459431986806209049[37] = 0;
   out_6459431986806209049[38] = 0;
   out_6459431986806209049[39] = 0;
   out_6459431986806209049[40] = 1;
   out_6459431986806209049[41] = 0;
   out_6459431986806209049[42] = 0;
   out_6459431986806209049[43] = 0;
   out_6459431986806209049[44] = 0;
   out_6459431986806209049[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6459431986806209049[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6459431986806209049[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6459431986806209049[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6459431986806209049[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6459431986806209049[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6459431986806209049[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6459431986806209049[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6459431986806209049[53] = -9.8000000000000007*dt;
   out_6459431986806209049[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6459431986806209049[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6459431986806209049[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6459431986806209049[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6459431986806209049[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6459431986806209049[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6459431986806209049[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6459431986806209049[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6459431986806209049[62] = 0;
   out_6459431986806209049[63] = 0;
   out_6459431986806209049[64] = 0;
   out_6459431986806209049[65] = 0;
   out_6459431986806209049[66] = 0;
   out_6459431986806209049[67] = 0;
   out_6459431986806209049[68] = 0;
   out_6459431986806209049[69] = 0;
   out_6459431986806209049[70] = 1;
   out_6459431986806209049[71] = 0;
   out_6459431986806209049[72] = 0;
   out_6459431986806209049[73] = 0;
   out_6459431986806209049[74] = 0;
   out_6459431986806209049[75] = 0;
   out_6459431986806209049[76] = 0;
   out_6459431986806209049[77] = 0;
   out_6459431986806209049[78] = 0;
   out_6459431986806209049[79] = 0;
   out_6459431986806209049[80] = 1;
}
void h_25(double *state, double *unused, double *out_1968240230009004181) {
   out_1968240230009004181[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5940609591942357782) {
   out_5940609591942357782[0] = 0;
   out_5940609591942357782[1] = 0;
   out_5940609591942357782[2] = 0;
   out_5940609591942357782[3] = 0;
   out_5940609591942357782[4] = 0;
   out_5940609591942357782[5] = 0;
   out_5940609591942357782[6] = 1;
   out_5940609591942357782[7] = 0;
   out_5940609591942357782[8] = 0;
}
void h_24(double *state, double *unused, double *out_8990479235979821569) {
   out_8990479235979821569[0] = state[4];
   out_8990479235979821569[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3278069295697998609) {
   out_3278069295697998609[0] = 0;
   out_3278069295697998609[1] = 0;
   out_3278069295697998609[2] = 0;
   out_3278069295697998609[3] = 0;
   out_3278069295697998609[4] = 1;
   out_3278069295697998609[5] = 0;
   out_3278069295697998609[6] = 0;
   out_3278069295697998609[7] = 0;
   out_3278069295697998609[8] = 0;
   out_3278069295697998609[9] = 0;
   out_3278069295697998609[10] = 0;
   out_3278069295697998609[11] = 0;
   out_3278069295697998609[12] = 0;
   out_3278069295697998609[13] = 0;
   out_3278069295697998609[14] = 1;
   out_3278069295697998609[15] = 0;
   out_3278069295697998609[16] = 0;
   out_3278069295697998609[17] = 0;
}
void h_30(double *state, double *unused, double *out_1693046167724498292) {
   out_1693046167724498292[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5811270644799117712) {
   out_5811270644799117712[0] = 0;
   out_5811270644799117712[1] = 0;
   out_5811270644799117712[2] = 0;
   out_5811270644799117712[3] = 0;
   out_5811270644799117712[4] = 1;
   out_5811270644799117712[5] = 0;
   out_5811270644799117712[6] = 0;
   out_5811270644799117712[7] = 0;
   out_5811270644799117712[8] = 0;
}
void h_26(double *state, double *unused, double *out_911359153084199359) {
   out_911359153084199359[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2199106273068301558) {
   out_2199106273068301558[0] = 0;
   out_2199106273068301558[1] = 0;
   out_2199106273068301558[2] = 0;
   out_2199106273068301558[3] = 0;
   out_2199106273068301558[4] = 0;
   out_2199106273068301558[5] = 0;
   out_2199106273068301558[6] = 0;
   out_2199106273068301558[7] = 1;
   out_2199106273068301558[8] = 0;
}
void h_27(double *state, double *unused, double *out_7310107465895034744) {
   out_7310107465895034744[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3636507332998692801) {
   out_3636507332998692801[0] = 0;
   out_3636507332998692801[1] = 0;
   out_3636507332998692801[2] = 0;
   out_3636507332998692801[3] = 1;
   out_3636507332998692801[4] = 0;
   out_3636507332998692801[5] = 0;
   out_3636507332998692801[6] = 0;
   out_3636507332998692801[7] = 0;
   out_3636507332998692801[8] = 0;
}
void h_29(double *state, double *unused, double *out_9128189602153199105) {
   out_9128189602153199105[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1923144606129141768) {
   out_1923144606129141768[0] = 0;
   out_1923144606129141768[1] = 1;
   out_1923144606129141768[2] = 0;
   out_1923144606129141768[3] = 0;
   out_1923144606129141768[4] = 0;
   out_1923144606129141768[5] = 0;
   out_1923144606129141768[6] = 0;
   out_1923144606129141768[7] = 0;
   out_1923144606129141768[8] = 0;
}
void h_28(double *state, double *unused, double *out_2308434198234189099) {
   out_2308434198234189099[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3159254410940388806) {
   out_3159254410940388806[0] = 1;
   out_3159254410940388806[1] = 0;
   out_3159254410940388806[2] = 0;
   out_3159254410940388806[3] = 0;
   out_3159254410940388806[4] = 0;
   out_3159254410940388806[5] = 0;
   out_3159254410940388806[6] = 0;
   out_3159254410940388806[7] = 0;
   out_3159254410940388806[8] = 0;
}
void h_31(double *state, double *unused, double *out_6671637246709537703) {
   out_6671637246709537703[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5971255553819318210) {
   out_5971255553819318210[0] = 0;
   out_5971255553819318210[1] = 0;
   out_5971255553819318210[2] = 0;
   out_5971255553819318210[3] = 0;
   out_5971255553819318210[4] = 0;
   out_5971255553819318210[5] = 0;
   out_5971255553819318210[6] = 0;
   out_5971255553819318210[7] = 0;
   out_5971255553819318210[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1070714730182928934) {
  err_fun(nom_x, delta_x, out_1070714730182928934);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1165789664415453656) {
  inv_err_fun(nom_x, true_x, out_1165789664415453656);
}
void car_H_mod_fun(double *state, double *out_585124696828107175) {
  H_mod_fun(state, out_585124696828107175);
}
void car_f_fun(double *state, double dt, double *out_7399759131319460689) {
  f_fun(state,  dt, out_7399759131319460689);
}
void car_F_fun(double *state, double dt, double *out_6459431986806209049) {
  F_fun(state,  dt, out_6459431986806209049);
}
void car_h_25(double *state, double *unused, double *out_1968240230009004181) {
  h_25(state, unused, out_1968240230009004181);
}
void car_H_25(double *state, double *unused, double *out_5940609591942357782) {
  H_25(state, unused, out_5940609591942357782);
}
void car_h_24(double *state, double *unused, double *out_8990479235979821569) {
  h_24(state, unused, out_8990479235979821569);
}
void car_H_24(double *state, double *unused, double *out_3278069295697998609) {
  H_24(state, unused, out_3278069295697998609);
}
void car_h_30(double *state, double *unused, double *out_1693046167724498292) {
  h_30(state, unused, out_1693046167724498292);
}
void car_H_30(double *state, double *unused, double *out_5811270644799117712) {
  H_30(state, unused, out_5811270644799117712);
}
void car_h_26(double *state, double *unused, double *out_911359153084199359) {
  h_26(state, unused, out_911359153084199359);
}
void car_H_26(double *state, double *unused, double *out_2199106273068301558) {
  H_26(state, unused, out_2199106273068301558);
}
void car_h_27(double *state, double *unused, double *out_7310107465895034744) {
  h_27(state, unused, out_7310107465895034744);
}
void car_H_27(double *state, double *unused, double *out_3636507332998692801) {
  H_27(state, unused, out_3636507332998692801);
}
void car_h_29(double *state, double *unused, double *out_9128189602153199105) {
  h_29(state, unused, out_9128189602153199105);
}
void car_H_29(double *state, double *unused, double *out_1923144606129141768) {
  H_29(state, unused, out_1923144606129141768);
}
void car_h_28(double *state, double *unused, double *out_2308434198234189099) {
  h_28(state, unused, out_2308434198234189099);
}
void car_H_28(double *state, double *unused, double *out_3159254410940388806) {
  H_28(state, unused, out_3159254410940388806);
}
void car_h_31(double *state, double *unused, double *out_6671637246709537703) {
  h_31(state, unused, out_6671637246709537703);
}
void car_H_31(double *state, double *unused, double *out_5971255553819318210) {
  H_31(state, unused, out_5971255553819318210);
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
