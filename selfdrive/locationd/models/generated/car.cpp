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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4793426118989952565) {
   out_4793426118989952565[0] = delta_x[0] + nom_x[0];
   out_4793426118989952565[1] = delta_x[1] + nom_x[1];
   out_4793426118989952565[2] = delta_x[2] + nom_x[2];
   out_4793426118989952565[3] = delta_x[3] + nom_x[3];
   out_4793426118989952565[4] = delta_x[4] + nom_x[4];
   out_4793426118989952565[5] = delta_x[5] + nom_x[5];
   out_4793426118989952565[6] = delta_x[6] + nom_x[6];
   out_4793426118989952565[7] = delta_x[7] + nom_x[7];
   out_4793426118989952565[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6104751408298337559) {
   out_6104751408298337559[0] = -nom_x[0] + true_x[0];
   out_6104751408298337559[1] = -nom_x[1] + true_x[1];
   out_6104751408298337559[2] = -nom_x[2] + true_x[2];
   out_6104751408298337559[3] = -nom_x[3] + true_x[3];
   out_6104751408298337559[4] = -nom_x[4] + true_x[4];
   out_6104751408298337559[5] = -nom_x[5] + true_x[5];
   out_6104751408298337559[6] = -nom_x[6] + true_x[6];
   out_6104751408298337559[7] = -nom_x[7] + true_x[7];
   out_6104751408298337559[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6067088922067929627) {
   out_6067088922067929627[0] = 1.0;
   out_6067088922067929627[1] = 0;
   out_6067088922067929627[2] = 0;
   out_6067088922067929627[3] = 0;
   out_6067088922067929627[4] = 0;
   out_6067088922067929627[5] = 0;
   out_6067088922067929627[6] = 0;
   out_6067088922067929627[7] = 0;
   out_6067088922067929627[8] = 0;
   out_6067088922067929627[9] = 0;
   out_6067088922067929627[10] = 1.0;
   out_6067088922067929627[11] = 0;
   out_6067088922067929627[12] = 0;
   out_6067088922067929627[13] = 0;
   out_6067088922067929627[14] = 0;
   out_6067088922067929627[15] = 0;
   out_6067088922067929627[16] = 0;
   out_6067088922067929627[17] = 0;
   out_6067088922067929627[18] = 0;
   out_6067088922067929627[19] = 0;
   out_6067088922067929627[20] = 1.0;
   out_6067088922067929627[21] = 0;
   out_6067088922067929627[22] = 0;
   out_6067088922067929627[23] = 0;
   out_6067088922067929627[24] = 0;
   out_6067088922067929627[25] = 0;
   out_6067088922067929627[26] = 0;
   out_6067088922067929627[27] = 0;
   out_6067088922067929627[28] = 0;
   out_6067088922067929627[29] = 0;
   out_6067088922067929627[30] = 1.0;
   out_6067088922067929627[31] = 0;
   out_6067088922067929627[32] = 0;
   out_6067088922067929627[33] = 0;
   out_6067088922067929627[34] = 0;
   out_6067088922067929627[35] = 0;
   out_6067088922067929627[36] = 0;
   out_6067088922067929627[37] = 0;
   out_6067088922067929627[38] = 0;
   out_6067088922067929627[39] = 0;
   out_6067088922067929627[40] = 1.0;
   out_6067088922067929627[41] = 0;
   out_6067088922067929627[42] = 0;
   out_6067088922067929627[43] = 0;
   out_6067088922067929627[44] = 0;
   out_6067088922067929627[45] = 0;
   out_6067088922067929627[46] = 0;
   out_6067088922067929627[47] = 0;
   out_6067088922067929627[48] = 0;
   out_6067088922067929627[49] = 0;
   out_6067088922067929627[50] = 1.0;
   out_6067088922067929627[51] = 0;
   out_6067088922067929627[52] = 0;
   out_6067088922067929627[53] = 0;
   out_6067088922067929627[54] = 0;
   out_6067088922067929627[55] = 0;
   out_6067088922067929627[56] = 0;
   out_6067088922067929627[57] = 0;
   out_6067088922067929627[58] = 0;
   out_6067088922067929627[59] = 0;
   out_6067088922067929627[60] = 1.0;
   out_6067088922067929627[61] = 0;
   out_6067088922067929627[62] = 0;
   out_6067088922067929627[63] = 0;
   out_6067088922067929627[64] = 0;
   out_6067088922067929627[65] = 0;
   out_6067088922067929627[66] = 0;
   out_6067088922067929627[67] = 0;
   out_6067088922067929627[68] = 0;
   out_6067088922067929627[69] = 0;
   out_6067088922067929627[70] = 1.0;
   out_6067088922067929627[71] = 0;
   out_6067088922067929627[72] = 0;
   out_6067088922067929627[73] = 0;
   out_6067088922067929627[74] = 0;
   out_6067088922067929627[75] = 0;
   out_6067088922067929627[76] = 0;
   out_6067088922067929627[77] = 0;
   out_6067088922067929627[78] = 0;
   out_6067088922067929627[79] = 0;
   out_6067088922067929627[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1262970013950409256) {
   out_1262970013950409256[0] = state[0];
   out_1262970013950409256[1] = state[1];
   out_1262970013950409256[2] = state[2];
   out_1262970013950409256[3] = state[3];
   out_1262970013950409256[4] = state[4];
   out_1262970013950409256[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1262970013950409256[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1262970013950409256[7] = state[7];
   out_1262970013950409256[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8280434690380054427) {
   out_8280434690380054427[0] = 1;
   out_8280434690380054427[1] = 0;
   out_8280434690380054427[2] = 0;
   out_8280434690380054427[3] = 0;
   out_8280434690380054427[4] = 0;
   out_8280434690380054427[5] = 0;
   out_8280434690380054427[6] = 0;
   out_8280434690380054427[7] = 0;
   out_8280434690380054427[8] = 0;
   out_8280434690380054427[9] = 0;
   out_8280434690380054427[10] = 1;
   out_8280434690380054427[11] = 0;
   out_8280434690380054427[12] = 0;
   out_8280434690380054427[13] = 0;
   out_8280434690380054427[14] = 0;
   out_8280434690380054427[15] = 0;
   out_8280434690380054427[16] = 0;
   out_8280434690380054427[17] = 0;
   out_8280434690380054427[18] = 0;
   out_8280434690380054427[19] = 0;
   out_8280434690380054427[20] = 1;
   out_8280434690380054427[21] = 0;
   out_8280434690380054427[22] = 0;
   out_8280434690380054427[23] = 0;
   out_8280434690380054427[24] = 0;
   out_8280434690380054427[25] = 0;
   out_8280434690380054427[26] = 0;
   out_8280434690380054427[27] = 0;
   out_8280434690380054427[28] = 0;
   out_8280434690380054427[29] = 0;
   out_8280434690380054427[30] = 1;
   out_8280434690380054427[31] = 0;
   out_8280434690380054427[32] = 0;
   out_8280434690380054427[33] = 0;
   out_8280434690380054427[34] = 0;
   out_8280434690380054427[35] = 0;
   out_8280434690380054427[36] = 0;
   out_8280434690380054427[37] = 0;
   out_8280434690380054427[38] = 0;
   out_8280434690380054427[39] = 0;
   out_8280434690380054427[40] = 1;
   out_8280434690380054427[41] = 0;
   out_8280434690380054427[42] = 0;
   out_8280434690380054427[43] = 0;
   out_8280434690380054427[44] = 0;
   out_8280434690380054427[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8280434690380054427[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8280434690380054427[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8280434690380054427[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8280434690380054427[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8280434690380054427[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8280434690380054427[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8280434690380054427[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8280434690380054427[53] = -9.8000000000000007*dt;
   out_8280434690380054427[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8280434690380054427[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8280434690380054427[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8280434690380054427[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8280434690380054427[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8280434690380054427[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8280434690380054427[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8280434690380054427[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8280434690380054427[62] = 0;
   out_8280434690380054427[63] = 0;
   out_8280434690380054427[64] = 0;
   out_8280434690380054427[65] = 0;
   out_8280434690380054427[66] = 0;
   out_8280434690380054427[67] = 0;
   out_8280434690380054427[68] = 0;
   out_8280434690380054427[69] = 0;
   out_8280434690380054427[70] = 1;
   out_8280434690380054427[71] = 0;
   out_8280434690380054427[72] = 0;
   out_8280434690380054427[73] = 0;
   out_8280434690380054427[74] = 0;
   out_8280434690380054427[75] = 0;
   out_8280434690380054427[76] = 0;
   out_8280434690380054427[77] = 0;
   out_8280434690380054427[78] = 0;
   out_8280434690380054427[79] = 0;
   out_8280434690380054427[80] = 1;
}
void h_25(double *state, double *unused, double *out_3171541189463855340) {
   out_3171541189463855340[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2582795789210079086) {
   out_2582795789210079086[0] = 0;
   out_2582795789210079086[1] = 0;
   out_2582795789210079086[2] = 0;
   out_2582795789210079086[3] = 0;
   out_2582795789210079086[4] = 0;
   out_2582795789210079086[5] = 0;
   out_2582795789210079086[6] = 1;
   out_2582795789210079086[7] = 0;
   out_2582795789210079086[8] = 0;
}
void h_24(double *state, double *unused, double *out_4459677364439223850) {
   out_4459677364439223850[0] = state[4];
   out_4459677364439223850[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2112338307166740362) {
   out_2112338307166740362[0] = 0;
   out_2112338307166740362[1] = 0;
   out_2112338307166740362[2] = 0;
   out_2112338307166740362[3] = 0;
   out_2112338307166740362[4] = 1;
   out_2112338307166740362[5] = 0;
   out_2112338307166740362[6] = 0;
   out_2112338307166740362[7] = 0;
   out_2112338307166740362[8] = 0;
   out_2112338307166740362[9] = 0;
   out_2112338307166740362[10] = 0;
   out_2112338307166740362[11] = 0;
   out_2112338307166740362[12] = 0;
   out_2112338307166740362[13] = 0;
   out_2112338307166740362[14] = 1;
   out_2112338307166740362[15] = 0;
   out_2112338307166740362[16] = 0;
   out_2112338307166740362[17] = 0;
}
void h_30(double *state, double *unused, double *out_3446735251748361229) {
   out_3446735251748361229[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1944900540917529112) {
   out_1944900540917529112[0] = 0;
   out_1944900540917529112[1] = 0;
   out_1944900540917529112[2] = 0;
   out_1944900540917529112[3] = 0;
   out_1944900540917529112[4] = 1;
   out_1944900540917529112[5] = 0;
   out_1944900540917529112[6] = 0;
   out_1944900540917529112[7] = 0;
   out_1944900540917529112[8] = 0;
}
void h_26(double *state, double *unused, double *out_738825703499674061) {
   out_738825703499674061[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1158707529663977138) {
   out_1158707529663977138[0] = 0;
   out_1158707529663977138[1] = 0;
   out_1158707529663977138[2] = 0;
   out_1158707529663977138[3] = 0;
   out_1158707529663977138[4] = 0;
   out_1158707529663977138[5] = 0;
   out_1158707529663977138[6] = 0;
   out_1158707529663977138[7] = 1;
   out_1158707529663977138[8] = 0;
}
void h_27(double *state, double *unused, double *out_4651469294038202300) {
   out_4651469294038202300[0] = state[3];
}
void H_27(double *state, double *unused, double *out_278693530266414105) {
   out_278693530266414105[0] = 0;
   out_278693530266414105[1] = 0;
   out_278693530266414105[2] = 0;
   out_278693530266414105[3] = 1;
   out_278693530266414105[4] = 0;
   out_278693530266414105[5] = 0;
   out_278693530266414105[6] = 0;
   out_278693530266414105[7] = 0;
   out_278693530266414105[8] = 0;
}
void h_29(double *state, double *unused, double *out_4050051136424838746) {
   out_4050051136424838746[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1434669196603136928) {
   out_1434669196603136928[0] = 0;
   out_1434669196603136928[1] = 1;
   out_1434669196603136928[2] = 0;
   out_1434669196603136928[3] = 0;
   out_1434669196603136928[4] = 0;
   out_1434669196603136928[5] = 0;
   out_1434669196603136928[6] = 0;
   out_1434669196603136928[7] = 0;
   out_1434669196603136928[8] = 0;
}
void h_28(double *state, double *unused, double *out_8569367563836024369) {
   out_8569367563836024369[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6517068213672667502) {
   out_6517068213672667502[0] = 1;
   out_6517068213672667502[1] = 0;
   out_6517068213672667502[2] = 0;
   out_6517068213672667502[3] = 0;
   out_6517068213672667502[4] = 0;
   out_6517068213672667502[5] = 0;
   out_6517068213672667502[6] = 0;
   out_6517068213672667502[7] = 0;
   out_6517068213672667502[8] = 0;
}
void h_31(double *state, double *unused, double *out_6635325407527154392) {
   out_6635325407527154392[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2613441751087039514) {
   out_2613441751087039514[0] = 0;
   out_2613441751087039514[1] = 0;
   out_2613441751087039514[2] = 0;
   out_2613441751087039514[3] = 0;
   out_2613441751087039514[4] = 0;
   out_2613441751087039514[5] = 0;
   out_2613441751087039514[6] = 0;
   out_2613441751087039514[7] = 0;
   out_2613441751087039514[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4793426118989952565) {
  err_fun(nom_x, delta_x, out_4793426118989952565);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6104751408298337559) {
  inv_err_fun(nom_x, true_x, out_6104751408298337559);
}
void car_H_mod_fun(double *state, double *out_6067088922067929627) {
  H_mod_fun(state, out_6067088922067929627);
}
void car_f_fun(double *state, double dt, double *out_1262970013950409256) {
  f_fun(state,  dt, out_1262970013950409256);
}
void car_F_fun(double *state, double dt, double *out_8280434690380054427) {
  F_fun(state,  dt, out_8280434690380054427);
}
void car_h_25(double *state, double *unused, double *out_3171541189463855340) {
  h_25(state, unused, out_3171541189463855340);
}
void car_H_25(double *state, double *unused, double *out_2582795789210079086) {
  H_25(state, unused, out_2582795789210079086);
}
void car_h_24(double *state, double *unused, double *out_4459677364439223850) {
  h_24(state, unused, out_4459677364439223850);
}
void car_H_24(double *state, double *unused, double *out_2112338307166740362) {
  H_24(state, unused, out_2112338307166740362);
}
void car_h_30(double *state, double *unused, double *out_3446735251748361229) {
  h_30(state, unused, out_3446735251748361229);
}
void car_H_30(double *state, double *unused, double *out_1944900540917529112) {
  H_30(state, unused, out_1944900540917529112);
}
void car_h_26(double *state, double *unused, double *out_738825703499674061) {
  h_26(state, unused, out_738825703499674061);
}
void car_H_26(double *state, double *unused, double *out_1158707529663977138) {
  H_26(state, unused, out_1158707529663977138);
}
void car_h_27(double *state, double *unused, double *out_4651469294038202300) {
  h_27(state, unused, out_4651469294038202300);
}
void car_H_27(double *state, double *unused, double *out_278693530266414105) {
  H_27(state, unused, out_278693530266414105);
}
void car_h_29(double *state, double *unused, double *out_4050051136424838746) {
  h_29(state, unused, out_4050051136424838746);
}
void car_H_29(double *state, double *unused, double *out_1434669196603136928) {
  H_29(state, unused, out_1434669196603136928);
}
void car_h_28(double *state, double *unused, double *out_8569367563836024369) {
  h_28(state, unused, out_8569367563836024369);
}
void car_H_28(double *state, double *unused, double *out_6517068213672667502) {
  H_28(state, unused, out_6517068213672667502);
}
void car_h_31(double *state, double *unused, double *out_6635325407527154392) {
  h_31(state, unused, out_6635325407527154392);
}
void car_H_31(double *state, double *unused, double *out_2613441751087039514) {
  H_31(state, unused, out_2613441751087039514);
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
