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
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4341611149106626338) {
   out_4341611149106626338[0] = delta_x[0] + nom_x[0];
   out_4341611149106626338[1] = delta_x[1] + nom_x[1];
   out_4341611149106626338[2] = delta_x[2] + nom_x[2];
   out_4341611149106626338[3] = delta_x[3] + nom_x[3];
   out_4341611149106626338[4] = delta_x[4] + nom_x[4];
   out_4341611149106626338[5] = delta_x[5] + nom_x[5];
   out_4341611149106626338[6] = delta_x[6] + nom_x[6];
   out_4341611149106626338[7] = delta_x[7] + nom_x[7];
   out_4341611149106626338[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6563642220978654068) {
   out_6563642220978654068[0] = -nom_x[0] + true_x[0];
   out_6563642220978654068[1] = -nom_x[1] + true_x[1];
   out_6563642220978654068[2] = -nom_x[2] + true_x[2];
   out_6563642220978654068[3] = -nom_x[3] + true_x[3];
   out_6563642220978654068[4] = -nom_x[4] + true_x[4];
   out_6563642220978654068[5] = -nom_x[5] + true_x[5];
   out_6563642220978654068[6] = -nom_x[6] + true_x[6];
   out_6563642220978654068[7] = -nom_x[7] + true_x[7];
   out_6563642220978654068[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1862272950594096159) {
   out_1862272950594096159[0] = 1.0;
   out_1862272950594096159[1] = 0;
   out_1862272950594096159[2] = 0;
   out_1862272950594096159[3] = 0;
   out_1862272950594096159[4] = 0;
   out_1862272950594096159[5] = 0;
   out_1862272950594096159[6] = 0;
   out_1862272950594096159[7] = 0;
   out_1862272950594096159[8] = 0;
   out_1862272950594096159[9] = 0;
   out_1862272950594096159[10] = 1.0;
   out_1862272950594096159[11] = 0;
   out_1862272950594096159[12] = 0;
   out_1862272950594096159[13] = 0;
   out_1862272950594096159[14] = 0;
   out_1862272950594096159[15] = 0;
   out_1862272950594096159[16] = 0;
   out_1862272950594096159[17] = 0;
   out_1862272950594096159[18] = 0;
   out_1862272950594096159[19] = 0;
   out_1862272950594096159[20] = 1.0;
   out_1862272950594096159[21] = 0;
   out_1862272950594096159[22] = 0;
   out_1862272950594096159[23] = 0;
   out_1862272950594096159[24] = 0;
   out_1862272950594096159[25] = 0;
   out_1862272950594096159[26] = 0;
   out_1862272950594096159[27] = 0;
   out_1862272950594096159[28] = 0;
   out_1862272950594096159[29] = 0;
   out_1862272950594096159[30] = 1.0;
   out_1862272950594096159[31] = 0;
   out_1862272950594096159[32] = 0;
   out_1862272950594096159[33] = 0;
   out_1862272950594096159[34] = 0;
   out_1862272950594096159[35] = 0;
   out_1862272950594096159[36] = 0;
   out_1862272950594096159[37] = 0;
   out_1862272950594096159[38] = 0;
   out_1862272950594096159[39] = 0;
   out_1862272950594096159[40] = 1.0;
   out_1862272950594096159[41] = 0;
   out_1862272950594096159[42] = 0;
   out_1862272950594096159[43] = 0;
   out_1862272950594096159[44] = 0;
   out_1862272950594096159[45] = 0;
   out_1862272950594096159[46] = 0;
   out_1862272950594096159[47] = 0;
   out_1862272950594096159[48] = 0;
   out_1862272950594096159[49] = 0;
   out_1862272950594096159[50] = 1.0;
   out_1862272950594096159[51] = 0;
   out_1862272950594096159[52] = 0;
   out_1862272950594096159[53] = 0;
   out_1862272950594096159[54] = 0;
   out_1862272950594096159[55] = 0;
   out_1862272950594096159[56] = 0;
   out_1862272950594096159[57] = 0;
   out_1862272950594096159[58] = 0;
   out_1862272950594096159[59] = 0;
   out_1862272950594096159[60] = 1.0;
   out_1862272950594096159[61] = 0;
   out_1862272950594096159[62] = 0;
   out_1862272950594096159[63] = 0;
   out_1862272950594096159[64] = 0;
   out_1862272950594096159[65] = 0;
   out_1862272950594096159[66] = 0;
   out_1862272950594096159[67] = 0;
   out_1862272950594096159[68] = 0;
   out_1862272950594096159[69] = 0;
   out_1862272950594096159[70] = 1.0;
   out_1862272950594096159[71] = 0;
   out_1862272950594096159[72] = 0;
   out_1862272950594096159[73] = 0;
   out_1862272950594096159[74] = 0;
   out_1862272950594096159[75] = 0;
   out_1862272950594096159[76] = 0;
   out_1862272950594096159[77] = 0;
   out_1862272950594096159[78] = 0;
   out_1862272950594096159[79] = 0;
   out_1862272950594096159[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4060615684978379852) {
   out_4060615684978379852[0] = state[0];
   out_4060615684978379852[1] = state[1];
   out_4060615684978379852[2] = state[2];
   out_4060615684978379852[3] = state[3];
   out_4060615684978379852[4] = state[4];
   out_4060615684978379852[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4060615684978379852[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4060615684978379852[7] = state[7];
   out_4060615684978379852[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7512294491876261158) {
   out_7512294491876261158[0] = 1;
   out_7512294491876261158[1] = 0;
   out_7512294491876261158[2] = 0;
   out_7512294491876261158[3] = 0;
   out_7512294491876261158[4] = 0;
   out_7512294491876261158[5] = 0;
   out_7512294491876261158[6] = 0;
   out_7512294491876261158[7] = 0;
   out_7512294491876261158[8] = 0;
   out_7512294491876261158[9] = 0;
   out_7512294491876261158[10] = 1;
   out_7512294491876261158[11] = 0;
   out_7512294491876261158[12] = 0;
   out_7512294491876261158[13] = 0;
   out_7512294491876261158[14] = 0;
   out_7512294491876261158[15] = 0;
   out_7512294491876261158[16] = 0;
   out_7512294491876261158[17] = 0;
   out_7512294491876261158[18] = 0;
   out_7512294491876261158[19] = 0;
   out_7512294491876261158[20] = 1;
   out_7512294491876261158[21] = 0;
   out_7512294491876261158[22] = 0;
   out_7512294491876261158[23] = 0;
   out_7512294491876261158[24] = 0;
   out_7512294491876261158[25] = 0;
   out_7512294491876261158[26] = 0;
   out_7512294491876261158[27] = 0;
   out_7512294491876261158[28] = 0;
   out_7512294491876261158[29] = 0;
   out_7512294491876261158[30] = 1;
   out_7512294491876261158[31] = 0;
   out_7512294491876261158[32] = 0;
   out_7512294491876261158[33] = 0;
   out_7512294491876261158[34] = 0;
   out_7512294491876261158[35] = 0;
   out_7512294491876261158[36] = 0;
   out_7512294491876261158[37] = 0;
   out_7512294491876261158[38] = 0;
   out_7512294491876261158[39] = 0;
   out_7512294491876261158[40] = 1;
   out_7512294491876261158[41] = 0;
   out_7512294491876261158[42] = 0;
   out_7512294491876261158[43] = 0;
   out_7512294491876261158[44] = 0;
   out_7512294491876261158[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7512294491876261158[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7512294491876261158[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7512294491876261158[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7512294491876261158[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7512294491876261158[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7512294491876261158[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7512294491876261158[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7512294491876261158[53] = -9.8000000000000007*dt;
   out_7512294491876261158[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7512294491876261158[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7512294491876261158[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7512294491876261158[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7512294491876261158[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7512294491876261158[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7512294491876261158[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7512294491876261158[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7512294491876261158[62] = 0;
   out_7512294491876261158[63] = 0;
   out_7512294491876261158[64] = 0;
   out_7512294491876261158[65] = 0;
   out_7512294491876261158[66] = 0;
   out_7512294491876261158[67] = 0;
   out_7512294491876261158[68] = 0;
   out_7512294491876261158[69] = 0;
   out_7512294491876261158[70] = 1;
   out_7512294491876261158[71] = 0;
   out_7512294491876261158[72] = 0;
   out_7512294491876261158[73] = 0;
   out_7512294491876261158[74] = 0;
   out_7512294491876261158[75] = 0;
   out_7512294491876261158[76] = 0;
   out_7512294491876261158[77] = 0;
   out_7512294491876261158[78] = 0;
   out_7512294491876261158[79] = 0;
   out_7512294491876261158[80] = 1;
}
void h_25(double *state, double *unused, double *out_5848465558922810029) {
   out_5848465558922810029[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2604461143443793040) {
   out_2604461143443793040[0] = 0;
   out_2604461143443793040[1] = 0;
   out_2604461143443793040[2] = 0;
   out_2604461143443793040[3] = 0;
   out_2604461143443793040[4] = 0;
   out_2604461143443793040[5] = 0;
   out_2604461143443793040[6] = 1;
   out_2604461143443793040[7] = 0;
   out_2604461143443793040[8] = 0;
}
void h_24(double *state, double *unused, double *out_7826187274481621079) {
   out_7826187274481621079[0] = state[4];
   out_7826187274481621079[1] = state[5];
}
void H_24(double *state, double *unused, double *out_431811544438293474) {
   out_431811544438293474[0] = 0;
   out_431811544438293474[1] = 0;
   out_431811544438293474[2] = 0;
   out_431811544438293474[3] = 0;
   out_431811544438293474[4] = 1;
   out_431811544438293474[5] = 0;
   out_431811544438293474[6] = 0;
   out_431811544438293474[7] = 0;
   out_431811544438293474[8] = 0;
   out_431811544438293474[9] = 0;
   out_431811544438293474[10] = 0;
   out_431811544438293474[11] = 0;
   out_431811544438293474[12] = 0;
   out_431811544438293474[13] = 0;
   out_431811544438293474[14] = 1;
   out_431811544438293474[15] = 0;
   out_431811544438293474[16] = 0;
   out_431811544438293474[17] = 0;
}
void h_30(double *state, double *unused, double *out_2211016030668787679) {
   out_2211016030668787679[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5122794101951041667) {
   out_5122794101951041667[0] = 0;
   out_5122794101951041667[1] = 0;
   out_5122794101951041667[2] = 0;
   out_5122794101951041667[3] = 0;
   out_5122794101951041667[4] = 1;
   out_5122794101951041667[5] = 0;
   out_5122794101951041667[6] = 0;
   out_5122794101951041667[7] = 0;
   out_5122794101951041667[8] = 0;
}
void h_26(double *state, double *unused, double *out_7212689298329633324) {
   out_7212689298329633324[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1137042175430263184) {
   out_1137042175430263184[0] = 0;
   out_1137042175430263184[1] = 0;
   out_1137042175430263184[2] = 0;
   out_1137042175430263184[3] = 0;
   out_1137042175430263184[4] = 0;
   out_1137042175430263184[5] = 0;
   out_1137042175430263184[6] = 0;
   out_1137042175430263184[7] = 1;
   out_1137042175430263184[8] = 0;
}
void h_27(double *state, double *unused, double *out_8885251610367142714) {
   out_8885251610367142714[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2948030790150616756) {
   out_2948030790150616756[0] = 0;
   out_2948030790150616756[1] = 0;
   out_2948030790150616756[2] = 0;
   out_2948030790150616756[3] = 1;
   out_2948030790150616756[4] = 0;
   out_2948030790150616756[5] = 0;
   out_2948030790150616756[6] = 0;
   out_2948030790150616756[7] = 0;
   out_2948030790150616756[8] = 0;
}
void h_29(double *state, double *unused, double *out_2790657236992057966) {
   out_2790657236992057966[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5633025446265433851) {
   out_5633025446265433851[0] = 0;
   out_5633025446265433851[1] = 1;
   out_5633025446265433851[2] = 0;
   out_5633025446265433851[3] = 0;
   out_5633025446265433851[4] = 0;
   out_5633025446265433851[5] = 0;
   out_5633025446265433851[6] = 0;
   out_5633025446265433851[7] = 0;
   out_5633025446265433851[8] = 0;
}
void h_28(double *state, double *unused, double *out_4219625227456378339) {
   out_4219625227456378339[0] = state[0];
}
void H_28(double *state, double *unused, double *out_550626429195903277) {
   out_550626429195903277[0] = 1;
   out_550626429195903277[1] = 0;
   out_550626429195903277[2] = 0;
   out_550626429195903277[3] = 0;
   out_550626429195903277[4] = 0;
   out_550626429195903277[5] = 0;
   out_550626429195903277[6] = 0;
   out_550626429195903277[7] = 0;
   out_550626429195903277[8] = 0;
}
void h_31(double *state, double *unused, double *out_6123659621207315918) {
   out_6123659621207315918[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1763250277663614660) {
   out_1763250277663614660[0] = 0;
   out_1763250277663614660[1] = 0;
   out_1763250277663614660[2] = 0;
   out_1763250277663614660[3] = 0;
   out_1763250277663614660[4] = 0;
   out_1763250277663614660[5] = 0;
   out_1763250277663614660[6] = 0;
   out_1763250277663614660[7] = 0;
   out_1763250277663614660[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4341611149106626338) {
  err_fun(nom_x, delta_x, out_4341611149106626338);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6563642220978654068) {
  inv_err_fun(nom_x, true_x, out_6563642220978654068);
}
void car_H_mod_fun(double *state, double *out_1862272950594096159) {
  H_mod_fun(state, out_1862272950594096159);
}
void car_f_fun(double *state, double dt, double *out_4060615684978379852) {
  f_fun(state,  dt, out_4060615684978379852);
}
void car_F_fun(double *state, double dt, double *out_7512294491876261158) {
  F_fun(state,  dt, out_7512294491876261158);
}
void car_h_25(double *state, double *unused, double *out_5848465558922810029) {
  h_25(state, unused, out_5848465558922810029);
}
void car_H_25(double *state, double *unused, double *out_2604461143443793040) {
  H_25(state, unused, out_2604461143443793040);
}
void car_h_24(double *state, double *unused, double *out_7826187274481621079) {
  h_24(state, unused, out_7826187274481621079);
}
void car_H_24(double *state, double *unused, double *out_431811544438293474) {
  H_24(state, unused, out_431811544438293474);
}
void car_h_30(double *state, double *unused, double *out_2211016030668787679) {
  h_30(state, unused, out_2211016030668787679);
}
void car_H_30(double *state, double *unused, double *out_5122794101951041667) {
  H_30(state, unused, out_5122794101951041667);
}
void car_h_26(double *state, double *unused, double *out_7212689298329633324) {
  h_26(state, unused, out_7212689298329633324);
}
void car_H_26(double *state, double *unused, double *out_1137042175430263184) {
  H_26(state, unused, out_1137042175430263184);
}
void car_h_27(double *state, double *unused, double *out_8885251610367142714) {
  h_27(state, unused, out_8885251610367142714);
}
void car_H_27(double *state, double *unused, double *out_2948030790150616756) {
  H_27(state, unused, out_2948030790150616756);
}
void car_h_29(double *state, double *unused, double *out_2790657236992057966) {
  h_29(state, unused, out_2790657236992057966);
}
void car_H_29(double *state, double *unused, double *out_5633025446265433851) {
  H_29(state, unused, out_5633025446265433851);
}
void car_h_28(double *state, double *unused, double *out_4219625227456378339) {
  h_28(state, unused, out_4219625227456378339);
}
void car_H_28(double *state, double *unused, double *out_550626429195903277) {
  H_28(state, unused, out_550626429195903277);
}
void car_h_31(double *state, double *unused, double *out_6123659621207315918) {
  h_31(state, unused, out_6123659621207315918);
}
void car_H_31(double *state, double *unused, double *out_1763250277663614660) {
  H_31(state, unused, out_1763250277663614660);
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
