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
void err_fun(double *nom_x, double *delta_x, double *out_8546428699701313973) {
   out_8546428699701313973[0] = delta_x[0] + nom_x[0];
   out_8546428699701313973[1] = delta_x[1] + nom_x[1];
   out_8546428699701313973[2] = delta_x[2] + nom_x[2];
   out_8546428699701313973[3] = delta_x[3] + nom_x[3];
   out_8546428699701313973[4] = delta_x[4] + nom_x[4];
   out_8546428699701313973[5] = delta_x[5] + nom_x[5];
   out_8546428699701313973[6] = delta_x[6] + nom_x[6];
   out_8546428699701313973[7] = delta_x[7] + nom_x[7];
   out_8546428699701313973[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6276729264926819565) {
   out_6276729264926819565[0] = -nom_x[0] + true_x[0];
   out_6276729264926819565[1] = -nom_x[1] + true_x[1];
   out_6276729264926819565[2] = -nom_x[2] + true_x[2];
   out_6276729264926819565[3] = -nom_x[3] + true_x[3];
   out_6276729264926819565[4] = -nom_x[4] + true_x[4];
   out_6276729264926819565[5] = -nom_x[5] + true_x[5];
   out_6276729264926819565[6] = -nom_x[6] + true_x[6];
   out_6276729264926819565[7] = -nom_x[7] + true_x[7];
   out_6276729264926819565[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7387032395725791226) {
   out_7387032395725791226[0] = 1.0;
   out_7387032395725791226[1] = 0;
   out_7387032395725791226[2] = 0;
   out_7387032395725791226[3] = 0;
   out_7387032395725791226[4] = 0;
   out_7387032395725791226[5] = 0;
   out_7387032395725791226[6] = 0;
   out_7387032395725791226[7] = 0;
   out_7387032395725791226[8] = 0;
   out_7387032395725791226[9] = 0;
   out_7387032395725791226[10] = 1.0;
   out_7387032395725791226[11] = 0;
   out_7387032395725791226[12] = 0;
   out_7387032395725791226[13] = 0;
   out_7387032395725791226[14] = 0;
   out_7387032395725791226[15] = 0;
   out_7387032395725791226[16] = 0;
   out_7387032395725791226[17] = 0;
   out_7387032395725791226[18] = 0;
   out_7387032395725791226[19] = 0;
   out_7387032395725791226[20] = 1.0;
   out_7387032395725791226[21] = 0;
   out_7387032395725791226[22] = 0;
   out_7387032395725791226[23] = 0;
   out_7387032395725791226[24] = 0;
   out_7387032395725791226[25] = 0;
   out_7387032395725791226[26] = 0;
   out_7387032395725791226[27] = 0;
   out_7387032395725791226[28] = 0;
   out_7387032395725791226[29] = 0;
   out_7387032395725791226[30] = 1.0;
   out_7387032395725791226[31] = 0;
   out_7387032395725791226[32] = 0;
   out_7387032395725791226[33] = 0;
   out_7387032395725791226[34] = 0;
   out_7387032395725791226[35] = 0;
   out_7387032395725791226[36] = 0;
   out_7387032395725791226[37] = 0;
   out_7387032395725791226[38] = 0;
   out_7387032395725791226[39] = 0;
   out_7387032395725791226[40] = 1.0;
   out_7387032395725791226[41] = 0;
   out_7387032395725791226[42] = 0;
   out_7387032395725791226[43] = 0;
   out_7387032395725791226[44] = 0;
   out_7387032395725791226[45] = 0;
   out_7387032395725791226[46] = 0;
   out_7387032395725791226[47] = 0;
   out_7387032395725791226[48] = 0;
   out_7387032395725791226[49] = 0;
   out_7387032395725791226[50] = 1.0;
   out_7387032395725791226[51] = 0;
   out_7387032395725791226[52] = 0;
   out_7387032395725791226[53] = 0;
   out_7387032395725791226[54] = 0;
   out_7387032395725791226[55] = 0;
   out_7387032395725791226[56] = 0;
   out_7387032395725791226[57] = 0;
   out_7387032395725791226[58] = 0;
   out_7387032395725791226[59] = 0;
   out_7387032395725791226[60] = 1.0;
   out_7387032395725791226[61] = 0;
   out_7387032395725791226[62] = 0;
   out_7387032395725791226[63] = 0;
   out_7387032395725791226[64] = 0;
   out_7387032395725791226[65] = 0;
   out_7387032395725791226[66] = 0;
   out_7387032395725791226[67] = 0;
   out_7387032395725791226[68] = 0;
   out_7387032395725791226[69] = 0;
   out_7387032395725791226[70] = 1.0;
   out_7387032395725791226[71] = 0;
   out_7387032395725791226[72] = 0;
   out_7387032395725791226[73] = 0;
   out_7387032395725791226[74] = 0;
   out_7387032395725791226[75] = 0;
   out_7387032395725791226[76] = 0;
   out_7387032395725791226[77] = 0;
   out_7387032395725791226[78] = 0;
   out_7387032395725791226[79] = 0;
   out_7387032395725791226[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7992121703102262561) {
   out_7992121703102262561[0] = state[0];
   out_7992121703102262561[1] = state[1];
   out_7992121703102262561[2] = state[2];
   out_7992121703102262561[3] = state[3];
   out_7992121703102262561[4] = state[4];
   out_7992121703102262561[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7992121703102262561[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7992121703102262561[7] = state[7];
   out_7992121703102262561[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6052186748360722795) {
   out_6052186748360722795[0] = 1;
   out_6052186748360722795[1] = 0;
   out_6052186748360722795[2] = 0;
   out_6052186748360722795[3] = 0;
   out_6052186748360722795[4] = 0;
   out_6052186748360722795[5] = 0;
   out_6052186748360722795[6] = 0;
   out_6052186748360722795[7] = 0;
   out_6052186748360722795[8] = 0;
   out_6052186748360722795[9] = 0;
   out_6052186748360722795[10] = 1;
   out_6052186748360722795[11] = 0;
   out_6052186748360722795[12] = 0;
   out_6052186748360722795[13] = 0;
   out_6052186748360722795[14] = 0;
   out_6052186748360722795[15] = 0;
   out_6052186748360722795[16] = 0;
   out_6052186748360722795[17] = 0;
   out_6052186748360722795[18] = 0;
   out_6052186748360722795[19] = 0;
   out_6052186748360722795[20] = 1;
   out_6052186748360722795[21] = 0;
   out_6052186748360722795[22] = 0;
   out_6052186748360722795[23] = 0;
   out_6052186748360722795[24] = 0;
   out_6052186748360722795[25] = 0;
   out_6052186748360722795[26] = 0;
   out_6052186748360722795[27] = 0;
   out_6052186748360722795[28] = 0;
   out_6052186748360722795[29] = 0;
   out_6052186748360722795[30] = 1;
   out_6052186748360722795[31] = 0;
   out_6052186748360722795[32] = 0;
   out_6052186748360722795[33] = 0;
   out_6052186748360722795[34] = 0;
   out_6052186748360722795[35] = 0;
   out_6052186748360722795[36] = 0;
   out_6052186748360722795[37] = 0;
   out_6052186748360722795[38] = 0;
   out_6052186748360722795[39] = 0;
   out_6052186748360722795[40] = 1;
   out_6052186748360722795[41] = 0;
   out_6052186748360722795[42] = 0;
   out_6052186748360722795[43] = 0;
   out_6052186748360722795[44] = 0;
   out_6052186748360722795[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6052186748360722795[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6052186748360722795[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6052186748360722795[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6052186748360722795[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6052186748360722795[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6052186748360722795[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6052186748360722795[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6052186748360722795[53] = -9.8000000000000007*dt;
   out_6052186748360722795[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6052186748360722795[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6052186748360722795[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6052186748360722795[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6052186748360722795[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6052186748360722795[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6052186748360722795[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6052186748360722795[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6052186748360722795[62] = 0;
   out_6052186748360722795[63] = 0;
   out_6052186748360722795[64] = 0;
   out_6052186748360722795[65] = 0;
   out_6052186748360722795[66] = 0;
   out_6052186748360722795[67] = 0;
   out_6052186748360722795[68] = 0;
   out_6052186748360722795[69] = 0;
   out_6052186748360722795[70] = 1;
   out_6052186748360722795[71] = 0;
   out_6052186748360722795[72] = 0;
   out_6052186748360722795[73] = 0;
   out_6052186748360722795[74] = 0;
   out_6052186748360722795[75] = 0;
   out_6052186748360722795[76] = 0;
   out_6052186748360722795[77] = 0;
   out_6052186748360722795[78] = 0;
   out_6052186748360722795[79] = 0;
   out_6052186748360722795[80] = 1;
}
void h_25(double *state, double *unused, double *out_5253203141964689396) {
   out_5253203141964689396[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6823297006306510332) {
   out_6823297006306510332[0] = 0;
   out_6823297006306510332[1] = 0;
   out_6823297006306510332[2] = 0;
   out_6823297006306510332[3] = 0;
   out_6823297006306510332[4] = 0;
   out_6823297006306510332[5] = 0;
   out_6823297006306510332[6] = 1;
   out_6823297006306510332[7] = 0;
   out_6823297006306510332[8] = 0;
}
void h_24(double *state, double *unused, double *out_4448088281771825364) {
   out_4448088281771825364[0] = state[4];
   out_4448088281771825364[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2478580357452164122) {
   out_2478580357452164122[0] = 0;
   out_2478580357452164122[1] = 0;
   out_2478580357452164122[2] = 0;
   out_2478580357452164122[3] = 0;
   out_2478580357452164122[4] = 1;
   out_2478580357452164122[5] = 0;
   out_2478580357452164122[6] = 0;
   out_2478580357452164122[7] = 0;
   out_2478580357452164122[8] = 0;
   out_2478580357452164122[9] = 0;
   out_2478580357452164122[10] = 0;
   out_2478580357452164122[11] = 0;
   out_2478580357452164122[12] = 0;
   out_2478580357452164122[13] = 0;
   out_2478580357452164122[14] = 1;
   out_2478580357452164122[15] = 0;
   out_2478580357452164122[16] = 0;
   out_2478580357452164122[17] = 0;
}
void h_30(double *state, double *unused, double *out_9114886618393191048) {
   out_9114886618393191048[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4304964047799261705) {
   out_4304964047799261705[0] = 0;
   out_4304964047799261705[1] = 0;
   out_4304964047799261705[2] = 0;
   out_4304964047799261705[3] = 0;
   out_4304964047799261705[4] = 1;
   out_4304964047799261705[5] = 0;
   out_4304964047799261705[6] = 0;
   out_4304964047799261705[7] = 0;
   out_4304964047799261705[8] = 0;
}
void h_26(double *state, double *unused, double *out_6406977070144503880) {
   out_6406977070144503880[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7881943748528985060) {
   out_7881943748528985060[0] = 0;
   out_7881943748528985060[1] = 0;
   out_7881943748528985060[2] = 0;
   out_7881943748528985060[3] = 0;
   out_7881943748528985060[4] = 0;
   out_7881943748528985060[5] = 0;
   out_7881943748528985060[6] = 0;
   out_7881943748528985060[7] = 1;
   out_7881943748528985060[8] = 0;
}
void h_27(double *state, double *unused, double *out_8127123413026519497) {
   out_8127123413026519497[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6479727359599686616) {
   out_6479727359599686616[0] = 0;
   out_6479727359599686616[1] = 0;
   out_6479727359599686616[2] = 0;
   out_6479727359599686616[3] = 1;
   out_6479727359599686616[4] = 0;
   out_6479727359599686616[5] = 0;
   out_6479727359599686616[6] = 0;
   out_6479727359599686616[7] = 0;
   out_6479727359599686616[8] = 0;
}
void h_29(double *state, double *unused, double *out_8728541570639883051) {
   out_8728541570639883051[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3794732703484869521) {
   out_3794732703484869521[0] = 0;
   out_3794732703484869521[1] = 1;
   out_3794732703484869521[2] = 0;
   out_3794732703484869521[3] = 0;
   out_3794732703484869521[4] = 0;
   out_3794732703484869521[5] = 0;
   out_3794732703484869521[6] = 0;
   out_3794732703484869521[7] = 0;
   out_3794732703484869521[8] = 0;
}
void h_28(double *state, double *unused, double *out_2901216197191194550) {
   out_2901216197191194550[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8877131720554400095) {
   out_8877131720554400095[0] = 1;
   out_8877131720554400095[1] = 0;
   out_8877131720554400095[2] = 0;
   out_8877131720554400095[3] = 0;
   out_8877131720554400095[4] = 0;
   out_8877131720554400095[5] = 0;
   out_8877131720554400095[6] = 0;
   out_8877131720554400095[7] = 0;
   out_8877131720554400095[8] = 0;
}
void h_31(double *state, double *unused, double *out_5202243027854662809) {
   out_5202243027854662809[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7255735646295633584) {
   out_7255735646295633584[0] = 0;
   out_7255735646295633584[1] = 0;
   out_7255735646295633584[2] = 0;
   out_7255735646295633584[3] = 0;
   out_7255735646295633584[4] = 0;
   out_7255735646295633584[5] = 0;
   out_7255735646295633584[6] = 0;
   out_7255735646295633584[7] = 0;
   out_7255735646295633584[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8546428699701313973) {
  err_fun(nom_x, delta_x, out_8546428699701313973);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6276729264926819565) {
  inv_err_fun(nom_x, true_x, out_6276729264926819565);
}
void car_H_mod_fun(double *state, double *out_7387032395725791226) {
  H_mod_fun(state, out_7387032395725791226);
}
void car_f_fun(double *state, double dt, double *out_7992121703102262561) {
  f_fun(state,  dt, out_7992121703102262561);
}
void car_F_fun(double *state, double dt, double *out_6052186748360722795) {
  F_fun(state,  dt, out_6052186748360722795);
}
void car_h_25(double *state, double *unused, double *out_5253203141964689396) {
  h_25(state, unused, out_5253203141964689396);
}
void car_H_25(double *state, double *unused, double *out_6823297006306510332) {
  H_25(state, unused, out_6823297006306510332);
}
void car_h_24(double *state, double *unused, double *out_4448088281771825364) {
  h_24(state, unused, out_4448088281771825364);
}
void car_H_24(double *state, double *unused, double *out_2478580357452164122) {
  H_24(state, unused, out_2478580357452164122);
}
void car_h_30(double *state, double *unused, double *out_9114886618393191048) {
  h_30(state, unused, out_9114886618393191048);
}
void car_H_30(double *state, double *unused, double *out_4304964047799261705) {
  H_30(state, unused, out_4304964047799261705);
}
void car_h_26(double *state, double *unused, double *out_6406977070144503880) {
  h_26(state, unused, out_6406977070144503880);
}
void car_H_26(double *state, double *unused, double *out_7881943748528985060) {
  H_26(state, unused, out_7881943748528985060);
}
void car_h_27(double *state, double *unused, double *out_8127123413026519497) {
  h_27(state, unused, out_8127123413026519497);
}
void car_H_27(double *state, double *unused, double *out_6479727359599686616) {
  H_27(state, unused, out_6479727359599686616);
}
void car_h_29(double *state, double *unused, double *out_8728541570639883051) {
  h_29(state, unused, out_8728541570639883051);
}
void car_H_29(double *state, double *unused, double *out_3794732703484869521) {
  H_29(state, unused, out_3794732703484869521);
}
void car_h_28(double *state, double *unused, double *out_2901216197191194550) {
  h_28(state, unused, out_2901216197191194550);
}
void car_H_28(double *state, double *unused, double *out_8877131720554400095) {
  H_28(state, unused, out_8877131720554400095);
}
void car_h_31(double *state, double *unused, double *out_5202243027854662809) {
  h_31(state, unused, out_5202243027854662809);
}
void car_H_31(double *state, double *unused, double *out_7255735646295633584) {
  H_31(state, unused, out_7255735646295633584);
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
