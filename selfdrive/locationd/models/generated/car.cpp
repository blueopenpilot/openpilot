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
void err_fun(double *nom_x, double *delta_x, double *out_4219125439312905275) {
   out_4219125439312905275[0] = delta_x[0] + nom_x[0];
   out_4219125439312905275[1] = delta_x[1] + nom_x[1];
   out_4219125439312905275[2] = delta_x[2] + nom_x[2];
   out_4219125439312905275[3] = delta_x[3] + nom_x[3];
   out_4219125439312905275[4] = delta_x[4] + nom_x[4];
   out_4219125439312905275[5] = delta_x[5] + nom_x[5];
   out_4219125439312905275[6] = delta_x[6] + nom_x[6];
   out_4219125439312905275[7] = delta_x[7] + nom_x[7];
   out_4219125439312905275[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8640473231899427946) {
   out_8640473231899427946[0] = -nom_x[0] + true_x[0];
   out_8640473231899427946[1] = -nom_x[1] + true_x[1];
   out_8640473231899427946[2] = -nom_x[2] + true_x[2];
   out_8640473231899427946[3] = -nom_x[3] + true_x[3];
   out_8640473231899427946[4] = -nom_x[4] + true_x[4];
   out_8640473231899427946[5] = -nom_x[5] + true_x[5];
   out_8640473231899427946[6] = -nom_x[6] + true_x[6];
   out_8640473231899427946[7] = -nom_x[7] + true_x[7];
   out_8640473231899427946[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6403599997404205727) {
   out_6403599997404205727[0] = 1.0;
   out_6403599997404205727[1] = 0;
   out_6403599997404205727[2] = 0;
   out_6403599997404205727[3] = 0;
   out_6403599997404205727[4] = 0;
   out_6403599997404205727[5] = 0;
   out_6403599997404205727[6] = 0;
   out_6403599997404205727[7] = 0;
   out_6403599997404205727[8] = 0;
   out_6403599997404205727[9] = 0;
   out_6403599997404205727[10] = 1.0;
   out_6403599997404205727[11] = 0;
   out_6403599997404205727[12] = 0;
   out_6403599997404205727[13] = 0;
   out_6403599997404205727[14] = 0;
   out_6403599997404205727[15] = 0;
   out_6403599997404205727[16] = 0;
   out_6403599997404205727[17] = 0;
   out_6403599997404205727[18] = 0;
   out_6403599997404205727[19] = 0;
   out_6403599997404205727[20] = 1.0;
   out_6403599997404205727[21] = 0;
   out_6403599997404205727[22] = 0;
   out_6403599997404205727[23] = 0;
   out_6403599997404205727[24] = 0;
   out_6403599997404205727[25] = 0;
   out_6403599997404205727[26] = 0;
   out_6403599997404205727[27] = 0;
   out_6403599997404205727[28] = 0;
   out_6403599997404205727[29] = 0;
   out_6403599997404205727[30] = 1.0;
   out_6403599997404205727[31] = 0;
   out_6403599997404205727[32] = 0;
   out_6403599997404205727[33] = 0;
   out_6403599997404205727[34] = 0;
   out_6403599997404205727[35] = 0;
   out_6403599997404205727[36] = 0;
   out_6403599997404205727[37] = 0;
   out_6403599997404205727[38] = 0;
   out_6403599997404205727[39] = 0;
   out_6403599997404205727[40] = 1.0;
   out_6403599997404205727[41] = 0;
   out_6403599997404205727[42] = 0;
   out_6403599997404205727[43] = 0;
   out_6403599997404205727[44] = 0;
   out_6403599997404205727[45] = 0;
   out_6403599997404205727[46] = 0;
   out_6403599997404205727[47] = 0;
   out_6403599997404205727[48] = 0;
   out_6403599997404205727[49] = 0;
   out_6403599997404205727[50] = 1.0;
   out_6403599997404205727[51] = 0;
   out_6403599997404205727[52] = 0;
   out_6403599997404205727[53] = 0;
   out_6403599997404205727[54] = 0;
   out_6403599997404205727[55] = 0;
   out_6403599997404205727[56] = 0;
   out_6403599997404205727[57] = 0;
   out_6403599997404205727[58] = 0;
   out_6403599997404205727[59] = 0;
   out_6403599997404205727[60] = 1.0;
   out_6403599997404205727[61] = 0;
   out_6403599997404205727[62] = 0;
   out_6403599997404205727[63] = 0;
   out_6403599997404205727[64] = 0;
   out_6403599997404205727[65] = 0;
   out_6403599997404205727[66] = 0;
   out_6403599997404205727[67] = 0;
   out_6403599997404205727[68] = 0;
   out_6403599997404205727[69] = 0;
   out_6403599997404205727[70] = 1.0;
   out_6403599997404205727[71] = 0;
   out_6403599997404205727[72] = 0;
   out_6403599997404205727[73] = 0;
   out_6403599997404205727[74] = 0;
   out_6403599997404205727[75] = 0;
   out_6403599997404205727[76] = 0;
   out_6403599997404205727[77] = 0;
   out_6403599997404205727[78] = 0;
   out_6403599997404205727[79] = 0;
   out_6403599997404205727[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8494721620328521265) {
   out_8494721620328521265[0] = state[0];
   out_8494721620328521265[1] = state[1];
   out_8494721620328521265[2] = state[2];
   out_8494721620328521265[3] = state[3];
   out_8494721620328521265[4] = state[4];
   out_8494721620328521265[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8494721620328521265[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8494721620328521265[7] = state[7];
   out_8494721620328521265[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7925611537347342114) {
   out_7925611537347342114[0] = 1;
   out_7925611537347342114[1] = 0;
   out_7925611537347342114[2] = 0;
   out_7925611537347342114[3] = 0;
   out_7925611537347342114[4] = 0;
   out_7925611537347342114[5] = 0;
   out_7925611537347342114[6] = 0;
   out_7925611537347342114[7] = 0;
   out_7925611537347342114[8] = 0;
   out_7925611537347342114[9] = 0;
   out_7925611537347342114[10] = 1;
   out_7925611537347342114[11] = 0;
   out_7925611537347342114[12] = 0;
   out_7925611537347342114[13] = 0;
   out_7925611537347342114[14] = 0;
   out_7925611537347342114[15] = 0;
   out_7925611537347342114[16] = 0;
   out_7925611537347342114[17] = 0;
   out_7925611537347342114[18] = 0;
   out_7925611537347342114[19] = 0;
   out_7925611537347342114[20] = 1;
   out_7925611537347342114[21] = 0;
   out_7925611537347342114[22] = 0;
   out_7925611537347342114[23] = 0;
   out_7925611537347342114[24] = 0;
   out_7925611537347342114[25] = 0;
   out_7925611537347342114[26] = 0;
   out_7925611537347342114[27] = 0;
   out_7925611537347342114[28] = 0;
   out_7925611537347342114[29] = 0;
   out_7925611537347342114[30] = 1;
   out_7925611537347342114[31] = 0;
   out_7925611537347342114[32] = 0;
   out_7925611537347342114[33] = 0;
   out_7925611537347342114[34] = 0;
   out_7925611537347342114[35] = 0;
   out_7925611537347342114[36] = 0;
   out_7925611537347342114[37] = 0;
   out_7925611537347342114[38] = 0;
   out_7925611537347342114[39] = 0;
   out_7925611537347342114[40] = 1;
   out_7925611537347342114[41] = 0;
   out_7925611537347342114[42] = 0;
   out_7925611537347342114[43] = 0;
   out_7925611537347342114[44] = 0;
   out_7925611537347342114[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7925611537347342114[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7925611537347342114[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7925611537347342114[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7925611537347342114[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7925611537347342114[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7925611537347342114[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7925611537347342114[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7925611537347342114[53] = -9.8000000000000007*dt;
   out_7925611537347342114[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7925611537347342114[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7925611537347342114[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7925611537347342114[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7925611537347342114[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7925611537347342114[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7925611537347342114[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7925611537347342114[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7925611537347342114[62] = 0;
   out_7925611537347342114[63] = 0;
   out_7925611537347342114[64] = 0;
   out_7925611537347342114[65] = 0;
   out_7925611537347342114[66] = 0;
   out_7925611537347342114[67] = 0;
   out_7925611537347342114[68] = 0;
   out_7925611537347342114[69] = 0;
   out_7925611537347342114[70] = 1;
   out_7925611537347342114[71] = 0;
   out_7925611537347342114[72] = 0;
   out_7925611537347342114[73] = 0;
   out_7925611537347342114[74] = 0;
   out_7925611537347342114[75] = 0;
   out_7925611537347342114[76] = 0;
   out_7925611537347342114[77] = 0;
   out_7925611537347342114[78] = 0;
   out_7925611537347342114[79] = 0;
   out_7925611537347342114[80] = 1;
}
void h_25(double *state, double *unused, double *out_8319850071724330228) {
   out_8319850071724330228[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3040935136717207570) {
   out_3040935136717207570[0] = 0;
   out_3040935136717207570[1] = 0;
   out_3040935136717207570[2] = 0;
   out_3040935136717207570[3] = 0;
   out_3040935136717207570[4] = 0;
   out_3040935136717207570[5] = 0;
   out_3040935136717207570[6] = 1;
   out_3040935136717207570[7] = 0;
   out_3040935136717207570[8] = 0;
}
void h_24(double *state, double *unused, double *out_5122675272519993604) {
   out_5122675272519993604[0] = state[4];
   out_5122675272519993604[1] = state[5];
}
void H_24(double *state, double *unused, double *out_868285537711708004) {
   out_868285537711708004[0] = 0;
   out_868285537711708004[1] = 0;
   out_868285537711708004[2] = 0;
   out_868285537711708004[3] = 0;
   out_868285537711708004[4] = 1;
   out_868285537711708004[5] = 0;
   out_868285537711708004[6] = 0;
   out_868285537711708004[7] = 0;
   out_868285537711708004[8] = 0;
   out_868285537711708004[9] = 0;
   out_868285537711708004[10] = 0;
   out_868285537711708004[11] = 0;
   out_868285537711708004[12] = 0;
   out_868285537711708004[13] = 0;
   out_868285537711708004[14] = 1;
   out_868285537711708004[15] = 0;
   out_868285537711708004[16] = 0;
   out_868285537711708004[17] = 0;
}
void h_30(double *state, double *unused, double *out_8595044134008836117) {
   out_8595044134008836117[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5559268095224456197) {
   out_5559268095224456197[0] = 0;
   out_5559268095224456197[1] = 0;
   out_5559268095224456197[2] = 0;
   out_5559268095224456197[3] = 0;
   out_5559268095224456197[4] = 1;
   out_5559268095224456197[5] = 0;
   out_5559268095224456197[6] = 0;
   out_5559268095224456197[7] = 0;
   out_5559268095224456197[8] = 0;
}
void h_26(double *state, double *unused, double *out_2716758646450121029) {
   out_2716758646450121029[0] = state[7];
}
void H_26(double *state, double *unused, double *out_700568182156848654) {
   out_700568182156848654[0] = 0;
   out_700568182156848654[1] = 0;
   out_700568182156848654[2] = 0;
   out_700568182156848654[3] = 0;
   out_700568182156848654[4] = 0;
   out_700568182156848654[5] = 0;
   out_700568182156848654[6] = 0;
   out_700568182156848654[7] = 1;
   out_700568182156848654[8] = 0;
}
void h_27(double *state, double *unused, double *out_6629402236988649268) {
   out_6629402236988649268[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7782862166408399414) {
   out_7782862166408399414[0] = 0;
   out_7782862166408399414[1] = 0;
   out_7782862166408399414[2] = 0;
   out_7782862166408399414[3] = 1;
   out_7782862166408399414[4] = 0;
   out_7782862166408399414[5] = 0;
   out_7782862166408399414[6] = 0;
   out_7782862166408399414[7] = 0;
   out_7782862166408399414[8] = 0;
}
void h_29(double *state, double *unused, double *out_9024150106849758680) {
   out_9024150106849758680[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6069499439538848381) {
   out_6069499439538848381[0] = 0;
   out_6069499439538848381[1] = 1;
   out_6069499439538848381[2] = 0;
   out_6069499439538848381[3] = 0;
   out_6069499439538848381[4] = 0;
   out_6069499439538848381[5] = 0;
   out_6069499439538848381[6] = 0;
   out_6069499439538848381[7] = 0;
   out_6069499439538848381[8] = 0;
}
void h_28(double *state, double *unused, double *out_6815668569060056703) {
   out_6815668569060056703[0] = state[0];
}
void H_28(double *state, double *unused, double *out_987100422469317807) {
   out_987100422469317807[0] = 1;
   out_987100422469317807[1] = 0;
   out_987100422469317807[2] = 0;
   out_987100422469317807[3] = 0;
   out_987100422469317807[4] = 0;
   out_987100422469317807[5] = 0;
   out_987100422469317807[6] = 0;
   out_987100422469317807[7] = 0;
   out_987100422469317807[8] = 0;
}
void h_31(double *state, double *unused, double *out_2233954507100698029) {
   out_2233954507100698029[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1326776284390200130) {
   out_1326776284390200130[0] = 0;
   out_1326776284390200130[1] = 0;
   out_1326776284390200130[2] = 0;
   out_1326776284390200130[3] = 0;
   out_1326776284390200130[4] = 0;
   out_1326776284390200130[5] = 0;
   out_1326776284390200130[6] = 0;
   out_1326776284390200130[7] = 0;
   out_1326776284390200130[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4219125439312905275) {
  err_fun(nom_x, delta_x, out_4219125439312905275);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8640473231899427946) {
  inv_err_fun(nom_x, true_x, out_8640473231899427946);
}
void car_H_mod_fun(double *state, double *out_6403599997404205727) {
  H_mod_fun(state, out_6403599997404205727);
}
void car_f_fun(double *state, double dt, double *out_8494721620328521265) {
  f_fun(state,  dt, out_8494721620328521265);
}
void car_F_fun(double *state, double dt, double *out_7925611537347342114) {
  F_fun(state,  dt, out_7925611537347342114);
}
void car_h_25(double *state, double *unused, double *out_8319850071724330228) {
  h_25(state, unused, out_8319850071724330228);
}
void car_H_25(double *state, double *unused, double *out_3040935136717207570) {
  H_25(state, unused, out_3040935136717207570);
}
void car_h_24(double *state, double *unused, double *out_5122675272519993604) {
  h_24(state, unused, out_5122675272519993604);
}
void car_H_24(double *state, double *unused, double *out_868285537711708004) {
  H_24(state, unused, out_868285537711708004);
}
void car_h_30(double *state, double *unused, double *out_8595044134008836117) {
  h_30(state, unused, out_8595044134008836117);
}
void car_H_30(double *state, double *unused, double *out_5559268095224456197) {
  H_30(state, unused, out_5559268095224456197);
}
void car_h_26(double *state, double *unused, double *out_2716758646450121029) {
  h_26(state, unused, out_2716758646450121029);
}
void car_H_26(double *state, double *unused, double *out_700568182156848654) {
  H_26(state, unused, out_700568182156848654);
}
void car_h_27(double *state, double *unused, double *out_6629402236988649268) {
  h_27(state, unused, out_6629402236988649268);
}
void car_H_27(double *state, double *unused, double *out_7782862166408399414) {
  H_27(state, unused, out_7782862166408399414);
}
void car_h_29(double *state, double *unused, double *out_9024150106849758680) {
  h_29(state, unused, out_9024150106849758680);
}
void car_H_29(double *state, double *unused, double *out_6069499439538848381) {
  H_29(state, unused, out_6069499439538848381);
}
void car_h_28(double *state, double *unused, double *out_6815668569060056703) {
  h_28(state, unused, out_6815668569060056703);
}
void car_H_28(double *state, double *unused, double *out_987100422469317807) {
  H_28(state, unused, out_987100422469317807);
}
void car_h_31(double *state, double *unused, double *out_2233954507100698029) {
  h_31(state, unused, out_2233954507100698029);
}
void car_H_31(double *state, double *unused, double *out_1326776284390200130) {
  H_31(state, unused, out_1326776284390200130);
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
