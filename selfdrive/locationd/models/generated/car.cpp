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
void err_fun(double *nom_x, double *delta_x, double *out_1093100257688424678) {
   out_1093100257688424678[0] = delta_x[0] + nom_x[0];
   out_1093100257688424678[1] = delta_x[1] + nom_x[1];
   out_1093100257688424678[2] = delta_x[2] + nom_x[2];
   out_1093100257688424678[3] = delta_x[3] + nom_x[3];
   out_1093100257688424678[4] = delta_x[4] + nom_x[4];
   out_1093100257688424678[5] = delta_x[5] + nom_x[5];
   out_1093100257688424678[6] = delta_x[6] + nom_x[6];
   out_1093100257688424678[7] = delta_x[7] + nom_x[7];
   out_1093100257688424678[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7128642225905965818) {
   out_7128642225905965818[0] = -nom_x[0] + true_x[0];
   out_7128642225905965818[1] = -nom_x[1] + true_x[1];
   out_7128642225905965818[2] = -nom_x[2] + true_x[2];
   out_7128642225905965818[3] = -nom_x[3] + true_x[3];
   out_7128642225905965818[4] = -nom_x[4] + true_x[4];
   out_7128642225905965818[5] = -nom_x[5] + true_x[5];
   out_7128642225905965818[6] = -nom_x[6] + true_x[6];
   out_7128642225905965818[7] = -nom_x[7] + true_x[7];
   out_7128642225905965818[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5507642244994157908) {
   out_5507642244994157908[0] = 1.0;
   out_5507642244994157908[1] = 0;
   out_5507642244994157908[2] = 0;
   out_5507642244994157908[3] = 0;
   out_5507642244994157908[4] = 0;
   out_5507642244994157908[5] = 0;
   out_5507642244994157908[6] = 0;
   out_5507642244994157908[7] = 0;
   out_5507642244994157908[8] = 0;
   out_5507642244994157908[9] = 0;
   out_5507642244994157908[10] = 1.0;
   out_5507642244994157908[11] = 0;
   out_5507642244994157908[12] = 0;
   out_5507642244994157908[13] = 0;
   out_5507642244994157908[14] = 0;
   out_5507642244994157908[15] = 0;
   out_5507642244994157908[16] = 0;
   out_5507642244994157908[17] = 0;
   out_5507642244994157908[18] = 0;
   out_5507642244994157908[19] = 0;
   out_5507642244994157908[20] = 1.0;
   out_5507642244994157908[21] = 0;
   out_5507642244994157908[22] = 0;
   out_5507642244994157908[23] = 0;
   out_5507642244994157908[24] = 0;
   out_5507642244994157908[25] = 0;
   out_5507642244994157908[26] = 0;
   out_5507642244994157908[27] = 0;
   out_5507642244994157908[28] = 0;
   out_5507642244994157908[29] = 0;
   out_5507642244994157908[30] = 1.0;
   out_5507642244994157908[31] = 0;
   out_5507642244994157908[32] = 0;
   out_5507642244994157908[33] = 0;
   out_5507642244994157908[34] = 0;
   out_5507642244994157908[35] = 0;
   out_5507642244994157908[36] = 0;
   out_5507642244994157908[37] = 0;
   out_5507642244994157908[38] = 0;
   out_5507642244994157908[39] = 0;
   out_5507642244994157908[40] = 1.0;
   out_5507642244994157908[41] = 0;
   out_5507642244994157908[42] = 0;
   out_5507642244994157908[43] = 0;
   out_5507642244994157908[44] = 0;
   out_5507642244994157908[45] = 0;
   out_5507642244994157908[46] = 0;
   out_5507642244994157908[47] = 0;
   out_5507642244994157908[48] = 0;
   out_5507642244994157908[49] = 0;
   out_5507642244994157908[50] = 1.0;
   out_5507642244994157908[51] = 0;
   out_5507642244994157908[52] = 0;
   out_5507642244994157908[53] = 0;
   out_5507642244994157908[54] = 0;
   out_5507642244994157908[55] = 0;
   out_5507642244994157908[56] = 0;
   out_5507642244994157908[57] = 0;
   out_5507642244994157908[58] = 0;
   out_5507642244994157908[59] = 0;
   out_5507642244994157908[60] = 1.0;
   out_5507642244994157908[61] = 0;
   out_5507642244994157908[62] = 0;
   out_5507642244994157908[63] = 0;
   out_5507642244994157908[64] = 0;
   out_5507642244994157908[65] = 0;
   out_5507642244994157908[66] = 0;
   out_5507642244994157908[67] = 0;
   out_5507642244994157908[68] = 0;
   out_5507642244994157908[69] = 0;
   out_5507642244994157908[70] = 1.0;
   out_5507642244994157908[71] = 0;
   out_5507642244994157908[72] = 0;
   out_5507642244994157908[73] = 0;
   out_5507642244994157908[74] = 0;
   out_5507642244994157908[75] = 0;
   out_5507642244994157908[76] = 0;
   out_5507642244994157908[77] = 0;
   out_5507642244994157908[78] = 0;
   out_5507642244994157908[79] = 0;
   out_5507642244994157908[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8894714846457968717) {
   out_8894714846457968717[0] = state[0];
   out_8894714846457968717[1] = state[1];
   out_8894714846457968717[2] = state[2];
   out_8894714846457968717[3] = state[3];
   out_8894714846457968717[4] = state[4];
   out_8894714846457968717[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8894714846457968717[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8894714846457968717[7] = state[7];
   out_8894714846457968717[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1917681428035716347) {
   out_1917681428035716347[0] = 1;
   out_1917681428035716347[1] = 0;
   out_1917681428035716347[2] = 0;
   out_1917681428035716347[3] = 0;
   out_1917681428035716347[4] = 0;
   out_1917681428035716347[5] = 0;
   out_1917681428035716347[6] = 0;
   out_1917681428035716347[7] = 0;
   out_1917681428035716347[8] = 0;
   out_1917681428035716347[9] = 0;
   out_1917681428035716347[10] = 1;
   out_1917681428035716347[11] = 0;
   out_1917681428035716347[12] = 0;
   out_1917681428035716347[13] = 0;
   out_1917681428035716347[14] = 0;
   out_1917681428035716347[15] = 0;
   out_1917681428035716347[16] = 0;
   out_1917681428035716347[17] = 0;
   out_1917681428035716347[18] = 0;
   out_1917681428035716347[19] = 0;
   out_1917681428035716347[20] = 1;
   out_1917681428035716347[21] = 0;
   out_1917681428035716347[22] = 0;
   out_1917681428035716347[23] = 0;
   out_1917681428035716347[24] = 0;
   out_1917681428035716347[25] = 0;
   out_1917681428035716347[26] = 0;
   out_1917681428035716347[27] = 0;
   out_1917681428035716347[28] = 0;
   out_1917681428035716347[29] = 0;
   out_1917681428035716347[30] = 1;
   out_1917681428035716347[31] = 0;
   out_1917681428035716347[32] = 0;
   out_1917681428035716347[33] = 0;
   out_1917681428035716347[34] = 0;
   out_1917681428035716347[35] = 0;
   out_1917681428035716347[36] = 0;
   out_1917681428035716347[37] = 0;
   out_1917681428035716347[38] = 0;
   out_1917681428035716347[39] = 0;
   out_1917681428035716347[40] = 1;
   out_1917681428035716347[41] = 0;
   out_1917681428035716347[42] = 0;
   out_1917681428035716347[43] = 0;
   out_1917681428035716347[44] = 0;
   out_1917681428035716347[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1917681428035716347[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1917681428035716347[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1917681428035716347[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1917681428035716347[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1917681428035716347[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1917681428035716347[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1917681428035716347[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1917681428035716347[53] = -9.8000000000000007*dt;
   out_1917681428035716347[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1917681428035716347[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1917681428035716347[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1917681428035716347[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1917681428035716347[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1917681428035716347[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1917681428035716347[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1917681428035716347[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1917681428035716347[62] = 0;
   out_1917681428035716347[63] = 0;
   out_1917681428035716347[64] = 0;
   out_1917681428035716347[65] = 0;
   out_1917681428035716347[66] = 0;
   out_1917681428035716347[67] = 0;
   out_1917681428035716347[68] = 0;
   out_1917681428035716347[69] = 0;
   out_1917681428035716347[70] = 1;
   out_1917681428035716347[71] = 0;
   out_1917681428035716347[72] = 0;
   out_1917681428035716347[73] = 0;
   out_1917681428035716347[74] = 0;
   out_1917681428035716347[75] = 0;
   out_1917681428035716347[76] = 0;
   out_1917681428035716347[77] = 0;
   out_1917681428035716347[78] = 0;
   out_1917681428035716347[79] = 0;
   out_1917681428035716347[80] = 1;
}
void h_25(double *state, double *unused, double *out_3196602313497850705) {
   out_3196602313497850705[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6748259161611645143) {
   out_6748259161611645143[0] = 0;
   out_6748259161611645143[1] = 0;
   out_6748259161611645143[2] = 0;
   out_6748259161611645143[3] = 0;
   out_6748259161611645143[4] = 0;
   out_6748259161611645143[5] = 0;
   out_6748259161611645143[6] = 1;
   out_6748259161611645143[7] = 0;
   out_6748259161611645143[8] = 0;
}
void h_24(double *state, double *unused, double *out_5915140457056590213) {
   out_5915140457056590213[0] = state[4];
   out_5915140457056590213[1] = state[5];
}
void H_24(double *state, double *unused, double *out_94053703503459386) {
   out_94053703503459386[0] = 0;
   out_94053703503459386[1] = 0;
   out_94053703503459386[2] = 0;
   out_94053703503459386[3] = 0;
   out_94053703503459386[4] = 1;
   out_94053703503459386[5] = 0;
   out_94053703503459386[6] = 0;
   out_94053703503459386[7] = 0;
   out_94053703503459386[8] = 0;
   out_94053703503459386[9] = 0;
   out_94053703503459386[10] = 0;
   out_94053703503459386[11] = 0;
   out_94053703503459386[12] = 0;
   out_94053703503459386[13] = 0;
   out_94053703503459386[14] = 1;
   out_94053703503459386[15] = 0;
   out_94053703503459386[16] = 0;
   out_94053703503459386[17] = 0;
}
void h_30(double *state, double *unused, double *out_3471796375782356594) {
   out_3471796375782356594[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6877598108754885213) {
   out_6877598108754885213[0] = 0;
   out_6877598108754885213[1] = 0;
   out_6877598108754885213[2] = 0;
   out_6877598108754885213[3] = 0;
   out_6877598108754885213[4] = 1;
   out_6877598108754885213[5] = 0;
   out_6877598108754885213[6] = 0;
   out_6877598108754885213[7] = 0;
   out_6877598108754885213[8] = 0;
}
void h_26(double *state, double *unused, double *out_6076201696591054245) {
   out_6076201696591054245[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7956981593223850249) {
   out_7956981593223850249[0] = 0;
   out_7956981593223850249[1] = 0;
   out_7956981593223850249[2] = 0;
   out_7956981593223850249[3] = 0;
   out_7956981593223850249[4] = 0;
   out_7956981593223850249[5] = 0;
   out_7956981593223850249[6] = 0;
   out_7956981593223850249[7] = 1;
   out_7956981593223850249[8] = 0;
}
void h_27(double *state, double *unused, double *out_9074887801056565793) {
   out_9074887801056565793[0] = state[3];
}
void H_27(double *state, double *unused, double *out_9052361420555310124) {
   out_9052361420555310124[0] = 0;
   out_9052361420555310124[1] = 0;
   out_9052361420555310124[2] = 0;
   out_9052361420555310124[3] = 1;
   out_9052361420555310124[4] = 0;
   out_9052361420555310124[5] = 0;
   out_9052361420555310124[6] = 0;
   out_9052361420555310124[7] = 0;
   out_9052361420555310124[8] = 0;
}
void h_29(double *state, double *unused, double *out_8473469643443202239) {
   out_8473469643443202239[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7681019926284690459) {
   out_7681019926284690459[0] = 0;
   out_7681019926284690459[1] = 1;
   out_7681019926284690459[2] = 0;
   out_7681019926284690459[3] = 0;
   out_7681019926284690459[4] = 0;
   out_7681019926284690459[5] = 0;
   out_7681019926284690459[6] = 0;
   out_7681019926284690459[7] = 0;
   out_7681019926284690459[8] = 0;
}
void h_28(double *state, double *unused, double *out_4145949056817660876) {
   out_4145949056817660876[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8802093875859534906) {
   out_8802093875859534906[0] = 1;
   out_8802093875859534906[1] = 0;
   out_8802093875859534906[2] = 0;
   out_8802093875859534906[3] = 0;
   out_8802093875859534906[4] = 0;
   out_8802093875859534906[5] = 0;
   out_8802093875859534906[6] = 0;
   out_8802093875859534906[7] = 0;
   out_8802093875859534906[8] = 0;
}
void h_31(double *state, double *unused, double *out_6753481078947960919) {
   out_6753481078947960919[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6717613199734684715) {
   out_6717613199734684715[0] = 0;
   out_6717613199734684715[1] = 0;
   out_6717613199734684715[2] = 0;
   out_6717613199734684715[3] = 0;
   out_6717613199734684715[4] = 0;
   out_6717613199734684715[5] = 0;
   out_6717613199734684715[6] = 0;
   out_6717613199734684715[7] = 0;
   out_6717613199734684715[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1093100257688424678) {
  err_fun(nom_x, delta_x, out_1093100257688424678);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7128642225905965818) {
  inv_err_fun(nom_x, true_x, out_7128642225905965818);
}
void car_H_mod_fun(double *state, double *out_5507642244994157908) {
  H_mod_fun(state, out_5507642244994157908);
}
void car_f_fun(double *state, double dt, double *out_8894714846457968717) {
  f_fun(state,  dt, out_8894714846457968717);
}
void car_F_fun(double *state, double dt, double *out_1917681428035716347) {
  F_fun(state,  dt, out_1917681428035716347);
}
void car_h_25(double *state, double *unused, double *out_3196602313497850705) {
  h_25(state, unused, out_3196602313497850705);
}
void car_H_25(double *state, double *unused, double *out_6748259161611645143) {
  H_25(state, unused, out_6748259161611645143);
}
void car_h_24(double *state, double *unused, double *out_5915140457056590213) {
  h_24(state, unused, out_5915140457056590213);
}
void car_H_24(double *state, double *unused, double *out_94053703503459386) {
  H_24(state, unused, out_94053703503459386);
}
void car_h_30(double *state, double *unused, double *out_3471796375782356594) {
  h_30(state, unused, out_3471796375782356594);
}
void car_H_30(double *state, double *unused, double *out_6877598108754885213) {
  H_30(state, unused, out_6877598108754885213);
}
void car_h_26(double *state, double *unused, double *out_6076201696591054245) {
  h_26(state, unused, out_6076201696591054245);
}
void car_H_26(double *state, double *unused, double *out_7956981593223850249) {
  H_26(state, unused, out_7956981593223850249);
}
void car_h_27(double *state, double *unused, double *out_9074887801056565793) {
  h_27(state, unused, out_9074887801056565793);
}
void car_H_27(double *state, double *unused, double *out_9052361420555310124) {
  H_27(state, unused, out_9052361420555310124);
}
void car_h_29(double *state, double *unused, double *out_8473469643443202239) {
  h_29(state, unused, out_8473469643443202239);
}
void car_H_29(double *state, double *unused, double *out_7681019926284690459) {
  H_29(state, unused, out_7681019926284690459);
}
void car_h_28(double *state, double *unused, double *out_4145949056817660876) {
  h_28(state, unused, out_4145949056817660876);
}
void car_H_28(double *state, double *unused, double *out_8802093875859534906) {
  H_28(state, unused, out_8802093875859534906);
}
void car_h_31(double *state, double *unused, double *out_6753481078947960919) {
  h_31(state, unused, out_6753481078947960919);
}
void car_H_31(double *state, double *unused, double *out_6717613199734684715) {
  H_31(state, unused, out_6717613199734684715);
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
