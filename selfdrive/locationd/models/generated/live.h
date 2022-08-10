#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_2853646381397912552);
void live_err_fun(double *nom_x, double *delta_x, double *out_5549535972816791141);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3634911457740076995);
void live_H_mod_fun(double *state, double *out_8058031850184286508);
void live_f_fun(double *state, double dt, double *out_3451938404271731457);
void live_F_fun(double *state, double dt, double *out_8702098680640092912);
void live_h_4(double *state, double *unused, double *out_5694133910873872321);
void live_H_4(double *state, double *unused, double *out_249419679862994927);
void live_h_9(double *state, double *unused, double *out_8951512206140578977);
void live_H_9(double *state, double *unused, double *out_7536638615127442397);
void live_h_10(double *state, double *unused, double *out_3736522873776615970);
void live_H_10(double *state, double *unused, double *out_8225287402177473278);
void live_h_12(double *state, double *unused, double *out_7383623745651831242);
void live_H_12(double *state, double *unused, double *out_6131838697179738069);
void live_h_31(double *state, double *unused, double *out_4598324046571392693);
void live_H_31(double *state, double *unused, double *out_7784633047839092488);
void live_h_32(double *state, double *unused, double *out_9083885247086774980);
void live_H_32(double *state, double *unused, double *out_7775629603022004632);
void live_h_13(double *state, double *unused, double *out_1269344890707383669);
void live_H_13(double *state, double *unused, double *out_6958956562019023321);
void live_h_14(double *state, double *unused, double *out_8951512206140578977);
void live_H_14(double *state, double *unused, double *out_7536638615127442397);
void live_h_33(double *state, double *unused, double *out_7446583057376225623);
void live_H_33(double *state, double *unused, double *out_4634076043200234884);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}