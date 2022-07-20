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
void live_H(double *in_vec, double *out_566270153146909501);
void live_err_fun(double *nom_x, double *delta_x, double *out_405319358855397025);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8156052676264259734);
void live_H_mod_fun(double *state, double *out_1600453883539340733);
void live_f_fun(double *state, double dt, double *out_1599125439170075910);
void live_F_fun(double *state, double dt, double *out_4929940219983912006);
void live_h_4(double *state, double *unused, double *out_6855841923841202583);
void live_H_4(double *state, double *unused, double *out_304980740677685349);
void live_h_9(double *state, double *unused, double *out_5190997632426750204);
void live_H_9(double *state, double *unused, double *out_546170387307275994);
void live_h_10(double *state, double *unused, double *out_3111393795685932532);
void live_H_10(double *state, double *unused, double *out_4348913938610489057);
void live_h_12(double *state, double *unused, double *out_1501816444684676061);
void live_H_12(double *state, double *unused, double *out_5324437148709647144);
void live_h_31(double *state, double *unused, double *out_6598056757083510281);
void live_H_31(double *state, double *unused, double *out_8070000181034660853);
void live_h_32(double *state, double *unused, double *out_3960579636252661886);
void live_H_32(double *state, double *unused, double *out_7757378486147215825);
void live_h_13(double *state, double *unused, double *out_5188972723401387423);
void live_H_13(double *state, double *unused, double *out_795624100583925901);
void live_h_14(double *state, double *unused, double *out_5190997632426750204);
void live_H_14(double *state, double *unused, double *out_546170387307275994);
void live_h_33(double *state, double *unused, double *out_4947597745478682718);
void live_H_33(double *state, double *unused, double *out_7226186888036033159);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}