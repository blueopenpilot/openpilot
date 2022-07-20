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
void live_H(double *in_vec, double *out_5454284932020681649);
void live_err_fun(double *nom_x, double *delta_x, double *out_7704279630345638266);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2513211039557519125);
void live_H_mod_fun(double *state, double *out_7435501040416024686);
void live_f_fun(double *state, double dt, double *out_4212934802868580656);
void live_F_fun(double *state, double dt, double *out_3120508586470747969);
void live_h_4(double *state, double *unused, double *out_1854745610775522181);
void live_H_4(double *state, double *unused, double *out_6353941194245937609);
void live_h_9(double *state, double *unused, double *out_953217976374423093);
void live_H_9(double *state, double *unused, double *out_6112751547616346964);
void live_h_10(double *state, double *unused, double *out_3419261070243978164);
void live_H_10(double *state, double *unused, double *out_197181602305955466);
void live_h_12(double *state, double *unused, double *out_1346848399456022453);
void live_H_12(double *state, double *unused, double *out_1334484786213975814);
void live_h_31(double *state, double *unused, double *out_1889562335172587885);
void live_H_31(double *state, double *unused, double *out_2987279136873330233);
void live_h_32(double *state, double *unused, double *out_5132173494779662762);
void live_H_32(double *state, double *unused, double *out_7328283912606437121);
void live_h_13(double *state, double *unused, double *out_5059856578377808030);
void live_H_13(double *state, double *unused, double *out_8212200507733000639);
void live_h_14(double *state, double *unused, double *out_953217976374423093);
void live_H_14(double *state, double *unused, double *out_6112751547616346964);
void live_h_33(double *state, double *unused, double *out_2217696332301297631);
void live_H_33(double *state, double *unused, double *out_163277867765527371);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}