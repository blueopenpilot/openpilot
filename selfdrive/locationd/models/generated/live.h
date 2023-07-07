#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_3708082936522869015);
void live_err_fun(double *nom_x, double *delta_x, double *out_5079214496152152084);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2899404642962030040);
void live_H_mod_fun(double *state, double *out_4638679797694418825);
void live_f_fun(double *state, double dt, double *out_8277743396465754765);
void live_F_fun(double *state, double dt, double *out_1885647329573244761);
void live_h_4(double *state, double *unused, double *out_4451881289460363506);
void live_H_4(double *state, double *unused, double *out_4025749365786923618);
void live_h_9(double *state, double *unused, double *out_1103722527081085284);
void live_H_9(double *state, double *unused, double *out_1136887813506844276);
void live_h_10(double *state, double *unused, double *out_8716633330559095776);
void live_H_10(double *state, double *unused, double *out_1838688048423808346);
void live_h_12(double *state, double *unused, double *out_1689726910422765345);
void live_H_12(double *state, double *unused, double *out_3404650340739329951);
void live_h_35(double *state, double *unused, double *out_4927334228421517648);
void live_H_35(double *state, double *unused, double *out_659087308414316242);
void live_h_32(double *state, double *unused, double *out_6954754509268440276);
void live_H_32(double *state, double *unused, double *out_6258497721281398475);
void live_h_13(double *state, double *unused, double *out_1463074775442288091);
void live_H_13(double *state, double *unused, double *out_1766128220440700911);
void live_h_14(double *state, double *unused, double *out_1103722527081085284);
void live_H_14(double *state, double *unused, double *out_1136887813506844276);
void live_h_33(double *state, double *unused, double *out_7924105795683185330);
void live_H_33(double *state, double *unused, double *out_2491469696224541362);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}