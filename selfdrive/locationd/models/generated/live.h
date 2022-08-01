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
void live_H(double *in_vec, double *out_4775142192765997536);
void live_err_fun(double *nom_x, double *delta_x, double *out_7188947162080794543);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6709545991995454978);
void live_H_mod_fun(double *state, double *out_2512347716181414513);
void live_f_fun(double *state, double dt, double *out_558080539079513041);
void live_F_fun(double *state, double dt, double *out_3407541149745054442);
void live_h_4(double *state, double *unused, double *out_612680352297535661);
void live_H_4(double *state, double *unused, double *out_5857182501871774522);
void live_h_9(double *state, double *unused, double *out_9018246780206215414);
void live_H_9(double *state, double *unused, double *out_1700014765516997039);
void live_h_10(double *state, double *unused, double *out_7053400825981112501);
void live_H_10(double *state, double *unused, double *out_1559623721113281845);
void live_h_12(double *state, double *unused, double *out_1816199632969465721);
void live_H_12(double *state, double *unused, double *out_6478281526919368189);
void live_h_35(double *state, double *unused, double *out_1949698199427841660);
void live_H_35(double *state, double *unused, double *out_9222899514465169718);
void live_h_32(double *state, double *unused, double *out_8859212339278433371);
void live_H_32(double *state, double *unused, double *out_8911222864356936870);
void live_h_13(double *state, double *unused, double *out_3741846921883364777);
void live_H_13(double *state, double *unused, double *out_3418955876817387491);
void live_h_14(double *state, double *unused, double *out_9018246780206215414);
void live_H_14(double *state, double *unused, double *out_1700014765516997039);
void live_h_33(double *state, double *unused, double *out_2316324009978192357);
void live_H_33(double *state, double *unused, double *out_6072342509826312114);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}