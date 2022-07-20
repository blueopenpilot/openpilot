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
void live_H(double *in_vec, double *out_3373784032601531946);
void live_err_fun(double *nom_x, double *delta_x, double *out_288273506848810992);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8217505745392555803);
void live_H_mod_fun(double *state, double *out_5186881794809025466);
void live_f_fun(double *state, double dt, double *out_2822127974921232432);
void live_F_fun(double *state, double dt, double *out_2562236053699961213);
void live_h_4(double *state, double *unused, double *out_3907418252982751204);
void live_H_4(double *state, double *unused, double *out_5788742369033568687);
void live_h_9(double *state, double *unused, double *out_1248987609065954858);
void live_H_9(double *state, double *unused, double *out_5370782769411535459);
void live_h_10(double *state, double *unused, double *out_3490979936692034853);
void live_H_10(double *state, double *unused, double *out_2953442199040047421);
void live_h_12(double *state, double *unused, double *out_199853913988466881);
void live_H_12(double *state, double *unused, double *out_4990873390993532437);
void live_h_31(double *state, double *unused, double *out_6016174519881823486);
void live_H_31(double *state, double *unused, double *out_2245310358668518728);
void live_h_32(double *state, double *unused, double *out_4968256648042067766);
void live_H_32(double *state, double *unused, double *out_3555994013539093830);
void live_h_13(double *state, double *unused, double *out_7146606626339293941);
void live_H_13(double *state, double *unused, double *out_4724261469866057608);
void live_h_14(double *state, double *unused, double *out_1248987609065954858);
void live_H_14(double *state, double *unused, double *out_5370782769411535459);
void live_h_33(double *state, double *unused, double *out_322095597797900011);
void live_H_33(double *state, double *unused, double *out_905246645970338876);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}