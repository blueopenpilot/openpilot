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
void live_H(double *in_vec, double *out_7209398253506453359);
void live_err_fun(double *nom_x, double *delta_x, double *out_994833721824308553);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4632021964642794099);
void live_H_mod_fun(double *state, double *out_7538307877216892783);
void live_f_fun(double *state, double dt, double *out_7035001334654508742);
void live_F_fun(double *state, double dt, double *out_8435949222489011857);
void live_h_4(double *state, double *unused, double *out_7668675417006875721);
void live_H_4(double *state, double *unused, double *out_4384325897146155976);
void live_h_9(double *state, double *unused, double *out_5789007276214887841);
void live_H_9(double *state, double *unused, double *out_4625515543775746621);
void live_h_10(double *state, double *unused, double *out_1062144383174807384);
void live_H_10(double *state, double *unused, double *out_220754046540745774);
void live_h_12(double *state, double *unused, double *out_8870844239671695412);
void live_H_12(double *state, double *unused, double *out_9042961768531433845);
void live_h_35(double *state, double *unused, double *out_8307517526811411061);
void live_H_35(double *state, double *unused, double *out_7750987954518763352);
void live_h_32(double *state, double *unused, double *out_5890826362672519379);
void live_H_32(double *state, double *unused, double *out_8754917781857779463);
void live_h_13(double *state, double *unused, double *out_8615652077084552254);
void live_H_13(double *state, double *unused, double *out_7107178695677299642);
void live_h_14(double *state, double *unused, double *out_5789007276214887841);
void live_H_14(double *state, double *unused, double *out_4625515543775746621);
void live_h_33(double *state, double *unused, double *out_6685733422617726486);
void live_H_33(double *state, double *unused, double *out_7545199114551930660);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}