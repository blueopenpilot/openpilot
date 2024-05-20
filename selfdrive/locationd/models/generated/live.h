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
void live_H(double *in_vec, double *out_4388803601849331947);
void live_err_fun(double *nom_x, double *delta_x, double *out_3555431415370206371);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6932391034376981081);
void live_H_mod_fun(double *state, double *out_6428357737830499900);
void live_f_fun(double *state, double dt, double *out_5010884756149981100);
void live_F_fun(double *state, double dt, double *out_5462900171218415481);
void live_h_4(double *state, double *unused, double *out_8521146831324346307);
void live_H_4(double *state, double *unused, double *out_646925349847414095);
void live_h_9(double *state, double *unused, double *out_3271024027003602113);
void live_H_9(double *state, double *unused, double *out_405735703217823450);
void live_h_10(double *state, double *unused, double *out_1286114781269543531);
void live_H_10(double *state, double *unused, double *out_1661832607362786689);
void live_h_12(double *state, double *unused, double *out_3235757136444442762);
void live_H_12(double *state, double *unused, double *out_4372531058184547700);
void live_h_31(double *state, double *unused, double *out_6330147658988393173);
void live_H_31(double *state, double *unused, double *out_7118094090509561409);
void live_h_32(double *state, double *unused, double *out_7818626693935651502);
void live_H_32(double *state, double *unused, double *out_2850901492465933750);
void live_h_13(double *state, double *unused, double *out_348349551587194058);
void live_H_13(double *state, double *unused, double *out_4680582928945569914);
void live_h_14(double *state, double *unused, double *out_3271024027003602113);
void live_H_14(double *state, double *unused, double *out_405735703217823450);
void live_h_33(double *state, double *unused, double *out_4696108645965244668);
void live_H_33(double *state, double *unused, double *out_5870293712164050885);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}